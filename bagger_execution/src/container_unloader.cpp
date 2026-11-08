/*
Stops bucket wheel while unloading due to highest priority.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bagger_interfaces/action/unload_container.hpp"
#include "control_msgs/msg/multi_dof_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"


class ContainerUnloader : public rclcpp::Node 
{
public:
    ContainerUnloader() : Node("container_unloader") 
    {
        pub_container = create_publisher<std_msgs::msg::Float64> (
            "/container_cmd",
            10
        );
        
        // Publish zero velocity to the bucket wheel to block it inside of timer, which starts in callback
        pub_cmd_wheel = create_publisher<control_msgs::msg::MultiDOFCommand> (
            "/bucket_wheel_cmd_priority",
            10
        );

        // Check if the bucket wheel has stopped
        sub_joint_states = create_subscription<sensor_msgs::msg::JointState> (
            "joint_states",
            10,
            std::bind(
                &ContainerUnloader::joint_state_callback,
                this,
                std::placeholders::_1
            )
        );

        action_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        action_server = rclcpp_action::create_server<bagger_interfaces::action::UnloadContainer> (
            this,
            "unload_container",
            std::bind(
                &ContainerUnloader::goal_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            ),
            std::bind(
                &ContainerUnloader::cancel_callback,
                this,
                std::placeholders::_1
            ),
            std::bind(
                &ContainerUnloader::execute_callback,
                this,
                std::placeholders::_1
            ),
            rcl_action_server_get_default_options(),
            action_group
        );

        RCLCPP_INFO(get_logger(), "Action Server ready on /unload_container");
    }

private:
    double wheel_speed = 0.0f;
    double container_position = 0.0f;
    double home_angle = 0.0;
    double target_angle = -2.75;
    double dumping_duration = 3.0;
    bool is_executing = false;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_container;
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr pub_cmd_wheel;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states;

    rclcpp_action::Server<bagger_interfaces::action::UnloadContainer>::SharedPtr action_server;

    rclcpp::TimerBase::SharedPtr block_wheel_timer = nullptr;

    rclcpp::CallbackGroup::SharedPtr action_group;
    std::mutex state_mutex;

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const bagger_interfaces::action::UnloadContainer::Goal> goal_request)
    {
        std::lock_guard<std::mutex> lock(state_mutex);

        if (is_executing)
        {
            return rclcpp_action::GoalResponse::REJECT;
        };
        is_executing = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    rclcpp_action::CancelResponse cancel_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<bagger_interfaces::action::UnloadContainer>> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received cancel request!");
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    void execute_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<bagger_interfaces::action::UnloadContainer>> goal_handle)
    {
        try
        {
            RCLCPP_INFO(get_logger(), "Executing unloading action...");

            block_wheel_timer = create_timer(
                std::chrono::milliseconds(100),
                [this]() {timer_block_wheel();}
            );

            std::shared_ptr<bagger_interfaces::action::UnloadContainer::Feedback> feedback_msg = std::make_shared<bagger_interfaces::action::UnloadContainer::Feedback>();
            feedback_msg->current_state = "WAITING_FOR_WHEEL_TO_STOP";
            goal_handle->publish_feedback(feedback_msg);

            while (std::abs(read_wheel_speed()) > 0.05f)
            {
                if (goal_handle->is_canceling())
                {
                    cancel_unloading(
                        goal_handle, "Canceled while stopping wheel"
                    );
                    return;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            feedback_msg->current_state = "TILTING CONTAINER";
            goal_handle->publish_feedback(feedback_msg);

            std_msgs::msg::Float64 msg;
            msg.data = target_angle;
            pub_container->publish(msg);

            while (target_angle - read_container_position() < -0.04)
            {
                if (goal_handle->is_canceling())
                {
                    return_home_and_cancel_unloading(
                        goal_handle, "Canceled while tilting container"
                    );
                    return;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            feedback_msg->current_state = "DUMPING";
            goal_handle->publish_feedback(feedback_msg);

            rclcpp::Time start_time = get_clock()->now();

            while ((get_clock()->now() - start_time).seconds() < dumping_duration)
            {
                if (goal_handle->is_canceling())
                {
                    return_home_and_cancel_unloading(
                        goal_handle, "Canceled while dumping"
                    );
                    return;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            feedback_msg->current_state = "RETURNING CONTAINER";
            goal_handle->publish_feedback(feedback_msg);

            msg.data = home_angle;
            pub_container->publish(msg);

            while (home_angle - read_container_position() < -0.04)
            {
                if (goal_handle->is_canceling())
                {
                    cancel_unloading(
                        goal_handle, "Canceled while returning container"
                    );
                    return;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            block_wheel_timer->cancel();

            std::shared_ptr<bagger_interfaces::action::UnloadContainer::Result> result = std::make_shared<bagger_interfaces::action::UnloadContainer::Result>();
            result->message = "FINISHED SUCCESSFULLY";
            goal_handle->succeed(result);

            finish_execution();
            return;
        }

        catch (const std::exception& error)
        {
            std::string message = std::string("Unload failed: ") + error.what();
            RCLCPP_ERROR(get_logger(), "%s", message.c_str());

            std_msgs::msg::Float64 msg;
            msg.data = home_angle;
            pub_container->publish(msg);

            while (std::abs(home_angle - read_container_position()) > 0.04)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            block_wheel_timer->cancel();

            std::shared_ptr<bagger_interfaces::action::UnloadContainer::Result> result = std::make_shared<bagger_interfaces::action::UnloadContainer::Result>();
            result->message = message;
            goal_handle->abort(result);
            
            finish_execution();
            return;
        }
    };

    void return_home_and_cancel_unloading(std::shared_ptr<rclcpp_action::ServerGoalHandle<bagger_interfaces::action::UnloadContainer>> goal_handle, std::string message)
    {
        std_msgs::msg::Float64 msg;
        msg.data = home_angle;
        pub_container->publish(msg);

        cancel_unloading(goal_handle, message);
        return;
    };

    void cancel_unloading(std::shared_ptr<rclcpp_action::ServerGoalHandle<bagger_interfaces::action::UnloadContainer>> goal_handle, std::string message)
    {
        std::shared_ptr<bagger_interfaces::action::UnloadContainer::Feedback> feedback_msg = std::make_shared<bagger_interfaces::action::UnloadContainer::Feedback>();
        feedback_msg->current_state = "CANCELING";
        goal_handle->publish_feedback(feedback_msg);

        while (std::abs(home_angle - read_container_position()) > 0.04)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        block_wheel_timer->cancel();

        std::shared_ptr<bagger_interfaces::action::UnloadContainer::Result> result = std::make_shared<bagger_interfaces::action::UnloadContainer::Result>();
        result->message = message;
        goal_handle->canceled(result);

        finish_execution();            
        return;
    };

    void joint_state_callback(sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        
        std::vector<std::string>::const_iterator it = std::find(
            msg->name.begin(),
            msg->name.end(),
            "bucket_wheel_joint"
        );
        if (it != msg->name.end())
        {
            const int idx = std::distance(msg->name.begin(), it);
            wheel_speed = msg->velocity[idx];
        }

        it = std::find(
            msg->name.begin(),
            msg->name.end(),
            "container_joint"
        );
        if (it != msg->name.end())
        {
            const int idx = std::distance(msg->name.begin(), it);
            container_position = msg->position[idx];
        }
    };

    void timer_block_wheel()
    {
        control_msgs::msg::MultiDOFCommand msg;
        msg.dof_names = {"bucket_wheel_joint"};
        msg.values = {0.0};
        pub_cmd_wheel->publish(msg);
    };

    double read_wheel_speed()
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        return wheel_speed;
    }

    double read_container_position()
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        return container_position;
    }

    void finish_execution()
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        is_executing = false;
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try {
        std::shared_ptr<ContainerUnloader> node = std::make_shared<ContainerUnloader>();

        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

        executor.add_node(node);
        executor.spin();
    }
    catch (...) {
        rclcpp::shutdown();
        throw;
    }

    rclcpp::shutdown();
    return 0;
}