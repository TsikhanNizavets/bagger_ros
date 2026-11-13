/*
Action. Drives backward, when distance jumps, stops, unloads, and drives back in the safety zone
*/

#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_action/action_server.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "bagger_interfaces/action/execute_dumping.hpp"
#include "bagger_interfaces/action/unload_container.hpp"


class DumpingExecutor : public rclcpp::Node
{
using UnloadContainer = bagger_interfaces::action::UnloadContainer;
using ExecuteDumping = bagger_interfaces::action::ExecuteDumping;

template<typename Action>
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Action>;

template<typename Action>
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Action>;

public:
    DumpingExecutor() : Node("dumping_executor")
    {
        using namespace std::placeholders;

        pub_cmd_vel = create_publisher<geometry_msgs::msg::TwistStamped> (
            "/cmd_vel_auto",
            10
        );

        sub_odom = create_subscription<nav_msgs::msg::Odometry> (
            "/odom",
            10,
            std::bind(&DumpingExecutor::odom_callback, this, _1)
        );

        sub_left_drop_off = create_subscription<sensor_msgs::msg::LaserScan> (
            "/left_drop_off/scan",
            10,
            std::bind(&DumpingExecutor::drop_off_callback, this, _1)
        );

        sub_right_drop_off = create_subscription<sensor_msgs::msg::LaserScan> (
            "/right_drop_off/scan",
            10,
            std::bind(&DumpingExecutor::drop_off_callback, this, _1)
        );

        action_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        action_server = rclcpp_action::create_server<ExecuteDumping> (
            this,
            "execute_dumping",
            std::bind(&DumpingExecutor::goal_callback, this, _1, _2),
            std::bind(&DumpingExecutor::cancel_callback, this, _1),
            std::bind(&DumpingExecutor::execute_callback, this, _1),
            rcl_action_server_get_default_options(),
            action_group
        );

        action_client = rclcpp_action::create_client<UnloadContainer>(
            this,
            "/unload_container",
            action_group
        );

        RCLCPP_INFO_STREAM(get_logger(), "DumpingExecutor ready on /execute_dumping");
    };
private:
    double drop_off_distance = 0.0;
    static constexpr double drive_speed = 0.1;
    std::optional<nav_msgs::msg::Odometry> latest_odom = std::nullopt;
    bool is_executing = false;
    bool child_cancel_sent = false;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_left_drop_off;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_right_drop_off;

    rclcpp_action::Server<ExecuteDumping>::SharedPtr action_server;

    rclcpp_action::Client<UnloadContainer>::SharedPtr action_client;

    rclcpp::TimerBase::SharedPtr return_timer;

    rclcpp::CallbackGroup::SharedPtr action_group;
    std::mutex state_mutex;

    ClientGoalHandle<UnloadContainer>::SharedPtr child_goal_handle = nullptr;

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID& uuid, ExecuteDumping::Goal::ConstSharedPtr goal_request)
    {
        std::lock_guard<std::mutex> lock(state_mutex);

        if (is_executing)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Reject goal: Already executing.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        is_executing = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<ServerGoalHandle<ExecuteDumping>> goal_handle)
    {
        std::lock_guard<std::mutex> lock(state_mutex);

        RCLCPP_INFO_STREAM(get_logger(), "Received cancel reqeust");
        if (child_goal_handle && !child_cancel_sent)
        {
            child_cancel_sent = true;
            action_client->async_cancel_goal(child_goal_handle);
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    };
    
    void execute_callback(const std::shared_ptr<ServerGoalHandle<ExecuteDumping>> goal_handle)
    {
        ExecuteDumping::Feedback::SharedPtr feedback_msg = std::make_shared<ExecuteDumping::Feedback>();
        ExecuteDumping::Result::SharedPtr result = std::make_shared<ExecuteDumping::Result>();

        try
        {
            RCLCPP_INFO_STREAM(get_logger(), "Executing dumping action...");

            // START DOCKING
            feedback_msg->current_state = "DOCKING";
            goal_handle->publish_feedback(feedback_msg);

            std::function<bool(double)> condition = [this](double) {
                std::lock_guard<std::mutex> lock(state_mutex); 
                return drop_off_distance < 0.7;
            };

            std::optional<double> driven_distance_while_docking = drive_while(condition, goal_handle, -drive_speed);

            if (!driven_distance_while_docking)
            {
                std::string message = "Canceled while docking";
                RCLCPP_INFO_STREAM(get_logger(), message);
                finish_execution(message, [goal_handle](ExecuteDumping::Result::SharedPtr result){
                    goal_handle->canceled(result);
                });
                return;
            }

            // CALL UNLOADING SERVER
            feedback_msg->current_state = "UNLOADING";
            goal_handle->publish_feedback(feedback_msg);

            if (!action_client->wait_for_action_server(std::chrono::milliseconds(100)))
            {
                std::string message = "Server /unload_container not available";
                RCLCPP_WARN_STREAM(get_logger(), message);
                finish_execution(message, [goal_handle](ExecuteDumping::Result::SharedPtr result){
                    goal_handle->abort(result);
                });
                return;
            }

            rclcpp_action::Client<UnloadContainer>::SendGoalOptions options;

            options.goal_response_callback = 
                [this, parent_goal_handle = goal_handle](ClientGoalHandle<UnloadContainer>::SharedPtr child_goal_handle)
                {
                    if (!child_goal_handle)
                    {
                        std::string message = "Unload goal rejected";
                        RCLCPP_WARN_STREAM(get_logger(), message);
                        finish_execution(message, [parent_goal_handle](ExecuteDumping::Result::SharedPtr result){
                            parent_goal_handle->abort(result);
                        });
                        return;
                    }

                    {
                        std::lock_guard<std::mutex> lock(state_mutex);

                        this->child_goal_handle = std::move(child_goal_handle);

                        if (parent_goal_handle->is_canceling())
                        {
                            child_cancel_sent = true;
                            action_client->async_cancel_goal(this->child_goal_handle);
                        }
                    }
                };

            options.feedback_callback =
                [parent_goal_handle = goal_handle](ClientGoalHandle<UnloadContainer>::SharedPtr child_goal_handle, UnloadContainer::Feedback::ConstSharedPtr feedback)
                {
                    ExecuteDumping::Feedback::SharedPtr forwarded = std::make_shared<ExecuteDumping::Feedback>();
                    forwarded->current_state = "DUMPING: " + feedback->current_state;
                    parent_goal_handle->publish_feedback(forwarded);
                };

            options.result_callback =
                [this, parent_goal_handle = goal_handle, driven_distance_while_docking](const ClientGoalHandle<UnloadContainer>::WrappedResult& response)
                {
                    
                    if (response.code == rclcpp_action::ResultCode::ABORTED)
                    {
                        std::string message = response.result->message;
                        RCLCPP_ERROR_STREAM(get_logger(), message);
                        finish_execution(message, [parent_goal_handle](ExecuteDumping::Result::SharedPtr result){
                            parent_goal_handle->abort(result);
                        });
                        return;
                    }
                    if (response.code == rclcpp_action::ResultCode::CANCELED)
                    {
                        std::string message = response.result->message;
                        RCLCPP_INFO_STREAM(get_logger(), message);
                        finish_execution(message, [parent_goal_handle](ExecuteDumping::Result::SharedPtr result){
                            parent_goal_handle->canceled(result);
                        });
                        return;
                    }

                    // return result_callback fast to prevent deadlock: 
                    // ros2 locks mutex for result_callback and process drive_while too long ->
                    // another ros2 internal callback waits for this mutex -> 
                    // two threads are occupied -> no free thread for odom_callback -> 
                    // endless driving
                    return_timer = create_timer(
                        std::chrono::milliseconds(100),
                        [this, parent_goal_handle, driven_distance_while_docking]()
                        {
                            try
                            {
                                return_timer->cancel();

                                ExecuteDumping::Feedback::SharedPtr feedback = std::make_shared<ExecuteDumping::Feedback>();
                                feedback->current_state = "RETURNING";
                                parent_goal_handle->publish_feedback(feedback);

                                std::function<bool(double)> condition = [this, driven_distance_while_docking](double driven_distance) {
                                    std::lock_guard<std::mutex> lock(state_mutex);
                                    return driven_distance < *driven_distance_while_docking;
                                };

                                if (!drive_while(condition, parent_goal_handle, drive_speed))
                                {
                                    std::string message = "Canceled while returning";
                                    RCLCPP_INFO_STREAM(get_logger(), message);
                                    finish_execution(message, [parent_goal_handle](ExecuteDumping::Result::SharedPtr result){
                                        parent_goal_handle->canceled(result);
                                    });
                                    return;
                                }

                                std::string message = "Dumping completed successfully";
                                RCLCPP_INFO_STREAM(get_logger(), message);
                                finish_execution(message, [parent_goal_handle](ExecuteDumping::Result::SharedPtr result){
                                    parent_goal_handle->succeed(result);
                                });
                            }
                            catch(const std::exception& error)
                            {
                                std::string message = std::string("Error: ") + error.what();
                                RCLCPP_ERROR_STREAM(get_logger(), message);
                                finish_execution(message, [parent_goal_handle](ExecuteDumping::Result::SharedPtr result){
                                    parent_goal_handle->abort(result);
                                });
                                return;
                            }
                        },
                        action_group
                    );

                    return;
                };

            if (goal_handle->is_canceling())
            {
                std::string message = "Canceled while unloading";
                RCLCPP_INFO_STREAM(get_logger(), message);
                finish_execution(message, [goal_handle](ExecuteDumping::Result::SharedPtr result){
                    goal_handle->canceled(result);
                });
                return;
            }

            action_client->async_send_goal(UnloadContainer::Goal{}, options);
            return;
        }
        catch(const std::exception& error)
        {
            std::string message = std::string("Dumping failed: ") + error.what();
            RCLCPP_ERROR_STREAM(get_logger(), message);
            finish_execution(message, [goal_handle](ExecuteDumping::Result::SharedPtr result){
                goal_handle->abort(result);
            });
            return;
        }
    };

    // std::nullopt - canceled during execution; double value - sucessfully finished
    std::optional<double> drive_while(const std::function<bool(double)>& condition,
                const std::shared_ptr<ServerGoalHandle<ExecuteDumping>>& parent_goal_handle,
                const double& linear_velocity)
    {
        geometry_msgs::msg::TwistStamped cmd_msg;
        cmd_msg.header.frame_id = "base_link";

        const geometry_msgs::msg::Point start_position = [this]() {
            std::lock_guard<std::mutex> lock(state_mutex); 
            return latest_odom->pose.pose.position;
        }();

        const double start_x = start_position.x;
        const double start_y = start_position.y;

        double driven_distance = 0.0;

        while (condition(driven_distance))
        {
            if (parent_goal_handle->is_canceling())
            {
                cmd_msg.header.stamp = get_clock()->now();
                cmd_msg.twist.linear.x = 0.0;
                pub_cmd_vel->publish(cmd_msg);

                while (std::abs([this]() {std::lock_guard<std::mutex> lock(state_mutex); return latest_odom->twist.twist.linear.x;}()) > 0.0001)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                return std::nullopt;
            }

            cmd_msg.header.stamp = get_clock()->now();
            cmd_msg.twist.linear.x = linear_velocity;
            pub_cmd_vel->publish(cmd_msg);

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                driven_distance = std::hypot(
                    latest_odom->pose.pose.position.x - start_x,
                    latest_odom->pose.pose.position.y - start_y
                );
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        cmd_msg.header.stamp = get_clock()->now();
        cmd_msg.twist.linear.x = 0.0;
        pub_cmd_vel->publish(cmd_msg);

        return driven_distance;
    };

    void odom_callback(std::shared_ptr<const nav_msgs::msg::Odometry> msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        latest_odom = *msg;
    };

    void drop_off_callback(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        drop_off_distance = msg->ranges[0];
    }

    void finish_execution(const std::string& message, const std::function<void(ExecuteDumping::Result::SharedPtr)>& complete)
    {
        std::lock_guard<std::mutex> lock(state_mutex);

        geometry_msgs::msg::TwistStamped stop_msg;
        stop_msg.header.frame_id = "base_link";
        stop_msg.header.stamp = get_clock()->now();
        stop_msg.twist.linear.x = 0.0;

        pub_cmd_vel->publish(stop_msg);

        ExecuteDumping::Result::SharedPtr result = std::make_shared<ExecuteDumping::Result>();
        result->message = message;
        complete(result);

        child_cancel_sent = false;
        child_goal_handle = nullptr;
        is_executing = false;
        return;
    };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        std::shared_ptr<DumpingExecutor> node = std::make_shared<DumpingExecutor>();

        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

        executor.add_node(node);
        executor.spin();
    }
    catch(...)
    {
        rclcpp::shutdown();
        throw;
    }

    rclcpp::shutdown();
    return 0;
}