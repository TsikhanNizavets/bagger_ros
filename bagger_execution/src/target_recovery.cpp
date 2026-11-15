/*
backward and then approach targes from different position
check if u don't fall into dumping area while moving backward
*/

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <limits>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_action/action_server.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "bagger_interfaces/action/target_recovery.hpp"
#include "bagger_interfaces/msg/target_transform_array.hpp"


class TargetRecovery : public rclcpp::Node
{
public:
    TargetRecovery() : Node("target_recovery")
    {
        using namespace std::placeholders;

        pub_cmd_vel = create_publisher<geometry_msgs::msg::TwistStamped> (
            "cmd_vel_auto",
            10
        );

        sub_targets = create_subscription<bagger_interfaces::msg::TargetTransformArray> (
            "/target_position_estimation/target_transform_array",
            10,
            std::bind(&TargetRecovery::targets_callback, this, _1)
        );

        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

        action_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        action_server = rclcpp_action::create_server<bagger_interfaces::action::TargetRecovery> (
            this,
            "target_recovery",
            std::bind(&TargetRecovery::goal_callback, this, _1, _2),
            std::bind(&TargetRecovery::cancel_callback, this, _1),
            std::bind(&TargetRecovery::execute_callback, this, _1),
            rcl_action_server_get_default_options(),
            action_group
        );
    }
private:
    static constexpr double linear_vel = -0.1;
    std::optional<geometry_msgs::msg::TransformStamped> target = std::nullopt;
    bool is_executing = false;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel;

    rclcpp::Subscription<bagger_interfaces::msg::TargetTransformArray>::SharedPtr sub_targets;

    rclcpp_action::Server<bagger_interfaces::action::TargetRecovery>::SharedPtr action_server;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;

    rclcpp::CallbackGroup::SharedPtr action_group;
    std::mutex state_mutex;

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const bagger_interfaces::action::TargetRecovery::Goal> goal_request)
    {
        std::lock_guard<std::mutex> lock(state_mutex);

        if (is_executing)
        {
            return rclcpp_action::GoalResponse::REJECT;
        };
        is_executing = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bagger_interfaces::action::TargetRecovery>> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received cancel request!");
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    void execute_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bagger_interfaces::action::TargetRecovery>> goal_handle)
    {
        write_target(std::nullopt);
        std::shared_ptr<bagger_interfaces::action::TargetRecovery::Result> result = std::make_shared<bagger_interfaces::action::TargetRecovery::Result>();

        const double distance_to_drive = goal_handle->get_goal()->driven_distance.data + 0.1;
        double driven_distance = 0;

        geometry_msgs::msg::TransformStamped pose = tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
        const double start_x = pose.transform.translation.x;
        const double start_y = pose.transform.translation.y;

        geometry_msgs::msg::TwistStamped msg_vel;
        msg_vel.header.frame_id = "base_link";

        while (driven_distance <= distance_to_drive)            // no protection agains endless loop
        {
            if (read_target().has_value())
            {
                msg_vel.header.stamp = get_clock()->now();
                msg_vel.twist.linear.x = 0.0;

                pub_cmd_vel->publish(msg_vel);
                // report success, return target_position
                result->target = *read_target();
                goal_handle->succeed(result);

                finish_execution();
                return;
            }
            
            msg_vel.header.stamp = get_clock()->now();
            msg_vel.twist.linear.x = linear_vel;

            pub_cmd_vel->publish(msg_vel);

            pose = tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
            driven_distance = std::sqrt(
                (pose.transform.translation.x - start_x) * (pose.transform.translation.x - start_x) +
                (pose.transform.translation.y - start_y) * (pose.transform.translation.y - start_y)
            );

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        msg_vel.header.stamp = get_clock()->now();
        msg_vel.twist.linear.x = 0.0;

        pub_cmd_vel->publish(msg_vel);

        // report failure
        goal_handle->abort(result);
        finish_execution();
        return;
    };

    void targets_callback(bagger_interfaces::msg::TargetTransformArray::ConstSharedPtr msg)
    {
        if (!msg->target.empty())
        {
            double min_distance_to_target = std::numeric_limits<float>::max();
            double max_distance_to_target = 2.5;
            std::optional<geometry_msgs::msg::TransformStamped> target = std::nullopt;

            for (std::vector<geometry_msgs::msg::TransformStamped>::const_iterator it = msg->target.begin(); it != msg->target.end(); ++it)
            {
                double distance_to_target = std::hypot(
                    it->transform.translation.x,
                    it->transform.translation.y,
                    it->transform.translation.z
                );
                if (distance_to_target < min_distance_to_target && distance_to_target < max_distance_to_target)
                {
                    min_distance_to_target = distance_to_target;
                    target = *it;
                }
            }

            if (target)
            {
                tf_buffer->setTransform(*target, "target_position_estimation");

                write_target(
                    tf_buffer->lookupTransform(
                        "odom",
                        target->child_frame_id,
                        rclcpp::Time(target->header.stamp)
                    )
                );
            }
        }
    };

    std::optional<geometry_msgs::msg::TransformStamped> read_target()
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        return target;
    }

    void write_target(std::optional<geometry_msgs::msg::TransformStamped> data)
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        target = data;
    }

    void finish_execution()
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        is_executing = false;
    };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        std::shared_ptr<TargetRecovery> node = std::make_shared<TargetRecovery>();

        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

        executor.add_node(node);
        executor.spin();
    }
    catch (...)
    {
        rclcpp::shutdown();
        throw;
    }

    rclcpp::shutdown();
    return 0;
}