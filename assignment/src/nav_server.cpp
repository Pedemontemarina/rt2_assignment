#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/utils.h>

#include "custom_interfaces/action/navigate_to.hpp"


using NavigateTo  = custom_interfaces::action::NavigateTo;
using GoalHandle  = rclcpp_action::ServerGoalHandle<NavigateTo>;
using namespace std::chrono_literals;

namespace assignment
{

class NavServer : public rclcpp::Node
{
public:
    explicit NavServer(const rclcpp::NodeOptions & options)
    : Node("nav_server", options)
    {
        //callback group
        cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        //publisher comandi
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // action server con 3 callback (goal, cancel, accepted)
        action_server_ = rclcpp_action::create_server<NavigateTo>(
            this,
            "navigate_to",
            std::bind(&NavServer::goal_callback,     this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavServer::cancel_callback,   this, std::placeholders::_1),
            std::bind(&NavServer::accepted_callback, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            cb_group_);

        // TF
        tf_buffer_       = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        //ascolta e aggiorna continuamente il buffer con le trasformazioni che riceve
        tf_listener_     = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "NavServer started");
    }

private:
    rclcpp::CallbackGroup::SharedPtr                         cb_group_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_pub_;
    rclcpp_action::Server<NavigateTo>::SharedPtr             action_server_;

    std::shared_ptr<tf2_ros::Buffer>             tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;

    std::mutex lock_;

    // traccia il goal in esecuzione
    std::shared_ptr<GoalHandle> current_goal_handle_;
    std::shared_ptr<GoalHandle> preempt_requested_for_;

    bool aligned_to_zero_;


    // ── goal callback ─────────────────────────────────────────────────────────
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateTo::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received goal: x=%.2f y=%.2f theta=%.2f",
            goal->x, goal->y, goal->theta);
          
        std::lock_guard<std::mutex> guard(lock_);
        if (current_goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "Preempting previous goal");
            preempt_requested_for_ = current_goal_handle_;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // ── cancel callback ───────────────────────────────────────────────────────
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<GoalHandle> /*goal_handle*/)
    {
        RCLCPP_WARN(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // ── accepted callback ─────────────────────────────────────────────────────
    // quando il goal viene accettato, viene eseguita questa callback in un thread separatot
    void accepted_callback(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{
            std::bind(&NavServer::execute_callback, this, std::placeholders::_1),
            goal_handle
        }.detach();
    }

    // ── execute callback ──────────────────────────────────────────────────────
    void execute_callback(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const double target_x     = goal_handle->get_goal()->x;
        const double target_y     = goal_handle->get_goal()->y;
        const double target_theta = goal_handle->get_goal()->theta;

        RCLCPP_INFO(this->get_logger(),
            "Executing: x=%.2f y=%.2f theta=%.2f",
            target_x, target_y, target_theta);

        {
            std::lock_guard<std::mutex> guard(lock_);
            current_goal_handle_ = goal_handle;
        }

        aligned_to_zero_ = false; 

        auto feedback_msg = std::make_shared<NavigateTo::Feedback>();
        auto result_msg   = std::make_shared<NavigateTo::Result>();
        geometry_msgs::msg::Twist vel_msg;

        constexpr double Kp           = 0.5;
        constexpr double Kp_th        = 0.4;
        constexpr double V_MAX        = 0.4;
        constexpr double W_MAX        = 1.0;
        constexpr double POS_THRESH   = 0.05;
        constexpr double ANGLE_THRESH = 0.05;

        // funzione di clamp per limitare i comandi, in prattica limita il valore di v a [-V_MAX, V_MAX] e w a [-W_MAX, W_MAX]
        auto clamp = [](double v, double lim) {
            return std::max(-lim, std::min(lim, v));
        };


        while (rclcpp::ok())
        {
            // ── cancel ────────────────────────────────────────────────────
            if (goal_handle->is_canceling()) {
                vel_msg = geometry_msgs::msg::Twist{};
                cmd_vel_pub_->publish(vel_msg);
                result_msg->message = "Cancelled by client";
                goal_handle->canceled(result_msg);

                std::lock_guard<std::mutex> guard(lock_);
                if (current_goal_handle_ == goal_handle)
                    current_goal_handle_ = nullptr;
                return;
            }

            // ── preemption ────────────────────────────────────────────────
            {
                std::lock_guard<std::mutex> guard(lock_);
                bool preempt_me = (preempt_requested_for_ &&
                                   goal_handle->get_goal_id() ==
                                   preempt_requested_for_->get_goal_id());
                if (preempt_me) {
                    vel_msg = geometry_msgs::msg::Twist{};
                    cmd_vel_pub_->publish(vel_msg);
                    result_msg->message = "Preempted by newer goal";
                    goal_handle->abort(result_msg);
                    if (current_goal_handle_ == goal_handle)
                        current_goal_handle_ = nullptr;
                    preempt_requested_for_ = nullptr;
                    return;
                }
            }

            // ── TF────────────────────────────────
            
            geometry_msgs::msg::TransformStamped odom_baselink;
            try {
                odom_baselink = tf_buffer_->lookupTransform("odom","base_link", tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF not available: %s", ex.what());
                std::this_thread::sleep_for(100ms);
                continue;
            }

            // posizione del robot in odom
            double rx = odom_baselink.transform.translation.x;
            double ry = odom_baselink.transform.translation.y;

            tf2::Quaternion q(
                odom_baselink.transform.rotation.x,
                odom_baselink.transform.rotation.y,
                odom_baselink.transform.rotation.z,
                odom_baselink.transform.rotation.w
            );

            double rtheta = tf2::getYaw(q);

            //errori di orientamento rispetto al goal
            double ex = target_x - rx;
            double ey = target_y - ry;
            double distance_to_goal = std::hypot(ex, ey);


            // heading error: angolo verso il goal relativo all'orientamento del robot
            double heading_error = std::atan2(ey, ex) - rtheta;
            while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

            // errore di orientamento finale
            double desired_theta_error = target_theta - rtheta;
            while (desired_theta_error >  M_PI) desired_theta_error -= 2.0 * M_PI;
            while (desired_theta_error < -M_PI) desired_theta_error += 2.0 * M_PI;

            
            // ── feedback ──────────────────────────────────────────────────
            feedback_msg->distance_to_goal = distance_to_goal;
            feedback_msg->angle_to_goal    = heading_error; 
            goal_handle->publish_feedback(feedback_msg);



            // ── controllo ─────────────────────────────────────────────────
            // 1) ruoto verso il goal
            if (distance_to_goal > POS_THRESH && std::abs(heading_error) > ANGLE_THRESH) {
                vel_msg.linear.x  = 0.0;
                vel_msg.linear.y  = 0.0;
                vel_msg.angular.z = clamp(Kp_th * heading_error, W_MAX);
            }
            // 2) avanzo verso il goal con correzione heading
            else if (distance_to_goal > POS_THRESH) {
                vel_msg.linear.x  = clamp(Kp * distance_to_goal, V_MAX);
                vel_msg.linear.y  = 0.0;
                vel_msg.angular.z = clamp(Kp_th * heading_error, W_MAX);
            }
            // 3) allineo con l'orientamento finale
            else if (std::abs(desired_theta_error) > ANGLE_THRESH) {
                vel_msg.linear.x  = 0.0;
                vel_msg.linear.y  = 0.0;
                vel_msg.angular.z = clamp(Kp_th * desired_theta_error, W_MAX);
            }
            // 4) goal raggiunto
            else {
                vel_msg = geometry_msgs::msg::Twist{};
                cmd_vel_pub_->publish(vel_msg);
                result_msg->message = "Goal reached!";
                goal_handle->succeed(result_msg);

                std::lock_guard<std::mutex> guard(lock_);
                if (current_goal_handle_ == goal_handle)
                    current_goal_handle_ = nullptr;

                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                return;
            }

            cmd_vel_pub_->publish(vel_msg);
            std::this_thread::sleep_for(100ms);

        }        
    }
};

}; // namespace assignment

RCLCPP_COMPONENTS_REGISTER_NODE(assignment::NavServer)