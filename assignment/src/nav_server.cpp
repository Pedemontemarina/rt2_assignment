#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

        /* //subscriber odometria
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&NavServer::odom_callback, this, std::placeholders::_1),
            sub_opts);
 */
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
        
        //per pubblicare il frma del movimento del robot,broadcaster per odom → base_link
        /* odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this); */
        //per pubblicare la posizione del goal relativa al robot!!!!!!
        goal_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "NavServer started");
    }

private:
    rclcpp::CallbackGroup::SharedPtr                         cb_group_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Server<NavigateTo>::SharedPtr             action_server_;

    std::shared_ptr<tf2_ros::Buffer>             tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> goal_broadcaster_;
     std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

    std::mutex lock_;

    // traccia il goal in esecuzione
    std::shared_ptr<GoalHandle> current_goal_handle_;
    std::shared_ptr<GoalHandle> preempt_requested_for_;

    // pubblica il frame "goal" nell'albero TF
    void publish_goal_frame(double x, double y, double theta)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id  = "goal";

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = std::sin(theta / 2.0);
        t.transform.rotation.w = std::cos(theta / 2.0);

        goal_broadcaster_->sendTransform(t);
    }

    // ── odom callback ─────────────────────────────────────────────────────────
    /* void odom_callback(nav_msgs::msg::Odometry::UniquePtr msg)
    {
        //  aggiorno la trasformazione odom → base_link (robot frame)
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg->header.stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_link";

        tf.transform.translation.x = msg->pose.pose.position.x;
        tf.transform.translation.y = msg->pose.pose.position.y;
        tf.transform.translation.z = 0.0;

        tf.transform.rotation = msg->pose.pose.orientation;

        odom_broadcaster_->sendTransform(tf);
    } */

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

        // pubblica il frame goal in TF rispetto a /odom fixed!!!
        publish_goal_frame(target_x, target_y, target_theta);

        auto feedback_msg = std::make_shared<NavigateTo::Feedback>();
        auto result_msg   = std::make_shared<NavigateTo::Result>();
        geometry_msgs::msg::Twist vel_msg;

        constexpr double Kp           = 0.5;
        constexpr double Kp_th        = 1.0;
        constexpr double V_MAX        = 0.4;
        constexpr double W_MAX        = 1.0;
        constexpr double POS_THRESH   = 0.05;
        constexpr double ANGLE_THRESH = 0.05;

        // funzione di clamp per limitare i comandi, in prattica limita il valore di v a [-V_MAX, V_MAX] e w a [-W_MAX, W_MAX]
        auto clamp = [](double v, double lim) {
            return std::max(-lim, std::min(lim, v));
        };

        while (true)
        {
            // ── 1. cancel ────────────────────────────────────────────────────
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

            // ── 2. preemption ────────────────────────────────────────────────
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

            // ── 3. calcolo dell'errore di posizione ed orientamento rispetto al goal ────────────────────────────────
            // prendo direttamente la trasformazione tra goal e base_link, che mi da direttamente l'errore di posizione ed orientamento
            
            geometry_msgs::msg::TransformStamped goal_baselink;
            try {
                goal_baselink = tf_buffer_->lookupTransform("base_link","goal", tf2::TimePointZero,
                                                            tf2::durationFromSec(0.1)   );
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF non disponibile: %s", ex.what());
                std::this_thread::sleep_for(100ms);
                continue;
            }

            //errori di posizione ed orientamento rispetto al goal
            double ex = goal_baselink.transform.translation.x;
            double ey = goal_baselink.transform.translation.y;
            double distance_to_goal = std::hypot(ex, ey);

            double e_theta = atan2(ey, ex);
            while (e_theta >  M_PI) e_theta -= 2.0 * M_PI;
            while (e_theta < -M_PI) e_theta += 2.0 * M_PI;

            // ── 5. feedback ──────────────────────────────────────────────────
            feedback_msg->distance_to_goal = distance_to_goal;
            feedback_msg->angle_to_goal    = e_theta;
            goal_handle->publish_feedback(feedback_msg);

            // ── 6. condizione di successo ────────────────────────────────────
            if (distance_to_goal < POS_THRESH && std::abs(e_theta) < ANGLE_THRESH) {
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

            // ── 7. controllo P ───────────────────────────────────────────────
            vel_msg.linear.x  = clamp(Kp    * ex,      V_MAX);
            vel_msg.linear.y  = clamp(Kp    * ey,      V_MAX);
            vel_msg.angular.z = clamp(Kp_th * e_theta, W_MAX);
            cmd_vel_pub_->publish(vel_msg);

            std::this_thread::sleep_for(100ms);
        }
    }
};

}  // namespace assignment

RCLCPP_COMPONENTS_REGISTER_NODE(assignment::NavServer)