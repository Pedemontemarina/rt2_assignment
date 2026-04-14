
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include "custom_interfaces/action/navigate_to.hpp"



using NavigateTo = custom_interfaces::action::NavigateTo;
using GoalHandleNavigateTo = rclcpp_action::ClientGoalHandle<NavigateTo>;

namespace assignment
{

class NavClient : public rclcpp::Node
{
public:
    explicit NavClient(const rclcpp::NodeOptions & options) : Node("nav_client", options),
    goal_handle_(nullptr) 
    {
        // subscriber goal
        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&NavClient::goal_callback, this, std::placeholders::_1));

        // subscriber cancel
        sub_cancel_ = this->create_subscription<std_msgs::msg::Empty>(
            "cancel_goal", 10,
            std::bind(&NavClient::cancel_callback, this, std::placeholders::_1));

        // action client
        action_client_ = rclcpp_action::create_client<NavigateTo>(this, "navigate_to");

        RCLCPP_INFO(this->get_logger(), "NavClient started, waiting for goals...");
    }

private:

    rclcpp_action::Client<NavigateTo>::SharedPtr action_client_;
    GoalHandleNavigateTo::SharedPtr goal_handle_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_cancel_;

    
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // estraggo x e y
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;

        // estraggo theta dal quaternione
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        double theta = atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz));

        RCLCPP_INFO(this->get_logger(),
            "New goal received: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);

        // se c'è già un goal attivo lo cancello prima
        if (goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "Cancelling previous goal...");
            action_client_->async_cancel_goal(goal_handle_,
                [this, x, y, theta](action_msgs::srv::CancelGoal::Response::SharedPtr) {
                    RCLCPP_INFO(this->get_logger(), "Previous goal cancelled, sending new one...");
                    goal_handle_ = nullptr;
                    send_goal(x, y, theta);
                });
            return;
        }

        send_goal(x, y, theta);
    }

    void cancel_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        if (!goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
            return;
        }

        action_client_->async_cancel_goal(goal_handle_,
            [this](action_msgs::srv::CancelGoal::Response::SharedPtr) {
                RCLCPP_INFO(this->get_logger(), "Goal successfully cancelled");
                goal_handle_ = nullptr;
            });
    }

    void send_goal(double x, double y, double theta)
    {
        // controllo che il server sia disponibile
        if (!action_client_->action_server_is_ready()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not ready!");
            return;
        }

        // costruisco il goal
        auto goal_msg = NavigateTo::Goal();
        goal_msg.x     = x;
        goal_msg.y     = y;
        goal_msg.theta = theta;

        // collego le callback
        auto options = rclcpp_action::Client<NavigateTo>::SendGoalOptions();

        options.goal_response_callback =
            [this](const GoalHandleNavigateTo::SharedPtr & goal_handle) {
                goal_response_callback(goal_handle);
            };

        options.feedback_callback =
            [this](GoalHandleNavigateTo::SharedPtr,
                   const std::shared_ptr<const NavigateTo::Feedback> feedback) {
                this->feedback_callback(feedback);
            };

        options.result_callback =
            [this](const GoalHandleNavigateTo::WrappedResult & result) {
                result_callback(result);
            };

        action_client_->async_send_goal(goal_msg, options);
        RCLCPP_INFO(this->get_logger(), "Goal sent to server");
    }

    // server ha accettato o rifiutato il goal
    void goal_response_callback(const GoalHandleNavigateTo::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server!");
        goal_handle_ = goal_handle; // salvo per eventuale cancel
    }

    // feedback periodico dal server 

    void feedback_callback(const std::shared_ptr<const NavigateTo::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(),
        "[Feedback] dist=%.3f  angle=%.3f",
        feedback->distance_to_goal,
        feedback->angle_to_goal);
}


    // goal completato (successo, fallimento o cancellato)
    void result_callback(const GoalHandleNavigateTo::WrappedResult & result)
    {
        goal_handle_ = nullptr; // goal finito, resetto

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal reached! %s",
                    result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal was cancelled");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
};

} 

RCLCPP_COMPONENTS_REGISTER_NODE(assignment::NavClient)  