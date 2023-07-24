#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <deadlock/action/some_action.hpp>
#include <rclcpp_action/create_client.hpp>

class ClientNode : public rclcpp::Node
{
public:

    using Action = deadlock::action::SomeAction;

    using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;


//     rclcpp_action::Server<cm_interfaces::action::NodeStateChange>::SharedPtr cmdNodeStateActionServer;
    ClientNode() : Node("DeadlockClient")
    {
        actionClient = rclcpp_action::create_client<Action>(this, "/deadlock/example" );

        RCLCPP_INFO_STREAM(get_logger(), "Waiting for action server");
        actionClient->wait_for_action_server();
        RCLCPP_INFO_STREAM(get_logger(), "Action server active");

        actionClient->async_cancel_all_goals();

        //note, the action is normaly created from a callback of a msg
        timer = create_timer(std::chrono::milliseconds(100), std::bind(&ClientNode::createAction, this));

    }

    void stateChangeGoalResponse(const GoalHandle::SharedPtr& goalHandle)
    {
        handle = goalHandle;
    }
    void stateChangeResult(const GoalHandle::WrappedResult& result)
    {
    }
    void stateChangeFeedback(
        const GoalHandle::SharedPtr goalHandle, const std::shared_ptr<const Action::Feedback> feedback)
    {
    }


    void createAction()
    {
        if(handle)
        {
            return;
        }

        Action::Goal goal;
        goal.force_control = true;

        RCLCPP_INFO_STREAM(get_logger(), "Setting new goal.");
        auto sendGoalOptions = rclcpp_action::Client<Action>::SendGoalOptions();
        sendGoalOptions.goal_response_callback =
            std::bind(&ClientNode::stateChangeGoalResponse, this, std::placeholders::_1);
        sendGoalOptions.feedback_callback = std::bind(&ClientNode::stateChangeFeedback, this, std::placeholders::_1,
                                                    std::placeholders::_2);
        sendGoalOptions.result_callback = std::bind(&ClientNode::stateChangeResult, this, std::placeholders::_1);

        actionClient->async_send_goal(goal, sendGoalOptions);
    }

private:

    GoalHandle::SharedPtr handle;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp_action::Client<Action>::SharedPtr actionClient;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ClientNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
