#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <deadlock/action/some_action.hpp>

class ServerNode : public rclcpp::Node
{
public:

    using Action = deadlock::action::SomeAction;

    using GoalHandle = rclcpp_action::ServerGoalHandle<deadlock::action::SomeAction>;


    ServerNode() : Node("DeadlockServer")
    {
        actionServer = rclcpp_action::create_server<deadlock::action::SomeAction>(
                            this, "/deadlock/example",
                            std::bind(&ServerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&ServerNode::cancelGoal, this, std::placeholders::_1),
                            std::bind(&ServerNode::acceptGoal, this, std::placeholders::_1));

        timer = create_timer(std::chrono::milliseconds(100), std::bind(&ServerNode::feedbackTimer, this));
    }


    void feedbackTimer()
    {
        if(activeGoal)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Sending Feedback");

            auto fb = std::make_shared<Action::Feedback>();
            fb->still_alive = true;
            activeGoal->publish_feedback(fb);
        }
    }

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                std::shared_ptr<const deadlock::action::SomeAction::Goal> /*goal*/)
    {
        if(activeGoal)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Rejected Goal, had already a handle");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO_STREAM(get_logger(), "Accepted Goal");

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelGoal(const std::shared_ptr<GoalHandle> goalHandle)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Canceled active goal");
        auto res = std::make_shared<Action::Result>();
        res->was_forced_off = false;
        goalHandle->canceled(res);

        activeGoal.reset();

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptGoal(std::shared_ptr<GoalHandle> goalHandle)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Goal activated");
        activeGoal = goalHandle;
    }

private:

    rclcpp::TimerBase::SharedPtr timer;

    rclcpp_action::Server<Action>::SharedPtr actionServer;

    std::shared_ptr<GoalHandle> activeGoal;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
