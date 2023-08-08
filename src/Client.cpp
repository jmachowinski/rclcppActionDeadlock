#include <deadlock/action/some_action.hpp>
#include <random>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/create_client.hpp>

class ClientHandler
{
public:
    using Action = deadlock::action::SomeAction;

    using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

    enum class ExecutionStatus
    {
        goalSend,
        executingPath,
        goalReached,
        error
    };

private:
    mutable std::mutex interfaceMutex;

    ExecutionStatus status;

    size_t id;

    bool cancelOngoing = false;

    rclcpp::Logger logger;
    rclcpp_action::Client<Action>::SharedPtr actionClient;

    std::shared_future<GoalHandle::SharedPtr> pathRequest;
    GoalHandle::SharedPtr execHandle;

public:
    ClientHandler(size_t id, rclcpp::Logger logger, rclcpp_action::Client<Action>::SharedPtr actionClient) :
        id(id),
        logger(logger),
        actionClient(actionClient)
    {
        Action::Goal goal;
        goal.id = id;

        RCLCPP_INFO_STREAM(logger, "Setting new goal for ID " << id << " ptr is " << this);
        auto sendGoalOptions = rclcpp_action::Client<Action>::SendGoalOptions();
        sendGoalOptions.goal_response_callback =
            std::bind(&ClientHandler::stateChangeGoalResponse, this, std::placeholders::_1);
        sendGoalOptions.feedback_callback =
            std::bind(&ClientHandler::stateChangeFeedback, this, std::placeholders::_1, std::placeholders::_2);
        sendGoalOptions.result_callback = std::bind(&ClientHandler::stateChangeResult, this, std::placeholders::_1);

        status = ExecutionStatus::goalSend;

        pathRequest = actionClient->async_send_goal(goal, sendGoalOptions);
    }

    bool isRunning()
    {
        std::lock_guard<std::mutex> lk(interfaceMutex);
        return status == ExecutionStatus::executingPath;
    }

    void dropHandle()
    {
        std::lock_guard<std::mutex> lk(interfaceMutex);
        pathRequest = std::future<GoalHandle::SharedPtr>{};
        execHandle.reset();
    }

    void cancelGoal()
    {
        // we need to create a second copy to the sharedptr in case the
        // second thread jumps in and releases execHandle
        GoalHandle::SharedPtr clientCopy;
        {
            std::lock_guard<std::mutex> lk(interfaceMutex);
            cancelOngoing = true;
            clientCopy = execHandle;
        }

        // this is a potential race condition
        if (clientCopy)
        {
            actionClient->async_cancel_goal(clientCopy);
            RCLCPP_INFO_STREAM(logger, "TrajectoryExecution " << id << ": Cancelation of goal by logic requested"
                                                              << " ptr is " << this);
        }

        {
            std::lock_guard<std::mutex> lk(interfaceMutex);
            cancelOngoing = false;
            execHandle.reset();
            clientCopy.reset();
        }
    }

    void stateChangeGoalResponse(const GoalHandle::SharedPtr& handle)
    {
        std::lock_guard<std::mutex> lk(interfaceMutex);
        if (status != ExecutionStatus::goalSend)
        {
            RCLCPP_INFO_STREAM(
                logger, "TrajectoryExecution::goalResponseCallback : Got goal response but did not send goal ? ID "
                            << id << " ptr is " << this);
            // we ignore any state transitions, if we are not actively planning, this should be error / cancel
            // transitions
            return;
        }

        if (execHandle && handle != execHandle)
        {
            // should never happen
            throw std::runtime_error("Internal error, got feedback callback for non matching handle");
        }

        if (!handle)
        {
            status = ExecutionStatus::error;
            pathRequest = std::future<GoalHandle::SharedPtr>{};
            RCLCPP_INFO_STREAM(logger, "TrajectoryExecution" << id << ": Goal rejected");
        }
        else
        {
            execHandle = handle;
            // lets clear the request, so that we are sure that we only hold one refernce to the handle
            pathRequest = std::future<GoalHandle::SharedPtr>{};
            status = ExecutionStatus::executingPath;
            RCLCPP_INFO_STREAM(logger, "TrajectoryExecution" << id << ": Goal accepted"
                                                             << " ptr is " << this);
        }
    }
    void stateChangeResult(const GoalHandle::WrappedResult& result)
    {
        std::lock_guard<std::mutex> lk(interfaceMutex);
        switch (result.code)
        {
            case rclcpp_action::ResultCode::UNKNOWN:
                RCLCPP_INFO_STREAM(logger, "TrajectoryExecution " << id << ": Internal Error, action result UNKNOWN"
                                                                  << " ptr is " << this);
                break;
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO_STREAM(logger, "TrajectoryExecution" << id << ": Reached target"
                                                                 << " ptr is " << this);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO_STREAM(logger, "TrajectoryExecution " << id << ": action result CANCELED"
                                                                  << " ptr is " << this);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO_STREAM(logger, "TrajectoryExecution " << id << ": action result ABORTED"
                                                                  << " ptr is " << this);
                break;
        }
        execHandle.reset();
    }

    void stateChangeFeedback(const GoalHandle::SharedPtr goalHandle,
                             const std::shared_ptr<const Action::Feedback> /*feedback*/)
    {
        std::lock_guard<std::mutex> lk(interfaceMutex);
        if (cancelOngoing)
        {
            RCLCPP_ERROR_STREAM(logger, "stateChangeFeedback for ID " << id << " hit race condition of cancel"
                                                                      << " ptr is " << this);
            return;
        }

        if (!execHandle)
        {
            RCLCPP_ERROR_STREAM(logger, "stateChangeFeedback for ID " << id << " called but handle was dropped"
                                                                      << " ptr is " << this);
        }

        if (execHandle != goalHandle)
        {
            RCLCPP_ERROR_STREAM(logger, "stateChangeFeedback for ID " << id << " called with invalid handle"
                                                                      << " ptr is " << this);
        }
    }
};

class ClientNode : public rclcpp::Node
{
public:
    using Action = deadlock::action::SomeAction;

    //     rclcpp_action::Server<cm_interfaces::action::NodeStateChange>::SharedPtr cmdNodeStateActionServer;
    ClientNode() : Node("DeadlockClient"), gen(rd())
    {
        actionCbg = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        actionClient = rclcpp_action::create_client<Action>(this, "/deadlock/example", actionCbg);

        RCLCPP_INFO_STREAM(get_logger(), "Waiting for action server");
        actionClient->wait_for_action_server();
        RCLCPP_INFO_STREAM(get_logger(), "Action server active");

        actionClient->async_cancel_all_goals();

        // note, the action is normaly created from a callback of a msg
        createTimer = create_timer(std::chrono::milliseconds(100), std::bind(&ClientNode::createAction, this));
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        cancelTimer = create_timer(std::chrono::milliseconds(1000), std::bind(&ClientNode::cancelAction, this));

        activeClients.resize(30);
        for (size_t i = 0; i < activeClients.size(); i++)
        {
            //             activeClients[i] = std::make_shared<ClientHandler>(i, get_logger(), actionClient);
        }
    }

    void createAction()
    {
        std::uniform_int_distribution<> distrib(0, activeClients.size() - 1);

        size_t id = distrib(gen);

        // save the old handle to keep it alive during application of new path
        std::shared_ptr<ClientHandler> ptr = activeClients[id];

        activeClients[id] = std::make_shared<ClientHandler>(id, get_logger(), actionClient);

        if (ptr)
        {
            ptr->dropHandle();
        }
    }

    void cancelAction()
    {
        std::uniform_int_distribution<> distrib(0, activeClients.size() - 1);

        size_t id = distrib(gen);

        if (activeClients[id] && activeClients[id]->isRunning())
        {
            activeClients[id]->cancelGoal();
        }
    }

private:
    std::random_device rd; // a seed source for the random number engine
    std::mt19937 gen;

    rclcpp::CallbackGroup::SharedPtr actionCbg;

    rclcpp::TimerBase::SharedPtr createTimer;
    rclcpp::TimerBase::SharedPtr cancelTimer;
    rclcpp_action::Client<Action>::SharedPtr actionClient;

    std::vector<std::shared_ptr<ClientHandler>> activeClients;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ClientNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
