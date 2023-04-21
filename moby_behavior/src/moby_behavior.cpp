#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include "nav2_client.hpp"
#include "sleep_node.hpp"

using namespace BT;

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
  
    auto nh = rclcpp::Node::make_shared("moby_behavior_tree");
    nh->declare_parameter("moby_bt", "no_default");
    std::string moby_bt;
    nh->get_parameter("moby_bt", moby_bt);

    RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", moby_bt.c_str());
  
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;
  
    factory.registerNodeType<Nav2Client>("Nav2Client");
    factory.registerNodeType<SleepNode>("SleepNode");

    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromFile(moby_bt);
  
    // Create a logger
    StdCoutLogger logger_cout(tree);
  
    NodeStatus status = NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickRoot();
        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  
    return 0;
}
