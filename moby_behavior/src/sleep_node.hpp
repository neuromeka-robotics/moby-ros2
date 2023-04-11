#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include "behaviortree_cpp_v3/action_node.h"

using namespace std::chrono;

// Example of Asynchronous node that uses StatefulActionNode as base class
using namespace std::chrono;

// Example of Asynchronous node that uses StatefulActionNode as base class
class SleepNode : public BT::StatefulActionNode
{
  public:
    SleepNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
      // amount of milliseconds that we want to sleep
      return{ BT::InputPort<int>("msec") };
    }

    BT::NodeStatus onStart() override
    {
      int msec = 0;
      getInput("msec", msec);

      if( msec <= 0 ) {
        // No need to go into the RUNNING state
        return BT::NodeStatus::SUCCESS;
      }
      else {
        // once the deadline is reached, we will return SUCCESS.
        deadline_ = system_clock::now() + milliseconds(msec);
        return BT::NodeStatus::RUNNING;
      }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus onRunning() override
    {
      if ( system_clock::now() >= deadline_ ) {
        return BT::NodeStatus::SUCCESS;
      }
      else {
        return BT::NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "SleepNode interrupted" << std::endl;
    }

  private:
    system_clock::time_point deadline_;
};
