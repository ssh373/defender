#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterMoveHeadNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain){}

    NodeStatus tick() override;

    static BT::PortsList providedPorts(){
        return {
            InputPort<double>("pitch", 0, "target head pitch"),
            InputPort<double>("yaw", 0, "target head yaw"),
        };
    }

private:
    Brain *brain;
};

class CamFindBall : public SyncActionNode{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;

private:
    double _cmdSequence[6][2];    
    rclcpp::Time _timeLastCmd;   
    int _cmdIndex;                
    long _cmdIntervalMSec;        
    long _cmdRestartIntervalMSec; 

    Brain *brain;

};