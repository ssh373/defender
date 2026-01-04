#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterGotoposeNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class GoToPose : public SyncActionNode
{
public:
    GoToPose(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.35, "여기까진 직진으로 성큼성큼 오다가, 여기부터 정면 바라보도록 회전"),
            InputPort<double>("stop_threshold", 0.1, "목표 위치 가까이 도달하면 정지"),
            InputPort<double>("v_limit", 0.5, "최대 속도"),
            InputPort<double>("target_pos_x", -2, "기본위치"),
            InputPort<double>("target_pos_y", 0.0, "기본위치"),
            InputPort<double>("target_pos_theta", 0.0, "골대중앙위치"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};