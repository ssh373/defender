#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterDecisionRoleNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀어지면 공을 추격하는 동작을 수행"),
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            InputPort<string>("position", "offense", "offense | defense, 공을 어느 방향으로 찰지 결정"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir = 0.0; 
    rclcpp::Time timeLastTick; 
};

class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀면 공 추격(Chase) 동작을 실행"),
            InputPort<double>("adjust_angle_tolerance", 0.1, "이 각도보다 작으면 adjust 동작을 실행"), //
            InputPort<double>("adjust_y_tolerance", 0.1, "y 방향偏移이 이 값보다 작으면 adjust 동작을 실행"),  //
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            InputPort<double>("auto_visual_kick_enable_dist_min", 2.0, "자동 시각 킥볼이 가능할 때 공의 최소 거리"),
            InputPort<double>("auto_visual_kick_enable_dist_max", 3.0, "자동 시각 킥볼이 가능할 때 공의 최대 거리"),
            InputPort<double>("auto_visual_kick_enable_angle", 0.785, "자동 시각 킥볼이 가능할 때 공의 각도 범위"),
            InputPort<double>("auto_visual_kick_obstacle_dist_threshold", 3.0, "자동 시각 킥볼이 가능할 때 공의 최대 거리"),
            InputPort<double>("auto_visual_kick_obstacle_angle_threshold", 1.744, "자동 시각 킥볼이 가능할 때 공의 각도 범위"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class DefenderDecide : public SyncActionNode
{
public: 
    DefenderDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀어지면 공을 추격하는 동작을 수행"),
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir = 0.0; 
    rclcpp::Time timeLastTick; 
};