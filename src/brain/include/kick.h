#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

/*
    해당 파일을 킥과 관련된 노드들을 모아놓은 헤더 파일입니다.
*/

void RegisterKickNodes(BT::BehaviorTreeFactory &factory, Brain* brain); // 노드 등록을 위한 함수


// 공과 골대와의 각도를 계산하여 킥 방향을 결정하는 노드
class CalcKickDir : public SyncActionNode {
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts(){
        return {
            InputPort<double>("cross_threshold", 0.2, "득점 가능한 각도 범위가 이 값보다 작으면 크로스(패스)로 전환")
        };
    }

    NodeStatus tick() override;

private:
    NodeStatus tick() override;

private:
    Brain *brain;
};

class CalcPassDir : public SyncActionNode {
public:
    CalcPassDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts(){
        return {
            InputPort<double>("pass_threshold", 3.0, "팀원과의 최대 거리")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("min_msec_kick", 500, "킥 동작을 최소한 이 시간(ms) 동안 실행"),
            InputPort<double>("msecs_stablize", 1000, "동작 안정화를 위해 정지 상태로 유지하는 시간(ms)"),
            InputPort<double>("speed_limit", 0.8, "속도의 최대값"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
    string _state = "kick"; // stablize | kick
    int _msecKick = 1000;    
    double _speed; 
    double _minRange; 
    tuple<double, double, double> _calcSpeed();
};