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
            InputPort<double>("cross_threshold", 0.2, "可进门的角度范围小于这个值时, 则传中")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};
