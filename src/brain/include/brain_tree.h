#pragma once

#include <tuple>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>

#include "types.h"

class Brain;

using namespace std;
using namespace BT;

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();
    void tick();

    // 블랙보드 접근 함수
    template <typename T>
    inline T getEntry(const string &key)
    {
        T value;
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;

    void initEntry(); // 블랙보드 초기화는 여기서 
};


/**
 * @brief 设置机器人的速度
 *
 * @param x,y,theta double, 机器人在 x，y 方向上的速度（m/s）和逆时针转动的角速度（rad/s), 默认值为 0. 全为 0 时，即相当于给出站立不动指令
 *
 */
class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Default x is 0"),
            InputPort<double>("y", 0, "Default y is 0"),
            InputPort<double>("theta", 0, "Default  theta is 0"),
        };
    }

private:
    Brain *brain;
};

// 原地踏步
class StepOnSpot : public SyncActionNode
{
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {};
    }

private:
    Brain *brain;
};

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("pitch", 0, "target head pitch"),
            InputPort<double>("yaw", 0, "target head yaw"),
        };
    }

private:
    Brain *brain;
};
