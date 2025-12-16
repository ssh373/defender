#include <cmath>
#include <cstdlib>
#include "brain_tree.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <ios>

/**
 * 这里使用宏定义来缩减 RegisterBuilder 的代码量
 * REGISTER_BUILDER(Test) 展开后的效果是
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *     [this](const string& name, const NodeConfig& config) { return make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(StepOnSpot)
    REGISTER_BUILDER(MoveHead)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // 여기서 블랙보드가 초기화됨
    initEntry();
}

void BrainTree::initEntry()
{
    // 여기서 블랙보드를 초기화하면 됨 
    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", ""); // 현재 ready,set,play,end 중 하나일 것 
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false); // 우리 팀 킥오프인지
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);
    setEntry<int>("control_state", 1); // control_state == 1 이면 단순 걷기로 
}

void BrainTree::tick()
{
    tree.tickOnce();
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->setVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus StepOnSpot::tick()
{
    std::srand(std::time(0));
    double vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01;

    auto res = brain->client->setVelocity(vx, 0, 0);
    return NodeStatus::SUCCESS;
}

NodeStatus MoveHead::tick()
{
    double pitch, yaw;
    getInput("pitch", pitch);
    getInput("yaw", yaw);
    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

