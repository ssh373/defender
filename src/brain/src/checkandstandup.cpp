#include "brain.h"
#include "checkandstandup.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

// BehaviorTree Factory에 Test 노드를 생성하는 함수를 등록하는 역할 -> 코드 양 줄일 수 있음
#define REGISTER_CHECKANDSTANDUP_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterCheckAndStandUpNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_CHECKANDSTANDUP_BUILDER(CheckAndStandUp)
}

NodeStatus CheckAndStandUp::tick()
{
    if (brain->tree->getEntry<bool>("gc_is_under_penalty") || brain->data->currentRobotModeIndex == 2) {
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog("reset recovery"));
        return NodeStatus::SUCCESS;
    }
    brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d, recoveryPerformed: %d recoveryState: %d currentRobotModeIndex: %d", brain->data->recoveryPerformedRetryCount, brain->data->recoveryPerformed, brain->data->recoveryState, brain->data->currentRobotModeIndex)));

    if (!brain->data->recoveryPerformed &&
        brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        // brain->data->isRecoveryAvailable && 
        brain->data->currentRobotModeIndex == 1 && 
        brain->data->recoveryPerformedRetryCount < brain->get_parameter("recovery.retry_max_count").get_value<int>()) {
        brain->client->standUp();
        brain->data->recoveryPerformed = true;
        //brain->speak("Trying to stand up");
        brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d", brain->data->recoveryPerformedRetryCount)));
        return NodeStatus::SUCCESS;
    }

    if (brain->data->recoveryPerformed && brain->data->currentRobotModeIndex == 10) {
        brain->data->recoveryPerformedRetryCount +=1;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog(format("Add retry count: %d", brain->data->recoveryPerformedRetryCount)));
    }


    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        brain->data->currentRobotModeIndex == 8) { 
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog("Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
    }

    return NodeStatus::SUCCESS;
}
