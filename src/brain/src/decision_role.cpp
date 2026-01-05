#include "brain.h"
#include "decision_role.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

#define REGISTER_DECISION_ROLE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterDecisionRoleNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_DECISION_ROLE_BUILDER(StrikerDecide)
    REGISTER_DECISION_ROLE_BUILDER(GoalieDecide)
    REGISTER_DECISION_ROLE_BUILDER(DefenderDecide)
}

NodeStatus StrikerDecide::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/striker_decide", rerun::TextLog(msg));
    };

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    const double goalpostMargin = 0.3; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, angleGoodForKick: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, angleGoodForKick));

    
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < M_PI / 6
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    } else if (!brain->data->tmImLead) {
        newDecision = "assist";
        color = 0x00FFFFFF;
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x0000FFFF;
    } 
    // 세트피스 상황에서 adjust 없이 바로 킥
    else if (
        (
            (
                (brain->tree->getEntry<string>("gc_game_sub_state_type") == "CORNER_KICK"
                || brain->tree->getEntry<string>("gc_game_sub_state_type") == "GOAL_KICK"
                || brain->tree->getEntry<string>("gc_game_sub_state_type") == "DIRECT_FREE_KICK"
                || brain->tree->getEntry<string>("gc_game_sub_state_type") == "THROW_IN")
                && brain->tree->getEntry<bool>("gc_is_sub_state_kickoff_side")
            )
            // 세트피스가 아닌 경기에서도 골대를 보고있고 장애물이 없다면 정렬 없이 킥
            || (
                angleGoodForKick          // 골대에 각이 있고
                && !avoidKick             // 장애물이 없고
                && reachedKickDir         // 킥 방향 정렬도 되어있다면
            )
        )
        && brain->data->ballDetected
        && ball.range < 0.4 // 거리가 매우 가까울 때 (Kick 노드 즉시 진입 가능 거리)
        && fabs(brain->data->ball.yawToRobot) < 0.2 // 각도가 거의 정면일 때 (~11도 이내)
    ) {
        newDecision = "one_touch";
        color = 0xFF0000FF; // Red color for one touch
    }
    else if (
        (
            (angleGoodForKick && !brain->data->isFreekickKickingOff) 
            || reachedKickDir
        )
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
        && !avoidKick
        && ball.range < 1.5
    ) {
        if (brain->data->kickType == "cross") newDecision = "cross";
        else newDecision = "kick";      
        color = 0x00FF00FF;
        brain->data->isFreekickKickingOff = false; 
    }
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleGoodForKick: %d lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleGoodForKick, brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);

    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1]; 
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

NodeStatus DefenderDecide::tick() {
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision;
    getInput("decision_in", lastDecision);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    // 수비수는 안전하게 걷어내는 것이 목표 (패스)
    // angleGoodForKick은 골대 방향을 보는지 확인하는 함수지만, 
    // 반코트 게임에서는 전방으로 차는 동작(패스/걷어내기)을 위해 그대로 사용합니다.
    const double goalpostMargin = 0.5; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");
    
    // 장애물 회피 로직
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < -2.0 
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;


    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < 0.1
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    bool passFound = brain->tree->getEntry<bool>("pass_found");
    bool isLead = brain->tree->getEntry<bool>("is_lead");

    auto pose = brain->data->robotPoseToField;

    // 레인 설정
    const double laneY = -2.5;
    const double laneTol = 0.10;   // 필요시 파라미터로 빼도 됨
    bool inLane = std::fabs(pose.y - laneY) < laneTol;

    // 1) 공을 모르면 -> find
    if (!(iKnowBallPos || tmBallPosReliable)) {
        newDecision = "find";
        color = 0xFFFFFFFF;
    }
    // 2) non-lead인데 레인 밖이면 -> return (레인 복귀)
    else if (!isLead && !inLane) {
        newDecision = "return";
        color = 0xFFFF00FF;
        
        if (lastDecision != "return") {
        brain->tree->setEntry("return_x", pose.x);

        // 화면 로그로 확인 (콘솔/화면)
        brain->log->logToScreen(
            "debug/ReturnTarget",
            format("Saved return_x=%.2f (pose=(%.2f,%.2f,%.2f)) target=(%.2f,-2.5)",
                   pose.x, pose.x, pose.y, pose.theta, pose.x),
            0xFFFF00FF
        );
        }
    }
    // 3) lead이면 -> (기존대로) chase / pass / adjust
    else if (isLead) {
        // 멀면 chase
        bool wasChasing = (lastDecision == "chase");
        if (ballRange > chaseRangeThreshold * (wasChasing ? 0.9 : 1.0)) {
            newDecision = "chase";
            color = 0x0000FFFF;
        }
        // 킥(패스) 조건
        else if (
            ((angleGoodForKick && !brain->data->isFreekickKickingOff) || reachedKickDir) &&
            brain->data->ballDetected &&
            std::fabs(brain->data->ball.yawToRobot) < 0.1 &&
            !avoidKick &&
            ball.range < 1.5
        ) {
            if (passFound) newDecision = "pass";
            else newDecision = "kick";
            color = 0x00FF00FF;
            brain->data->isFreekickKickingOff = false;
        }
        // 그 외 adjust
        else {
            newDecision = "adjust";
            color = 0xFFFF00FF;
        }
    }
    // 4) non-lead면서 레인 안이면 -> side_chase (항상)
    else {
        newDecision = "side_chase";
        color = 0x00FFFFFF;
    }

// 1. 공을 모르면 -> 찾기
// if (!(iKnowBallPos || tmBallPosReliable))
// {
//     newDecision = "find";
//     color = 0xFFFFFFFF;
// } 
// // 2. 내가 리드가 아니면 -> 패스 지점(x, y)으로 Return
// else if (!isLead) 
// {
//     newDecision = "return";
//     color = 0xFFFF00FF; // 노란색

//     // 저장된 좌표가 없으면 현재 위치를 기본값으로 설정 (첫 기동 시 안전장치)
//     if (!brain->tree->isDefined("return_x")) {
//         brain->tree->setEntry("return_x", pose.x);
//         brain->tree->setEntry("return_y", pose.y);
//     }
    
//     double tx = brain->tree->getEntry<double>("return_x");
//     double ty = brain->tree->getEntry<double>("return_y");
//     brain->log->logToScreen("debug/ReturnTarget", 
//         format("Returning to: x=%.2f, y=%.2f", tx, ty), 0xFFFF00FF);
// }
// // 3. 내가 리드인 경우 -> 적극적 동작 수행
// else 
// {
//     bool wasChasing = (lastDecision == "chase");

//     // 3-1. 공이 멀면 -> chase
//     if (ballRange > chaseRangeThreshold * (wasChasing ? 0.9 : 1.0)) 
//     {
//         newDecision = "chase";
//         color = 0x0000FFFF; // 파란색
//     }
//     // 3-2. 킥/패스 조건 만족 시 -> 실행
//     else if (
//         ((angleGoodForKick && !brain->data->isFreekickKickingOff) || reachedKickDir)
//         && brain->data->ballDetected
//         && fabs(brain->data->ball.yawToRobot) < 0.1
//         && !avoidKick
//         && ball.range < 1.5
//     ) {
//         if (passFound) {
//             newDecision = "pass";
//             color = 0x00FF00FF; // 초록색
//             brain->data->isFreekickKickingOff = false;

//             // [핵심] 패스 직전 위치(x, y)와 방향(theta) 저장
//             if (lastDecision != "pass") {
//                 brain->tree->setEntry("return_x", pose.x);
//                 brain->tree->setEntry("return_y", pose.y);
//                 brain->tree->setEntry("return_yaw", pose.theta);
                
//                 brain->log->logToScreen("debug/ReturnSave", 
//                     format("Saved Pass Point: x=%.2f y=%.2f", pose.x, pose.y), 0xFFFFFFFF);
//             }
//         }
//         else {
//             newDecision = "kick";
//             color = 0xFF0000FF; // 빨간색
//             brain->data->isFreekickKickingOff = false;
//         } 
//     }
//     // 3-3. 그 외 (각도가 안 맞으면) -> 위치 조정 (adjust)
//     else 
//     {
//         newDecision = "adjust";
//         color = 0xFFFF00FF; // 노란색
//     }
// }


    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Defend",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}