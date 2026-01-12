#include "brain.h"
#include "brain_tree.h"
#include "kick.h"

// BehaviorTree Factory에 Test 노드를 생성하는 함수를 등록하는 역할 -> 코드 양 줄일 수 있음
#define REGISTER_KICK_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterKickNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_KICK_BUILDER(CalcKickDir)
    REGISTER_KICK_BUILDER(CalcKickDirWithGoalkeeper)
    REGISTER_KICK_BUILDER(CalcPassDir)
    REGISTER_KICK_BUILDER(Kick)
}

// 해당 노드는 반코트용으로 짰음 -> 풀코트 용으로 나중에 수정 필요함 
// 제안 : 반코트, 풀코트 구분하도록 HalfCalcKickDir, FullCalcKickDir로 구분할 수도 있으니 알아서 잘 활용하길 바람
// 추가해야 될 것 -> 수비 상황 판단
NodeStatus CalcKickDir::tick(){
    // 좌·우 골포스트 각도 차가 이 값보다 작으면 → 골대가 “좁게 보인다” = 정면 슛 각이 안 나온다
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);

    string lastKickType = brain->data->kickType;
    if (lastKickType == "cross") crossThreshold += 0.1; // 이전 type이 cross였다면 좀 더 threshold를 높임으로써 cross를 유지할 수 있도록 함

    auto gpAngles = brain->getGoalPostAngles(0.0); // 공과 골대 각도 계산 -> 마진이 0으로 돼있음
    auto thetal = gpAngles[0]; auto thetar = gpAngles[1]; // 공 기준 좌우 골대 각도
    auto bPos = brain->data->ball.posToField; // 공 위치
    auto fd = brain->config->fieldDimensions; // 필드 정보
    auto color = 0xFFFFFFFF;

    // 풀코트용 슛 방향 계산
    // if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
    //     brain->data->kickType = "cross";
    //     color = 0xFF00FFFF;
    //     brain->data->kickDir = atan2(
    //         - bPos.y,
    //         fd.length/2 - fd.penaltyDist/2 - bPos.x
    //     );
    // }
    // else if (brain->isDefensing()) {
    //     brain->data->kickType = "block";
    //     color = 0xFFFF00FF;
    //     brain->data->kickDir = atan2(
    //         bPos.y,
    //         bPos.x + fd.length/2
    //     );
    // } else { 
    //     brain->data->kickType = "shoot";
    //     color = 0x00FF00FF;
    //     brain->data->kickDir = atan2(
    //         - bPos.y,
    //         fd.length/2 - bPos.x
    //     );
    //     if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; 
    // }

    // 반코트용 슛 방향 계산 ( 상대 골대가 음수라고 가정)
    // 추가해야 될 것 -> 수비 상황 판단
    // 그리고 항상 왼쪽이 양수일까?도 고민해보기
    // if (thetal - thetar < crossThreshold) {
    //     brain->data->kickType = "cross";
    //     color = 0xFF00FFFF;
    //     // atan2( 목표Y - 공Y , 목표X - 공X )
    //     // 반코트는 우리팀 진영을 상대팀으로 인식해야하므로 - 값이 들어감
    //     brain->data->kickDir = atan2( 
    //         0 - bPos.y, 
    //         - (fd.length/2 - fd.penaltyDist/2) - bPos.x); // 앞에 - 붙였음
    // }
    // else { 
        brain->data->kickType = "shoot";
        color = 0x00FF00FF;
        brain->data->kickDir = atan2(
            0 - bPos.y,
            - fd.length/2 - bPos.x // 여기도 - 붙였음
        );
        // 이것도 - 로 바꿈 + 0에서 M_PI로 바꿈
        if (brain->data->ball.posToField.x < - (brain->config->fieldDimensions.length / 2)) brain->data->kickDir = M_PI; 
    // }

    brain->log->setTimeNow();
    brain->log->log(
        "field/kick_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({color})
            .with_radii(0.01)
            .with_draw_order(31)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus CalcKickDirWithGoalkeeper::tick(){
    double crossThreshold;
    double goalkeeperMargin;
    getInput("cross_threshold", crossThreshold);
    getInput("goalkeeper_margin", goalkeeperMargin);

    auto bPos = brain->data->ball.posToField;
    auto fd = brain->config->fieldDimensions;
    auto color = 0xFFFFFFFF;

    // 1. 골키퍼(장애물) 찾기
    vector<GameObject> obstacles = brain->data->getObstacles();
    vector<GameObject> goalkeepers;
    
    // 골대 주변에 있는 장애물만 골키퍼 후보로 간주
    double goalX = (brain->config->fieldDimensions.length / 2);
    goalX = - (brain->config->fieldDimensions.length / 2); 

    for(const auto& obs : obstacles){
        // 골대 근처 (페널티 박스 + margin)에 있는 로봇을 골키퍼로 인식
        if(obs.posToField.x < goalX + fd.goalAreaLength + goalkeeperMargin){goalkeepers.push_back(obs);}
    }

    // 공 -> 왼쪽 포스트 각도(thetal), 공 -> 오른쪽 포스트 각도(thetar), 골키퍼가 이 사이에 있으면 각도를 좁혀야 함.
    double bestKickDir = 0.0;
    bool isBlocked = false;
    double maxOpenAngle = 0.0;
    string kickType = "shoot";

    // 골키퍼 없으면 기존 로직대로 그냥 킥
    if (goalkeepers.empty()) { bestKickDir = atan2(0 - bPos.y, goalX - bPos.x);} 
    // 골키퍼가 있다면        
    else {        
        vector<pair<double, double>> blockedIntervals;
        for(const auto& gk : goalkeepers){
            double dist = norm(gk.posToField.x - bPos.x, gk.posToField.y - bPos.y);
            double angleToGK = atan2(gk.posToField.y - bPos.y, gk.posToField.x - bPos.x);
            double angularWidth = atan2(goalkeeperMargin, dist);
            blockedIntervals.push_back({angleToGK - angularWidth, angleToGK + angularWidth});
        }
        
        double angleLeftPost = atan2(fd.goalWidth/2 - bPos.y, goalX - bPos.x);
        double angleRightPost = atan2(-fd.goalWidth/2 - bPos.y, goalX - bPos.x);
    
        auto gk = goalkeepers[0];
        double gkAngle = atan2(gk.posToField.y - bPos.y, gk.posToField.x - bPos.x);
        double angleToGoalCenter = atan2(0 - bPos.y, goalX - bPos.x);
        
        double diff = gkAngle - angleToGoalCenter;
        while(diff > M_PI) diff -= 2*M_PI;
        while(diff < -M_PI) diff += 2*M_PI;
        
        // 골키퍼가 중심보다 왼쪽에 있음(diff > 0) -> 오른쪽 포스트 쪽으로 슛
        if(diff > 0) {
            bestKickDir = angleRightPost + (angleToGoalCenter - angleRightPost) * 0.5; // 오른쪽 절반의 중간
        } else {
            bestKickDir = angleLeftPost + (angleToGoalCenter - angleLeftPost) * 0.5; // 왼쪽 절반의 중간
        }
        
        // 만약 골키퍼가 너무 중앙이라 양쪽 다 좁다 or thetal - thetar 자체가 작다면 -> 여기서 바로 cross로 가도 좋을듯
    }

    // 최종 결정
    double angleLeftPost = atan2(fd.goalWidth/2 - bPos.y, goalX - bPos.x);
    double angleRightPost = atan2(-fd.goalWidth/2 - bPos.y, goalX - bPos.x);
    
    double angleDiff = angleLeftPost - angleRightPost;
    while(angleDiff > M_PI) angleDiff -= 2*M_PI;
    while(angleDiff < -M_PI) angleDiff += 2*M_PI;
    
    double goalVisibleAngle = fabs(angleDiff);

    brain->data->kickDir = bestKickDir;

    // 만약 골문이 너무 좁거나(crossThreshold), 확실한 슛 각이 안나오면 cross psss가 된다면 pass로 수정
     if (goalVisibleAngle < crossThreshold) {
        kickType = "cross";
        color = 0xFF00FFFF;
     }
    
    brain->data->kickType = kickType;

    // 시각화
    brain->log->setTimeNow();
    brain->log->log(
        "field/kick_dir_gk",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({color}) 
            .with_radii(0.015) 
            .with_draw_order(32)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus CalcPassDir::tick(){
    double maxpassThreshold;
    double minpassThreshold;
    double scoreThreshold;
    getInput("max_pass_threshold", maxpassThreshold);    
    getInput("min_pass_threshold", minpassThreshold);
    getInput("score_threshold", scoreThreshold);

    auto bPos = brain->data->ball.posToField; // 공 위치
    int bestTeammateIdx = -1;
    int myPlayerId = brain->config->playerId;
    double maxTmSelectScore = -99999.0;
    auto fd = brain->config->fieldDimensions; // 필드 정보
    auto color = 0xFFFFFFFF;

    // 가장 가까운(혹은 적절한) 팀원 찾기
    for(int i=0; i<HL_MAX_NUM_PLAYERS; i++){
        // 나 자신 제외, 살아있는 팀원 확인
        if(i + 1 == brain->config->playerId) continue;
        if(!brain->data->tmStatus[i].isAlive) continue; 
        
        // 팀원 위치
        auto tmPos = brain->data->tmStatus[i].robotPoseToField;
        
        // 거리 계산
        double dist = norm(bPos.x - tmPos.x, bPos.y - tmPos.y);

        // 유효 거리 내에 있고, 가장 score가 높은 팀원 선택
        // 나랑 가까울 수록 높은 score, 전방에 있는(x좌표가 낮음) 선수일수록 높은 score
        if( dist > minpassThreshold && dist < maxpassThreshold){
            double score = -dist - tmPos.x;
            if(score > maxTmSelectScore){
                maxTmSelectScore = score;
                bestTeammateIdx = i;
            }
        }
    }

    // 패스가 가능한지 ? 아니라면 다음으로 (bestTeammateIdx가 한명이라도 있고, 목표 범위 내에 로봇이 왔는지 확인)
    bool passFound = (bestTeammateIdx != -1);
    setOutput("pass_found", passFound);

    if(!passFound){
        return NodeStatus::SUCCESS;
    }
    
    brain->data->kickType = "pass"; // 킥 타입 설정
    auto tmPos = brain->data->tmStatus[bestTeammateIdx].robotPoseToField; // 패스를 주기로 결정한 그 팀원 위치
    auto Opponents = brain->data->getRobots(); // Opponents. 사용하려면 label == Opponent 확인해야함
    
    double fieldlimitx = fd.length / 2.0;
    double fieldlimity = fd.width / 2.0;
    double maxScore = -99999.0;
    double tx, ty; // 타겟 좌표
    
    for (double x = tmPos.x - 2; x <= tmPos.x + 2; x += 0.2){
        for (double y = tmPos.y - 2; y <= tmPos.y + 2; y += 0.2){
            if (fabs(x) > fieldlimitx || fabs(y) > fieldlimity) continue; // 필드 범위를 벗어난다면 continue
            
            double score = 10.0
                        - (fabs(x - tmPos.x) * 0.5) // 팀원과의 거리
                        - (fabs(y - tmPos.y) * 0.5) // 팀원과의 거리
                        - x * 0.3; // x좌표가 낮을수록 높은 score
            
            Line passPath = {bPos.x, bPos.y, x, y};

            for (const auto& opponent : Opponents){
                if (opponent.label != "Opponent") continue; // 상대팀이 아니면 스킵
                
                // TODO : striker 코드에서 메모리 부분 받아오고 나면 아래 주석해제 -> 적용합시다 -> 함
                rclcpp::Time now = brain->get_clock()->now();
                double elapsed = (now - opponent.timePoint).seconds(); // 수비수를 마지막으로 본지 몇초 지났나
                double confidenceFactor = std::max(0.0, (5.0 - elapsed) / 5.0); // 시간이 지날수록 신뢰도 떨어지게
                if (confidenceFactor <= 0.0) continue;

                double distToPassPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, passPath);
                if (distToPassPath < 1.0){ // 경로선분 1미터 이내에 opponent가 있다면
                    score -= (1.0 - distToPassPath) * 20.0; * confidenceFactor // 추가
                }
                
            }

            // score가 가장 높은 타겟 선택
            if (score > maxScore){
                maxScore = score;
                tx = x;
                ty = y;
            }
        }
    }
    if (maxScore < scoreThreshold){ // 패스결심이 안선다면 패스하지 않음
        setOutput("pass_found", false);
        brain->data->tmStatus[myPlayerId].passSignal = false;
        brain->data->tmStatus[myPlayerId].passTargetX = 0.;
        brain->data->tmStatus[myPlayerId].passTargetY = 0.;
        // rerun log
        brain->log->log("field/pass_dir", 
            rerun::TextLog(format("Decided not to pass. Score: %.2f", maxScore))
        );
        return NodeStatus::RUNNING;
    }
    brain->data->tmStatus[myPlayerId].passSignal = true;
    brain->data->tmStatus[myPlayerId].passTargetX = tx;
    brain->data->tmStatus[myPlayerId].passTargetY = ty;

    // 공 -> 타겟 방향으로 킥
    brain->data->kickDir = atan2(ty - bPos.y, tx - bPos.x);
    
    // 거리에 비례한 speed limit 설정
    double d = norm(bPos.x - tx, bPos.y - ty);

    // 파라미터
    double k = 1.7;        // 감도

    // 경험적 식...
    double passSpeed = k * std::sqrt(d / 10.0);

    passSpeed = std::min(passSpeed, 2.0);

    // BT로 전달 (Kick에서 speed_limit으로 사용)
    setOutput("pass_speed_limit", passSpeed);

    // 디버그 로그
    brain->log->logToScreen(
        "debug/pass_speed",
        format("PassPower d=%.2f -> speed_limit=%.2f", d, passSpeed),
        0x00FFFFFF
    );


    brain->log->logToScreen("debug/Pass", format("Passing to TM %d to (%.2f, %.2f)", bestTeammateIdx+1, tx, ty), 0x00FF00FF);

    // 시각화
    brain->log->setTimeNow();
    brain->log->log(
        "field/pass_dir",
        rerun::Arrows2D::from_vectors({{(tx - bPos.x), -(ty - bPos.y)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({color}) // Cyan color for pass
            .with_radii(0.01)
    );

    return NodeStatus::SUCCESS;
}

// 승재욱 - 직접 만든 Kick
// 반코트용으로 부호 및 부등호 바뀐 부분 존재 -> 풀코트로 수정 필요
tuple<double, double, double> Kick::_calcSpeed() {
    double vx, vy, msecKick;


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;   
    double yawOffset = brain->config->yawOffset; 


    double adjustedYaw = brain->data->ball.yawToRobot + yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range; 
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01){ 
        vx = vxLimit;
        vy = 0.0;
    }
    else{ 
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit){
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    double speed = norm(vx, vy);
    msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
    
    return make_tuple(vx, vy, msecKick);
}

NodeStatus Kick::onStart(){

    // _while로 제어되므로 별도의 state check 불필요
    // if(brain->tree->getEntry<string>("striker_state") != "kick") return NodeStatus::SUCCESS;

    _minRange = brain->data->ball.range;
    _speed = 0.5;
    _startTime = brain->get_clock()->now();

    // 장애물 회피 로직
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    string role = brain->tree->getEntry<string>("player_role");
    
    if (
        avoidPushing
        && (role != "goal_keeper")
        && brain->data->robotPoseToField.x > -(brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength) // 음수 및 부등호 방향 변경 -> 반코트용
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        
        return NodeStatus::SUCCESS;
    }

    // [원본 그대로] 운동 지령 (게걸음 시작)
    double angle = brain->data->ball.yawToRobot;
    brain->client->crabWalk(angle, _speed);
    
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning(){
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Kick", rerun::TextLog(msg));
    };
    string kickType = getInput<string>("kick_type").value();
    brain->log->logToScreen("debug/Action", "Action: " + kickType, 0x00FF00FF);

    // if(brain->tree->getEntry<string>("striker_state") != "kick") return NodeStatus::SUCCESS;

    bool enableAbort;
    brain->get_parameter("strategy.abort_kick_when_ball_moved", enableAbort);
    auto ballRange = brain->data->ball.range;
    const double MOVE_RANGE_THRESHOLD = 0.3;
    const double BALL_LOST_THRESHOLD = 1000;  
    
    if (
        enableAbort 
        && (
            (brain->data->ballDetected && ballRange - _minRange > MOVE_RANGE_THRESHOLD) 
            || brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD 
        )
    ) {
        log("ball moved, abort kick");
        // brain->tree->setEntry("striker_state", "chase"); // 상태 변경은 Decide 노드에 맡김
        
        return NodeStatus::SUCCESS;
    }

    if (ballRange < _minRange) _minRange = ballRange;    

    // [원본 그대로] 킥 도중 장애물 감지 시 회피
    bool avoidPushing;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    
    if (
        avoidPushing
        && brain->data->robotPoseToField.x > -(brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength) // 음수 및 부등호 방향 변경 -> 반코트용
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        
        return NodeStatus::SUCCESS;
    }

    // [원본 그대로] 시간 체크 및 종료 처리
    double msecs = getInput<double>("min_msec_kick").value();
    double speedLimit = getInput<double>("speed_limit").value(); // speed 변수명 겹침 주의해서 speedLimit으로 변경 권장하나 원본 유지
    
    // 원본 로직: 시간에 거리/속도 항을 더함
    msecs = msecs + brain->data->ball.range / speedLimit * 1000;
    
    if (brain->msecsSince(_startTime) > msecs) { 
        brain->client->setVelocity(0, 0, 0);

        // 승재욱 추가
        // brain->tree->setEntry("striker_state", "chase");
        
        return NodeStatus::SUCCESS;
    }

    // [원본 그대로] 가속 로직 (점점 빨라짐)
    if (brain->data->ballDetected) { 
        double angle = brain->data->ball.yawToRobot;
        // _speed는 멤버 변수
        _speed += 0.1; 
        
        // 입력받은 제한 속도와 비교
        double currentCmdSpeed = min(speedLimit, _speed);
        
        brain->log->setTimeNow();
        brain->log->log(
            "debug/kick/speed",
            rerun::TextLog(format(
                "angle=%s | currentCmdSpeed=%.3f",
                angle, currentCmdSpeed
            ))
        );

        brain->client->crabWalk(angle, currentCmdSpeed);
    }
    
    // 승재욱 추가: _calcSpeed 활용하도록 변경
    // if(brain->data->ballDetected){
    //     auto [vx, vy, _] = _calcSpeed();
    //     double vtheta = brain->data->ball.yawToRobot * 1.5; // P-gain 1.5
    //     // 실제 평면 속도
    //     double v = std::sqrt(vx * vx + vy * vy);

    //     // === 로그 ===
    //     brain->log->setTimeNow();
    //     brain->log->log(
    //         "debug/kick/speed",
    //         rerun::TextLog(format(
    //             "kickType=%s | vx=%.3f vy=%.3f | v=%.3f | vtheta=%.3f | range=%.3f",
    //             kickType.c_str(),
    //             vx, vy,
    //             v,
    //             vtheta,
    //             ballRange
    //         ))
    //     );

    //     brain->client->setVelocity(vx, vy, vtheta);
    // }



    return NodeStatus::RUNNING;
}

void Kick::onHalted(){
    // [원본]
    // brain->tree->setEntry("striker_state", "chase");
    _startTime -= rclcpp::Duration(100, 0);
}
