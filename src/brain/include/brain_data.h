#pragma once

#include <string>
#include <mutex>
#include <tuple>

#include <sensor_msgs/msg/image.hpp>
#include "booster_interface/msg/odometer.hpp"
#include <Eigen/Dense> 

#include "types.h"
#include "RoboCupGameControlData.h"

using namespace std;

struct BrainData {
    BrainData();

    // -- 게임 컨트롤러 관련 데이터 -- 
    int score = 0;
    int oppoScore = 0;

    int penalty[HL_MAX_NUM_PLAYERS]; // 우리 팀 선수의 페널티 상태
    int oppoPenalty[HL_MAX_NUM_PLAYERS]; // 상대 팀 선수의 페널티 상태

    bool isKickingOff = false; // 킥오프(선공) 팀인지 여부
    rclcpp::Time kickoffStartTime; // 킥오프 시작 시간
    bool isFreekickKickingOff = false; // 프리킥 킥오프 팀인지 여부
    rclcpp::Time freekickKickoffStartTime; // 프리킥 킥오프 시작 시간
    int liveCount = 0; // 경기 가능한 선수 수
    int oppoLiveCount = 0; // 상대 경기 가능한 선수 수
    string realGameSubState; // 현재 게임의 하위 상태

    // 게임 상태 관련
    bool isDirectShoot = false; // 패널티킥에서 직접 슛팅으로 해도 되는지

};
