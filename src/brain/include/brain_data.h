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
    // 생성자
    BrainData();

    /* ----------------------------------------------------------------------------- 게임 컨트롤러 관련 파라미터 -------------------------------------------------------------------- */ 
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

    rclcpp::Time timeLastGamecontrolMsg; // 마지막으로 읽은 게임 컨트롤 메시지 시간

    /* -------------------------------------------------------------------------------- 게임 상태 관련 파라미터 -------------------------------------------------------------------- */
    bool isDirectShoot = false; // 패널티킥에서 직접 슛팅으로 해도 되는지
    double kickDir = 0.; // 킥 방향
    string kickType = "shoot"; // 킥 타입

    rclcpp::Time timeLastLogSave; 
    rclcpp::Time lastTick; 

    /* ------------------------------------------------------------------------------ detection 관련 파라미터 -------------------------------------------------------------------- */
    // 마지막 객체 감지 시간 기록
    rclcpp::Time timeLastDet; 
    rclcpp::Time timeLastLineDet; // 마지막 line seg 감지 시간 기록
    bool camConnected = false; // 카메라 연결 상태

    // ball
    GameObject ball;
    GameObject tmBall; 
    bool ballDetected = false; // 공 감지 플래그
    double robotBallAngleToField;

    VisionBox visionBox; 
    
    /* ---------------------------------------------------------------------------- 로봇 위치 관련 파라미터 -------------------------------------------------------------------- */
    Pose2D robotPoseToField;
    Pose2D odomToField;      
    Pose2D robotPoseToOdom;  

    inline vector<GameObject> getRobots() const {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        return _robots;
    }
    inline void setRobots(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        _robots = newVec;
    }

    Pose2D robot2field(const Pose2D &poseToRobot);
    Pose2D field2robot(const Pose2D &poseToField);

    /* ---------------------------------------------------------------------------- 로봇 머리 관련 파라미터 -------------------------------------------------------------------- */
    double headPitch;
    double headYaw;
    // 카메라 → 로봇 변환 행렬
    Eigen::Matrix4d camToRobot = Eigen::Matrix4d::Identity(); 

    /* ---------------------------------------------------------------------------- 로봇 recovery 관련 파라미터 -------------------------------------------------------------------- */
    RobotRecoveryState recoveryState = RobotRecoveryState::IS_READY;
    bool isRecoveryAvailable = false; 
    int currentRobotModeIndex = -1;
    int recoveryPerformedRetryCount = 0; 
    bool recoveryPerformed = false;
    
    /* ---------------------------------------------------------------------------- locator 관련 파라미터 -------------------------------------------------------------------- */
    rclcpp::Time lastSuccessfulLocalizeTime;
    vector<GameObject> getMarkingsByType(set<string> types={});
    vector<FieldMarker> getMarkersForLocator();

    /* ---------------------------------------------------------------------------- 팀원 정보 관련 파라미터 -------------------------------------------------------------------- */
    TMStatus tmStatus[HL_MAX_NUM_PLAYERS]; 
    int tmCmdId = 0; 
    rclcpp::Time tmLastCmdChangeTime; 
    int tmMyCmd = 0; 
    int tmMyCmdId = 0; 
    int tmReceivedCmd = 0; 
    bool tmImLead = true; 
    bool tmImAlive = true; 
    double tmMyCost = 0.;
    Point2D tmtarget;
    
    // Team Discovery 추적 (BrainCommunication 용)
    int discoveryMsgId = 0;
    rclcpp::Time discoveryMsgTime;
    
    // Team Communication 송수신 추적
    string tmIP = "";
    int sendId = 0;
    rclcpp::Time sendTime;
    
    // predictball
    Pose2D Pred_ball;        // 미래 위치 (horizon초 뒤)
    Pose2D Final_ball_pos;
    
    /* ---------------------------------------------------------------------------- 필드 라인 관련 파라미터 -------------------------------------------------------------------- */
    
    // 필드 라인 가져오는 변수
    inline vector<FieldLine> getFieldLines() const {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        return _fieldLines;
    }
    // 필드 라인 셋팅하는 변수
    inline void setFieldLines(const vector<FieldLine>& newVec) {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        _fieldLines = newVec;
    }
    inline vector<GameObject> getGoalposts() const {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        return _goalposts;
    }
    inline void setGoalposts(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        _goalposts = newVec;
    }
    inline vector<GameObject> getMarkings() const {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        return _markings;
    }
    inline void setMarkings(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        _markings = newVec;
    }

    /* ---------------------------------------------------------------------------- obstacle 관련 파라미터 -------------------------------------------------------------------- */
    inline vector<GameObject> getObstacles() const {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        return _obstacles;
    }
    inline void setObstacles(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        _obstacles = newVec;
    }

    /* ---------------------------------------------------------------------------- person 관련 파라미터 -------------------------------------------------------------------- */
    inline vector<GameObject> getPersons() const {
        std::lock_guard<std::mutex> lock(_personsMutex);
        return _persons;
    }
    inline void setPersons(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_personsMutex);
        _persons = newVec;
    }


private:
    vector<GameObject> _robots = {}; 
    mutable std::mutex _robotsMutex;

    vector<FieldLine> _fieldLines = {};
    mutable std::mutex _fieldLinesMutex;

    vector<GameObject> _goalposts = {}; 
    mutable std::mutex _goalpostsMutex;

    vector<GameObject> _markings = {};                             
    mutable std::mutex _markingsMutex;

    vector<GameObject> _obstacles = {};
    mutable std::mutex _obstaclesMutex;

    vector<GameObject> _persons = {};
    mutable std::mutex _personsMutex;

};
