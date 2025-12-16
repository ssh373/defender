#pragma once

#include <string>
#include <ostream>

#include "types.h"
#include "utils/math.h"
#include "RoboCupGameControlData.h"


using namespace std;


class BrainConfig
{
public:
    // config 순서대로 정렬함
    int teamId;                
    int playerId;               
    string fieldType;           
    string playerRole;          

    bool treatPersonAsRobot = false;  
    int numOfPlayers = 4;


    // 경기장 크기 정보
    FieldDimensions fieldDimensions; // 필드 kid_size인지 

    // BT 관련 변수 
    string treeFilePath; // behavior tree 파일 경로

    // 로봇 제어를 위한 파라미터
    double headYawLimitLeft = 1.1; // 머리 제어
    double headYawLimitRight = -1.1; 
    double headPitchLimitUp = 0.45; 
    double vxLimit = 0.8; // 속도 제한
    double vyLimit = 0.4;
    double vthetaLimit = 1.0;

    double vxFactor; //crabWalk에 사용
    double yawOffset;    

    // 멤버 함수
    void handle();
};