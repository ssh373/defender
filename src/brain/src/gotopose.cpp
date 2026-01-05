#include "brain.h"
#include "gotopose.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

// BehaviorTree Factory에 Test 노드를 생성하는 함수를 등록하는 역할 -> 코드 양 줄일 수 있음
#define REGISTER_GOTOPOSE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterGotoposeNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_GOTOPOSE_BUILDER(GoToPose) // Gotopose
}

NodeStatus GoToPose::tick(){
    double turn_Threshold;
    double stop_Threshold;
    double vLimit;
    getInput("turn_threshold", turn_Threshold); 
    getInput("stop_threshold", stop_Threshold); 
    getInput("v_limit", vLimit);

    // 목표지점
    double targetx, targety, targettheta;
    getInput("target_pos_x", targetx); 
    getInput("target_pos_y", targety); 
    getInput("target_pos_theta", targettheta); 

    // 본인 위치
    auto rPos = brain->data->robotPoseToField;
    double gx = rPos.x, gy = rPos.y, gtheta = rPos.theta;

    double errorx = targetx - gx;
    double errory = targety - gy;
    double targetdir = atan2(errory, errorx); // 내 위치에서 골대중앙을 이은 벡터의 각도
    double errortheta = toPInPI(targettheta - gtheta); // 이걸 P제어한다면 골대중앙을 쳐다볼것.

    double dist = norm(errorx, errory); // 골대중앙까지의 거리
    double controlx, controly, controltheta;
    double Kp = 4.0;
    double linearFactor = 1.0 / (1.0 + exp(-6.0 * (dist - 0.5)));
    
    if(dist > stop_Threshold){// 직진
      controltheta = errortheta * Kp;
      controltheta = cap(controltheta, 1.2, -1.2); // 회전 속도 제한

      controlx = errorx*cos(gtheta) + errory*sin(gtheta);
      controly = -errorx*sin(gtheta) + errory*cos(gtheta);
      controlx *= linearFactor;
      controly *= linearFactor;
      controlx = cap(controlx, vLimit, -vLimit*0.5);    
      controly = cap(controly, vLimit*0.5, -vLimit*0.5);
    }
    else if (fabs(errortheta) > 0.2) {
        controlx = 0;
        controly = 0;
        controltheta = errortheta * Kp;
    }
    else{ // 정지
        controlx = 0;
        controly = 0;
        controltheta = 0;
    }

		brain->client->setVelocity(controlx, controly, controltheta);
    return NodeStatus::SUCCESS;
}