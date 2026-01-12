#include "brain_data.h"
#include "utils/math.h"

BrainData::BrainData(){ std::fill(std::begin(penalty), std::end(penalty), SUBSTITUTE); }

vector<GameObject> BrainData::getMarkingsByType(set<string> types) {
    if (types.size() == 0) return getMarkings();

    // else
    vector<GameObject> res = {}; 
    auto markings = getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        if (types.count(markings[i].label) > 0) res.push_back(markings[i]);
    }
    
    return res;
}

vector<FieldMarker> BrainData::getMarkersForLocator(){
    vector<FieldMarker> res;
    auto markings = getMarkings();
    for (size_t i = 0; i < markings.size(); i++){
        auto label = markings[i].label;
        auto x = markings[i].posToRobot.x;
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;

        char markerType;
        if (label == "LCross")
            markerType = 'L';
        else if (label == "TCross")
            markerType = 'T';
        else if (label == "XCross")
            markerType = 'X';
        else if (label == "PenaltyPoint")
            markerType = 'P';

        res.push_back(FieldMarker{markerType, x, y, confidence});
    }
    return res;
}

Pose2D BrainData::robot2field(const Pose2D &poseToRobot){
    Pose2D poseToField;
    transCoord(
        poseToRobot.x, poseToRobot.y, poseToRobot.theta,
        robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
        poseToField.x, poseToField.y, poseToField.theta);
    poseToField.theta = toPInPI(poseToField.theta);
    return poseToField;
}

Pose2D BrainData::field2robot(const Pose2D &poseToField){
    Pose2D poseToRobot;
    double xfr, yfr, thetafr; // fr = field to robot
    yfr = sin(robotPoseToField.theta) * robotPoseToField.x - cos(robotPoseToField.theta) * robotPoseToField.y;
    xfr = -cos(robotPoseToField.theta) * robotPoseToField.x - sin(robotPoseToField.theta) * robotPoseToField.y;
    thetafr = -robotPoseToField.theta;
    transCoord(
        poseToField.x, poseToField.y, poseToField.theta,
        xfr, yfr, thetafr,
        poseToRobot.x, poseToRobot.y, poseToRobot.theta);
    return poseToRobot;
}

// 1.10 - 로봇 메모리 업데이트
void BrainData::updateRobots(const vector<GameObject>& newObservations, double retentionTime) {
    std::lock_guard<std::mutex> lock(_robotsMutex);
    
    // 현재 시간 (가장 최신 관측값의 시간 사용, 없으면 시스템 시간)
    rclcpp::Time now;
    if (!newObservations.empty()) {
        now = newObservations[0].timePoint;
    } else {
        now = rclcpp::Clock(RCL_ROS_TIME).now();
    }

    // 1. 새 관측값으로 기존 로봇 업데이트 혹은 추가
    for (const auto& newObj : newObservations) {
        // Opponent만 처리 (필수는 아니지만 명시적으로)
        // if (newObj.label != "Opponent") continue; 

        bool matched = false;
        double minDistance = 1.0; // 매칭 임계값 (1m)

        for (int i = 0; i < _robots.size(); i++) {
            double dist = norm(newObj.posToField.x - _robots[i].posToField.x, 
                               newObj.posToField.y - _robots[i].posToField.y);
            if (dist < minDistance) {
                _robots[i] = newObj;
                matched = true;
            }
        }
        if (!matched) {
            _robots.push_back(newObj);
        }
    }
}