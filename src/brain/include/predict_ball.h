#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterPredictballNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class PredictBallTraj : public SyncActionNode
{
public:
    PredictBallTraj(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("R_meas", 0.01, "measurement noise (R)"), 
            InputPort<double>("sigma_a", 1.5, "proccess noise (Q)"),
            InputPort<double>("P0_pos", 0.25, "위치 공분산의 초기값"), 
            InputPort<double>("P0_vel", 1.0, "속도 공분산의 초기값"), 
            InputPort<double>("horizon", 0.5, "horizen초 뒤의 공을 예측"),
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
            InputPort<double>("k_friction", 0.98, "매 틱마다의 감속비"),
            InputPort<double>("ball_deceleration", 0.8, "최종 정지 위치용 감속 가속도"),
        };
    }

    NodeStatus tick() override;

private:
    // ----- 시간 dt 계산용 -----
    bool has_prev_time_{false};
    rclcpp::Time prev_time_{0, 0, RCL_ROS_TIME};

    // ----- Kalman Filter 상태 -----
    bool kf_initialized_{false};
    double x_{0.0}, y_{0.0}, vx_{0.0}, vy_{0.0};
    double P_[4][4]{};  // 0으로 초기화

    // ----- 카메라 들어오는 프레임 판별용 -----
    bool has_last_meas_{false};
    rclcpp::Time last_meas_stamp_{0, 0, RCL_ROS_TIME};

    Brain *brain;
};