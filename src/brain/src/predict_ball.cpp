#include "brain.h"
#include "brain_tree.h"
#include "predict_ball.h"

#define REGISTER_PREDICT_BALL_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterPredictballNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_PREDICT_BALL_BUILDER(PredictBallTraj)

}

NodeStatus PredictBallTraj::tick()
{
    // 0) 초기값 
    double R_meas;    // measurement noise (R)
    double sigma_a;   // process noise (Q)용 가속도 표준편차
    double P0_pos;    // 초기 위치 분산
    double P0_vel;    // 초기 속도 분산
    
    // 4) 예측 단계 수정 (감속 모델 적용)
    double k = 0.95; // 감속 계수 (매 틱마다 속도가 5%씩 감소, 실험을 통해 조정 필요)
    getInput("k_friction", k);

    getInput("R_meas",  R_meas);
    getInput("sigma_a", sigma_a);
    getInput("P0_pos",  P0_pos);
    getInput("P0_vel",  P0_vel);

    // 1) 측정값 (필드 좌표계) 
    auto bPos = brain->data->ball.posToField;
    const double mx = bPos.x;   
    const double my = bPos.y;   

    // 2) dt 계산 
    // 2-1) prediction용 BT tick 계산
    const auto now = brain->get_clock()->now();
    double dt = 0.03;
    if (has_prev_time_) {
        dt = (now - prev_time_).seconds();
        dt = std::clamp(dt, 1e-3, 0.05);
    }
    prev_time_ = now;
    has_prev_time_ = true;
    // prtDebug("dt: " + to_string(dt)); // BT tick 프레임 확인 용도

    // 2-2) update용 카메라 프레임 판별
    const auto meas_stamp = brain->data->ball.timePoint;
    const bool meas_valid = brain->data->ballDetected; 

    bool new_meas = false;
    if (meas_valid) {
        if (!has_last_meas_ || meas_stamp != last_meas_stamp_) {
            new_meas = true;
        }
    }

    // 3) KF 초기화
    if (!kf_initialized_) {

        if (!meas_valid) {
            return NodeStatus::SUCCESS;
        } // 공 안 보이면 초기화하지 않음

        // 상태: [x y vx vy]  
        x_ = mx; y_ = my;
        vx_ = 0.0; vy_ = 0.0;

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                P_[i][j] = 0.0;

        P_[0][0] = P_[1][1] = P0_pos;
        P_[2][2] = P_[3][3] = P0_vel;

        kf_initialized_ = true;

        last_meas_stamp_ = meas_stamp;
        has_last_meas_ = true;

        return NodeStatus::SUCCESS;
    }

    // 4) 예측 단계 (CV: 필드 좌표계)
    // 4-1) 상태 예측
    const double vx_pred = vx_ * k;
    const double vy_pred = vy_ * k;
    const double x_pred  = x_ + vx_pred * dt;
    const double y_pred  = y_ + vy_pred * dt;

    // 4-2) 공분산 예측: P = F P F^T + Q
    const double F[4][4] = {
        { 1, 0,  k*dt, 0  },
        { 0, 1,  0,  k*dt },
        { 0, 0,  k,  0  },
        { 0, 0,  0,  k  }
    };

    const double sa2 = sigma_a * sigma_a;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    double Q[4][4] = {0};
    Q[0][0] = sa2 * (dt4 * 0.25);
    Q[1][1] = sa2 * (dt4 * 0.25);
    Q[0][2] = sa2 * (dt3 * 0.5);
    Q[2][0] = sa2 * (dt3 * 0.5);
    Q[1][3] = sa2 * (dt3 * 0.5);
    Q[3][1] = sa2 * (dt3 * 0.5);
    Q[2][2] = sa2 * (dt2);
    Q[3][3] = sa2 * (dt2);

    // FP = F * P
    double FP[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) sum += F[i][k] * P_[k][j];
            FP[i][j] = sum;
        }
    }

    // P_pred = FP * F^T + Q
    double P_pred[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) sum += FP[i][k] * F[j][k]; // F^T(k,j)=F(j,k)
            P_pred[i][j] = sum + Q[i][j];
        }
    }

    // 예측값 반영
    x_  = x_pred;
    y_  = y_pred;
    vx_ = vx_pred;
    vy_ = vy_pred;

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P_[i][j] = P_pred[i][j];

    // 5) 업데이트 단계 
    if (new_meas) { // 새로운 측정이 들어올 때만 업데이트
    const double r0 = mx - x_;
    const double r1 = my - y_;

    // S = H P H^T + R  (2x2), R = diag(R_meas, R_meas)
    const double S00 = P_[0][0] + R_meas;
    const double S01 = P_[0][1];
    const double S10 = P_[1][0];
    const double S11 = P_[1][1] + R_meas;

    double detS = S00 * S11 - S01 * S10;
    if (std::fabs(detS) < 1e-12) detS = (detS >= 0 ? 1e-12 : -1e-12);

    const double invS00 =  S11 / detS;
    const double invS01 = -S01 / detS;
    const double invS10 = -S10 / detS;
    const double invS11 =  S00 / detS;

    // K = P H^T S^-1 (4x2) : P의 첫 두 열만 사용
    const double K00 = P_[0][0] * invS00 + P_[0][1] * invS10;
    const double K01 = P_[0][0] * invS01 + P_[0][1] * invS11;

    const double K10 = P_[1][0] * invS00 + P_[1][1] * invS10;
    const double K11 = P_[1][0] * invS01 + P_[1][1] * invS11;

    const double K20 = P_[2][0] * invS00 + P_[2][1] * invS10;
    const double K21 = P_[2][0] * invS01 + P_[2][1] * invS11;

    const double K30 = P_[3][0] * invS00 + P_[3][1] * invS10;
    const double K31 = P_[3][0] * invS01 + P_[3][1] * invS11;

    // 상태 업데이트
    x_  += K00 * r0 + K01 * r1;
    y_  += K10 * r0 + K11 * r1;
    vx_ += K20 * r0 + K21 * r1;
    vy_ += K30 * r0 + K31 * r1;

    // 공분산 업데이트: P = (I - K H) P
    double Pold[4][4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            Pold[i][j] = P_[i][j];

    double IKH[4][4] = {0};
    for (int i = 0; i < 4; ++i) IKH[i][i] = 1.0;

    IKH[0][0] -= K00; IKH[0][1] -= K01;
    IKH[1][0] -= K10; IKH[1][1] -= K11;
    IKH[2][0] -= K20; IKH[2][1] -= K21;
    IKH[3][0] -= K30; IKH[3][1] -= K31;

    double Pnew[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) sum += IKH[i][k] * Pold[k][j];
            Pnew[i][j] = sum;
        }
    }

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P_[i][j] = Pnew[i][j]; 
        
    last_meas_stamp_ = meas_stamp;
    has_last_meas_ = true;
    }

    // 6) 미래 위치 예측 (horizon)
    double horizon = 0.5;
    getInput("horizon", horizon);

    const double pred_x = x_ + vx_ * horizon;
    const double pred_y = y_ + vy_ * horizon;

    // Pose2D로 변환 및 저장 (필드 좌표계)
    Pose2D Pred_ball;
    Pred_ball.x = pred_x;
    Pred_ball.y = pred_y;
    brain->data->Pred_ball = Pred_ball;

    // 6) 최종 위치 예측 (Final Stopping Position) - a(v) exponential model
    double a_min = 0.1;   // 고속에서 감속(작게)  (m/s^2)
    double a_max = 0.8;   // 저속에서 감속(크게)  (m/s^2)
    double k_av  = 4.0;   // v에 따른 전이 강도

    getInput("a_min", a_min);
    getInput("a_max", a_max);
    getInput("k_av",  k_av);

    a_min = std::max(a_min, 1e-3);
    a_max = std::max(a_max, a_min + 1e-3);
    k_av  = std::max(k_av,  1e-3);

    const double vx = vx_;
    const double vy = vy_;
    const double v  = std::sqrt(vx*vx + vy*vy);

    Pose2D Final_ball_pos;
    Final_ball_pos.x = x_;
    Final_ball_pos.y = y_;

    double a_eff = a_min;  // 디버그용 기본값
    double stop_dist = 0.0;

    if (v > 0.005) {
        // a(v) = a_min + (a_max-a_min)*exp(-k*v)
        a_eff = a_min + (a_max - a_min) * std::exp(-k_av * v);
        stop_dist = (v*v) / (2.0 * a_eff);

        const double ux = vx / v;
        const double uy = vy / v;

        Final_ball_pos.x = x_ + ux * stop_dist;
        Final_ball_pos.y = y_ + uy * stop_dist;
    }

    brain->data->Final_ball_pos = Final_ball_pos;


    // 7) 시각화 (rerun) - 필드 좌표계
    brain->log->setTimeNow();

    // --- FINAL 디버그 로그도 a_eff 기반으로 바꾸기 ---
    {
        double dx = Final_ball_pos.x - x_;
        double dy = Final_ball_pos.y - y_;
        double dF = std::hypot(dx, dy);

        std::ostringstream oss;
        oss << "[FINAL_DBG] "
            << "v=" << v
            << " a_eff=" << a_eff
            << " stop_dist=" << stop_dist
            << " | x_= (" << x_ << "," << y_ << ")"
            << " Final= (" << Final_ball_pos.x << "," << Final_ball_pos.y << ")"
            << " dF=" << dF;

        brain->log->log("debug/final_ball", rerun::TextLog(oss.str()));
    }

    {
    std::ostringstream oss;
    oss << "[VEL_DBG] "
        << "new_meas=" << (new_meas ? 1 : 0)
        << " dt=" << dt
        << " mx-my=(" << mx << "," << my << ")"
        << " x_-y_= (" << x_ << "," << y_ << ")"
        << " vx_=" << vx_ << " vy_=" << vy_
        << " v=" << std::sqrt(vx_*vx_ + vy_*vy_);

    brain->log->log("debug/velocity", rerun::TextLog(oss.str()));
        }

    // tick 끝부분(업데이트/예측 후) 어디든
    static double prev_v = 0.0;
    static bool has_prev_v = false;

    if (has_prev_v && dt > 1e-3 && dt < 0.1) {
    double a_est = (prev_v - v) / dt;  // 감속이므로 보통 +가 정상
    std::ostringstream oss;
    oss << "[A_EST] v_prev=" << prev_v << " v=" << v << " dt=" << dt << " a_est=" << a_est << " stop_dist=" << stop_dist;
    brain->log->log("debug/a_est", rerun::TextLog(oss.str()));
    }
    prev_v = v;
    has_prev_v = true;


    double ctPosx, ctPosy;
    getInput("ctPosx", ctPosx);
    getInput("ctPosy", ctPosy);
    double cx = ctPosx, cy = ctPosy;

    const rerun::components::Vector2D measured_ball{(float)(mx - cx), (float)(-(my - cy))};
    const rerun::components::Vector2D predicted_ball{(float)(pred_x - cx), (float)(-(pred_y - cy))};

    // if (new_meas) { 
    // brain->log->log(
    //     "field/measured_ball",
    //     rerun::Arrows2D::from_vectors({measured_ball})
    //         .with_origins({{-4.5, 0.0}})
    //         .with_colors({0x00FF00FF})
    //         .with_radii(0.01f)
    //         .with_draw_order(30)
    // );
    // }

    // brain->log->log(
    //     "field/predicted_ball",
    //     rerun::Arrows2D::from_vectors({predicted_ball})
    //         .with_origins({{-4.5, 0.0}})
    //         .with_colors({0xFFAA00FF})
    //         .with_radii(0.015f)
    //         .with_draw_order(32)
    // );

    const rerun::components::Vector2D final_ball_vec{(float)(Final_ball_pos.x - cx), (float)(-(Final_ball_pos.y - cy))};

    brain->log->log(
        "field/final_stop_pos",
        rerun::Points2D({{(float)Final_ball_pos.x, -(float)Final_ball_pos.y}}) 
            .with_colors({0xFF0000FF}) 
            .with_radii(0.05f)
    );

    return NodeStatus::SUCCESS;
}