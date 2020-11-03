#pragma once
#include <utility>

#include <Configuration.h>
#include <Utils/General/Clock.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>

class RobotSystem;

class ScorpioStateProvider {
   public:
    static ScorpioStateProvider* getStateProvider(RobotSystem* _robot);
    ~ScorpioStateProvider() {}

    void saveCurrentData();

    Clock clock;

    double curr_time;

    Eigen::VectorXd q_;
    Eigen::VectorXd qdot_;
    Eigen::VectorXd jpos_ini_;

    Eigen::VectorXd act_q_;
    Eigen::VectorXd act_qdot_;
    Eigen::VectorXd endeff_pos_;
    Eigen::Quaternion<double> endeff_ori_;

    int phase_copy;

    bool is_moving;
    bool is_closing;
    bool is_holding;
    bool is_opening;

    double closing_opening_start_time;

    void _build_active_joint_idx(){
        active_joint_idx_.resize(Scorpio::n_adof);
        active_joint_idx_[0] = 0;
        active_joint_idx_[1] = 1;
        active_joint_idx_[2] = 4;
        active_joint_idx_[3] = 5;
        active_joint_idx_[4] = 8;
        active_joint_idx_[5] = 9;
        active_joint_idx_[6] = 10;
    }

   private:
    ScorpioStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
    std::vector<int> active_joint_idx_;
};
