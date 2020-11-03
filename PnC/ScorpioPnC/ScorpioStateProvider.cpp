#include <PnC/RobotSystem/RobotSystem.hpp>
//#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>
#include <PnC/ScorpioPnC/ScorpioStateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

ScorpioStateProvider* ScorpioStateProvider::getStateProvider(
    RobotSystem* _robot) {
    static ScorpioStateProvider state_provider_(_robot);
    return &state_provider_;
}

ScorpioStateProvider::ScorpioStateProvider(RobotSystem* _robot) {
    myUtils::pretty_constructor(1, "Scorpio State Provider");

    is_moving = false;

    //Gripper Status boolean variable
    is_closing = false;
    is_holding = false;
    is_opening = false;

    closing_opening_start_time = 0;

    phase_copy = 0;
    robot_ = _robot;
    curr_time = 0.;

    q_ = Eigen::VectorXd::Zero(Scorpio::n_dof);
    qdot_ = Eigen::VectorXd::Zero(Scorpio::n_dof);
    jpos_ini_ = Eigen::VectorXd::Zero(Scorpio::n_dof);

    act_q_ = Eigen::VectorXd::Zero(Scorpio::n_adof); 
    act_qdot_ = Eigen::VectorXd::Zero(Scorpio::n_adof); 

    endeff_pos_ = Eigen::VectorXd::Zero(3);
    endeff_ori_ = Eigen::Quaternion<double> (1,0,0,0);

    _build_active_joint_idx();
}

void ScorpioStateProvider::saveCurrentData() {
    for (int i = 0; i < active_joint_idx_.size(); ++i) {
       act_q_[i] = robot_->getQ()[active_joint_idx_[i]]; 
       act_qdot_[i] = robot_->getQdot()[active_joint_idx_[i]]; 
    }

    endeff_pos_ = robot_->getBodyNodeIsometry("end_effector").translation();
    endeff_ori_ = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry("end_effector").linear());
}
