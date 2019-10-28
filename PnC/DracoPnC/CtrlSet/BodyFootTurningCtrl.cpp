#include <Configuration.h>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/PlannerSet/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

BodyFootTurningCtrl::BodyFootTurningCtrl(RobotSystem* robot,
                                         std::string swing_foot,
                                         FootStepPlanner* planner)
    : SwingPlanningCtrl(robot, swing_foot, planner) {
    myUtils::pretty_constructor(2, "Body Foot Turning Ctrl");

    push_down_height_ = 0.;
    swing_height_ = 0.05;
    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jacc_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kd_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    fin_foot_z_vel_ = 0.;
    fin_foot_z_acc_ = 0.;

    if (swing_foot == "lFoot") {
        foot_task_ = new LineFootTask(robot_, DracoBodyNode::lFootCenter);
    } else {
        foot_task_ = new LineFootTask(robot_, DracoBodyNode::rFootCenter);
    }
    base_task_ = new BodyRxRyZTask(robot_);

    // contact
    rfoot_contact_ =
        new PointContactSpec(robot_, DracoBodyNode::rFootCenter, 0.3);
    lfoot_contact_ =
        new PointContactSpec(robot_, DracoBodyNode::lFootCenter, 0.3);
    contact_list_.clear();
    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    fz_idx_in_cost_.clear();
    dim_contact_ = 0;
    for (int i = 0; i < contact_list_.size(); ++i) {
        fz_idx_in_cost_.push_back(dim_contact_ +
                                  contact_list_[i]->getFzIndex());
        dim_contact_ += contact_list_[i]->getDim();
    }

    // wbc
    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for (int i(0); i < robot_->getNumVirtualDofs(); ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ =
        Eigen::VectorXd::Constant(robot_->getNumDofs(), 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    for (int i = 0; i < contact_list_.size(); ++i) {
        wblc_data_->W_rf_[fz_idx_in_cost_[i]] = 0.01;
    }
    wblc_data_->tau_min_ =
        Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), -100.);
    wblc_data_->tau_max_ =
        Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), 100.);

    kin_wbc_contact_list_.clear();
    int jidx_offset(0);
    if (swing_foot == "lFoot") {
        jidx_offset = rfoot_contact_->getDim();
        for (int i(0); i < lfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + jidx_offset] = 5.0;
            wblc_data_->W_xddot_[i + jidx_offset] = 0.001;
        }
        wblc_data_->W_rf_[fz_idx_in_cost_[1]] = 0.5;
        ((PointContactSpec*)lfoot_contact_)->setMaxFz(0.0001);
        kin_wbc_contact_list_.push_back(rfoot_contact_);
    } else if (swing_foot == "rFoot") {
        for (int i(0); i < rfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + jidx_offset] = 5.0;
            wblc_data_->W_xddot_[i + jidx_offset] = 0.001;
        }
        wblc_data_->W_rf_[fz_idx_in_cost_[0]] = 0.5;

        ((PointContactSpec*)rfoot_contact_)->setMaxFz(0.0001);
        kin_wbc_contact_list_.push_back(lfoot_contact_);
    }

    // Create Minimum jerk objects
    for (size_t i = 0; i < 3; i++) {
        min_jerk_offset_.push_back(new MinJerk_OneDimension());
    }
}

void BodyFootTurningCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma;

    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void BodyFootTurningCtrl::_contact_setup() {
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyFootTurningCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    Eigen::MatrixXd A_rotor = A_;
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        A_rotor(i + robot_->getNumVirtualDofs(),
                i + robot_->getNumVirtualDofs()) += sp_->rotor_inertia[i];
    }
    Eigen::MatrixXd A_rotor_inv = A_rotor.inverse();

    wblc_->updateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(robot_->getNumVirtualDofs(),
                                        robot_->getNumActuatedDofs())) +
        Kd_.cwiseProduct(des_jvel_ -
                         sp_->qdot.tail(robot_->getNumActuatedDofs()));

    // myUtils::pretty_print(des_jpos_, std::cout, "des_jpos");
    // myUtils::pretty_print(des_jvel_, std::cout, "des_jvel");
    // myUtils::pretty_print(des_jacc_, std::cout, "des_jacc");
    // myUtils::pretty_print(des_jacc_cmd, std::cout, "des_jacc");

    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);

    sp_->qddot_cmd = wblc_data_->qddot_;
    for (int i(0); i < dim_contact_; ++i)
        sp_->reaction_forces[i] = wblc_data_->Fr_[i];
}

void BodyFootTurningCtrl::_task_setup() {
    double base_height_cmd = ini_base_height_;
    if (b_set_height_target_) base_height_cmd = des_body_height_;

    // =========================================================================
    // Body Posture Task Setup
    // =========================================================================
    Eigen::VectorXd pos_des(7);
    pos_des.setZero();
    Eigen::VectorXd vel_des(6);
    vel_des.setZero();
    Eigen::VectorXd acc_des(6);
    acc_des.setZero();

    // only rx, ry, z are used
    pos_des[0] = 1.;
    pos_des[1] = 0.;
    pos_des[2] = 0.;
    pos_des[3] = 0.;
    pos_des[4] = 0.;
    pos_des[5] = 0.;
    pos_des[6] = base_height_cmd;

    base_task_->updateTask(pos_des, vel_des, acc_des);

    // =========================================================================
    // Foot Task Setup
    // =========================================================================
    Eigen::VectorXd foot_pos_des(7);
    foot_pos_des.setZero();
    Eigen::VectorXd foot_vel_des(6);
    foot_vel_des.setZero();
    Eigen::VectorXd foot_acc_des(6);
    foot_acc_des.setZero();

    _CheckPlanning();
    //_GetSinusoidalSwingTrajectory();
    _GetBsplineSwingTrajectory();

    double traj_time = state_machine_time_ - half_swing_time_;
    if (state_machine_time_ > half_swing_time_) {
        double pos, vel, acc;
        for (int i(0); i < 3; ++i) {
            min_jerk_offset_[i]->getPos(traj_time, pos);
            min_jerk_offset_[i]->getVel(traj_time, vel);
            min_jerk_offset_[i]->getAcc(traj_time, acc);

            curr_foot_pos_des_[i] += pos;
            curr_foot_vel_des_[i] += vel;
            curr_foot_acc_des_[i] += acc;
        }
    }

    double t(myUtils::smooth_changing(0, 1, end_time_, state_machine_time_));
    double tdot(
        myUtils::smooth_changing_vel(0, 1, end_time_, state_machine_time_));
    double tddot(
        myUtils::smooth_changing_acc(0, 1, end_time_, state_machine_time_));
    Eigen::Quaternion<double> curr_foot_delta_quat =
        dart::math::expToQuat(foot_delta_so3_ * t);
    curr_foot_quat_des_ = curr_foot_delta_quat * ini_foot_ori_;
    curr_foot_so3_des_ = foot_delta_so3_ * tdot;

    foot_pos_des[0] = curr_foot_quat_des_.w();
    foot_pos_des[1] = curr_foot_quat_des_.x();
    foot_pos_des[2] = curr_foot_quat_des_.y();
    foot_pos_des[3] = curr_foot_quat_des_.z();
    for (int i = 0; i < 3; ++i) {
        foot_vel_des[i] = curr_foot_so3_des_[i];
    }

    for (int i(0); i < 3; ++i) {
        foot_pos_des[i + 4] = curr_foot_pos_des_[i];
        foot_vel_des[i + 3] = curr_foot_vel_des_[i];
        foot_acc_des[i + 3] = curr_foot_acc_des_[i];
    }
    foot_task_->updateTask(foot_pos_des, foot_vel_des, foot_acc_des);

    // =========================================================================
    // Task push back
    // =========================================================================
    task_list_.push_back(base_task_);
    task_list_.push_back(foot_task_);

    kin_wbc_->FindConfiguration(sp_->q, task_list_, kin_wbc_contact_list_,
                                des_jpos_, des_jvel_, des_jacc_);
}

void BodyFootTurningCtrl::_CheckPlanning() {
    if ((state_machine_time_ > 0.5 * end_time_) && b_replanning_ &&
        !b_replaned_) {
        Eigen::Vector3d target_loc;
        _Replanning(target_loc);

        Eigen::Vector3d target_offset;
        // X, Y target is originally set by intial_traget_loc
        for (int i(0); i < 2; ++i)
            target_offset[i] = target_loc[i] - initial_target_loc_[i];

        // Foot height (z) is set by the initial height
        target_offset[2] = 0.;  // target_loc[2] - ini_foot_pos_[2];

        _SetMinJerkOffset(target_offset);
        b_replaned_ = true;
    }
}

void BodyFootTurningCtrl::_Replanning(Eigen::Vector3d& target_loc) {
    // Direct value used
    Eigen::Vector3d com_pos = robot_->getCoMPosition();
    Eigen::Vector3d com_vel = robot_->getCoMVelocity();

    for (int i(0); i < 2; ++i) {
        com_pos[i] = sp_->q[i] + body_pt_offset_[i];
        // com_pos[i] += body_pt_offset_[i];

        // !! TEST !!
        com_vel[i] = sp_->qdot[i];
        // com_vel[i] = sp_->est_mocap_body_vel[i];
    }

    printf("planning com state: %f, %f, %f, %f\n", com_pos[0], com_pos[1],
           com_vel[0], com_vel[1]);

    OutputReversalPL pl_output;
    ParamReversalPL pl_param;
    pl_param.swing_time = end_time_ - state_machine_time_ +
                          transition_time_ * transition_phase_ratio_ +
                          stance_time_ * double_stance_ratio_;

    pl_param.des_loc = sp_->des_location;
    pl_param.stance_foot_loc = sp_->global_pos_local;
    pl_param.yaw_angle = sp_->q[DracoDoF::baseRotZ];

    if (swing_foot_ == "lFoot")
        pl_param.b_positive_sidestep = true;
    else
        pl_param.b_positive_sidestep = false;

    Eigen::Vector3d global_com_pos = com_pos + sp_->global_pos_local;

    planner_->getNextFootLocation(global_com_pos, com_vel, target_loc,
                                  &pl_param, &pl_output);

    Eigen::VectorXd ss_global(4);
    for (int i = 0; i < 4; ++i) {
        ss_global[i] = pl_output.switching_state[i];
    }

    // Time Modification
    replan_moment_ = state_machine_time_;
    end_time_ += pl_output.time_modification;
    target_loc -= sp_->global_pos_local;

    target_loc[2] = initial_target_loc_[2];

    for (int i(0); i < 2; ++i) {
        target_loc[i] += foot_landing_offset_[i];
    }
    myUtils::pretty_print(target_loc, std::cout, "next foot location");
}

void BodyFootTurningCtrl::firstVisit() {
    b_replaned_ = false;
    ini_config_ = sp_->q;

    if (swing_foot_ == "rFoot") {
        ini_foot_pos_ = robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter)
                            .translation();
        ini_foot_ori_ = Eigen::Quaternion<double>(
            robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).linear());
    } else if (swing_foot_ == "lFoot") {
        ini_foot_pos_ = robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter)
                            .translation();
        ini_foot_ori_ = Eigen::Quaternion<double>(
            robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).linear());
    } else {
        std::cout << "Wrong swing foot" << std::endl;
        exit(0);
    }
    foot_delta_quat_ = sp_->des_quat * ini_foot_ori_.inverse();
    foot_delta_so3_ = dart::math::quatToExp(foot_delta_quat_);
    ctrl_start_time_ = sp_->curr_time;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;

    initial_target_loc_[0] = sp_->q[0] + default_target_loc_[0];
    initial_target_loc_[1] = sp_->q[1] + default_target_loc_[1];
    initial_target_loc_[2] = -push_down_height_;

    _SetBspline(ini_foot_pos_, initial_target_loc_);

    Eigen::Vector3d foot_pos_offset;
    foot_pos_offset.setZero();
    foot_pos_offset[2] = 0.;
    _SetMinJerkOffset(foot_pos_offset);

    Eigen::Vector3d ini_com_pos_ = robot_->getCoMPosition();
    Eigen::Vector3d com_vel = robot_->getCoMVelocity();

    Eigen::VectorXd input_state(4);
    input_state[0] = ini_com_pos_[0];
    input_state[1] = ini_com_pos_[1];
    input_state[2] = com_vel[0];
    input_state[3] = com_vel[1];
}

void BodyFootTurningCtrl::_SetMinJerkOffset(const Eigen::Vector3d& offset) {
    // Initialize Minimum Jerk Parameter Containers
    Eigen::Vector3d init_params;
    Eigen::Vector3d final_params;

    // Set Minimum Jerk Boundary Conditions
    for (size_t i = 0; i < 3; i++) {
        // Set Dimension i's initial pos, vel and acceleration
        init_params.setZero();
        // Set Dimension i's final pos, vel, acceleration
        final_params.setZero();
        final_params[0] = offset[i];

        min_jerk_offset_[i]->setParams(init_params, final_params, 0.,
                                       half_swing_time_);
    }
}

bool BodyFootTurningCtrl::endOfPhase() {
    if (state_machine_time_ > (end_time_)) {
        printf(
            "[Body Foot Planning Ctrl] End, state_machine time/ end time: (%f, "
            "%f)\n",
            state_machine_time_, end_time_);
        // if(b_set_height_target_)  printf("b set height target: true\n");
        // else  printf("b set height target: false\n");
        // printf("\n");
        return true;
    }
    if (b_contact_switch_check_) {
        bool contact_happen(false);
        if (swing_foot_ == "lFoot" && sp_->b_lfoot_contact) {
            contact_happen = true;
        }
        if (swing_foot_ == "rFoot" && sp_->b_rfoot_contact) {
            contact_happen = true;
        }
        if (state_machine_time_ > end_time_ * 0.5 && contact_happen) {
            printf(
                "[Body Foot Planning Ctrl] Contact Happen, state_machine_time/ "
                "end time: (%f, %f)\n",
                state_machine_time_, end_time_);
            return true;
        }
    }

    return false;
}

void BodyFootTurningCtrl::ctrlInitialization(const YAML::Node& node) {
    ini_base_height_ = sp_->q[2];
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
        myUtils::readParameter(node, "swing_height", swing_height_);
        myUtils::readParameter(node, "push_down_height", push_down_height_);
        myUtils::readParameter(node, "default_target_foot_location",
                               default_target_loc_);
        myUtils::readParameter(node, "body_pt_offset", body_pt_offset_);
        myUtils::readParameter(node, "foot_landing_offset",
                               foot_landing_offset_);

        myUtils::readParameter(node, "fin_foot_z_vel", fin_foot_z_vel_);
        myUtils::readParameter(node, "fin_foot_z_acc", fin_foot_z_acc_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
    static bool b_bodypute_eigenvalue(true);
    if (b_bodypute_eigenvalue) {
        ((Reversal_LIPM_Planner*)planner_)
            ->CheckEigenValues(double_stance_ratio_ * stance_time_ +
                               transition_phase_ratio_ * transition_time_ +
                               end_time_);
        b_bodypute_eigenvalue = false;
    }
}

BodyFootTurningCtrl::~BodyFootTurningCtrl() {
    delete base_task_;
    delete foot_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_contact_;
    delete lfoot_contact_;

    for (int i = 0; i < min_jerk_offset_.size(); ++i)
        delete min_jerk_offset_[i];
}

void BodyFootTurningCtrl::_GetBsplineSwingTrajectory() {
    double pos[3];
    double vel[3];
    double acc[3];

    foot_traj_.getCurvePoint(state_machine_time_, pos);
    foot_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
    foot_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

    for (int i(0); i < 3; ++i) {
        curr_foot_pos_des_[i] = pos[i];
        curr_foot_vel_des_[i] = vel[i];
        curr_foot_acc_des_[i] = acc[i];
    }
}
void BodyFootTurningCtrl::_GetSinusoidalSwingTrajectory() {
    curr_foot_acc_des_.setZero();
    for (int i(0); i < 2; ++i) {
        curr_foot_pos_des_[i] =
            myUtils::smooth_changing(ini_foot_pos_[i], initial_target_loc_[i],
                                     end_time_, state_machine_time_);
        curr_foot_vel_des_[i] = myUtils::smooth_changing_vel(
            ini_foot_pos_[i], initial_target_loc_[i], end_time_,
            state_machine_time_);
        curr_foot_acc_des_[i] = myUtils::smooth_changing_acc(
            ini_foot_pos_[i], initial_target_loc_[i], end_time_,
            state_machine_time_);
    }
    // for Z (height)
    double amp(swing_height_ / 2.);
    double omega(2. * M_PI / end_time_);

    curr_foot_pos_des_[2] =
        ini_foot_pos_[2] + amp * (1 - cos(omega * state_machine_time_));
    curr_foot_vel_des_[2] = amp * omega * sin(omega * state_machine_time_);
    curr_foot_acc_des_[2] =
        amp * omega * omega * cos(omega * state_machine_time_);
}

void BodyFootTurningCtrl::_SetBspline(const Eigen::Vector3d& st_pos,
                                      const Eigen::Vector3d& des_pos) {
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];
    Eigen::Vector3d middle_pos;

    middle_pos = (st_pos + des_pos) / 2.;
    middle_pos[2] = swing_height_;

    // Initial and final position & velocity & acceleration
    for (int i(0); i < 3; ++i) {
        // Initial
        init[i] = st_pos[i];
        init[i + 3] = 0.;
        init[i + 6] = 0.;
        // Final
        fin[i] = des_pos[i];
        fin[i + 3] = 0.;
        fin[i + 6] = 0.;
        // Middle
        middle_pt[0][i] = middle_pos[i];
    }
    fin[5] = fin_foot_z_vel_;
    fin[8] = fin_foot_z_acc_;
    foot_traj_.SetParam(init, fin, middle_pt, end_time_);

    delete[] * middle_pt;
    delete[] middle_pt;
}
