#include <PnC/WBC/OSC.hpp>
#include <PnC/ScorpioPnC/CtrlSet/GripperCtrl.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Utils/IO/DataManager.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <PnC/ScorpioPnC/ScorpioStateProvider.hpp>
#include <PnC/ScorpioPnC/TaskSet/SelectedJointTask.hpp>

GripperCtrl::GripperCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "Gripper Ctrl");

    q_kp_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    q_kd_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    ini_pos_q_ = Eigen::VectorXd::Zero(robot_->getNumDofs());

    _build_active_joint_idx();
    joint_task_ = new SelectedJointTask(robot_, active_joint_idx_);

    _build_constraint_matrix();
    osc_ = new OSC(active_joint_, &Jc_);
    sp_ = ScorpioStateProvider::getStateProvider(_robot);
}

GripperCtrl::~GripperCtrl() {}

void GripperCtrl::_build_active_joint_idx(){
    active_joint_idx_.resize(Scorpio::n_adof);
    active_joint_idx_[0] = 0;
    active_joint_idx_[1] = 1;
    active_joint_idx_[2] = 4;
    active_joint_idx_[3] = 5;
    active_joint_idx_[4] = 8;
    active_joint_idx_[5] = 9;
    active_joint_idx_[6] = 10;

    active_joint_.resize(Scorpio::n_dof, true);
    active_joint_[2] = false;
    active_joint_[3] = false;
    active_joint_[6] = false;
    active_joint_[7] = false;
}

void GripperCtrl::_build_constraint_matrix(){
    Jc_ = Eigen::MatrixXd::Zero(6, Scorpio::n_dof);
    Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(6,Scorpio::n_dof);
    Eigen::MatrixXd J_body_1 = robot_->getBodyNodeJacobian("link2").block(3,0,3,robot_->getNumDofs()); 
    Eigen::MatrixXd J_constriant_1 = robot_->getBodyNodeJacobian("link4_end").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_constriant_diff_1 = J_constriant_1 - J_body_1;
    Jc_.block(0,0,3,robot_->getNumDofs()) =  J_constriant_diff_1;

    Eigen::MatrixXd J_body_2 = robot_->getBodyNodeJacobian("link6").block(3,0,3,robot_->getNumDofs()); 
    Eigen::MatrixXd J_constriant_2 = robot_->getBodyNodeJacobian("link8_end").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_constriant_diff_2 = J_constriant_2 - J_body_2;
    Jc_.block(3,0,3,robot_->getNumDofs()) =  J_constriant_diff_2;
}

void GripperCtrl::oneStep(void* _cmd) {
    if ((sp_->curr_time > sp_->closing_opening_start_time + 3.) && (sp_->is_closing)) {
       sp_->is_closing = false;
       sp_->closing_opening_start_time = 0;
       sp_->is_holding = true;
    }
    if ((sp_->curr_time > sp_->closing_opening_start_time + 3.) && (sp_->is_opening)) {
       sp_->is_opening = false;
       sp_->closing_opening_start_time = 0;
    }

    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    _task_setup();
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Scorpio::n_adof);
    _compute_torque(gamma);
    _PostProcessing_Command();

    ((ScorpioCommand*)_cmd)->jtrq = gamma;

    if (sp_->is_closing) {
        ((ScorpioCommand*)_cmd)->gripper_cmd = GRIPPER_STATUS::is_closing;
    }    
    else if (sp_->is_holding) {
        ((ScorpioCommand*)_cmd)->gripper_cmd = GRIPPER_STATUS::is_holding;
    }    
    else if (sp_->is_opening) {
        ((ScorpioCommand*)_cmd)->gripper_cmd = GRIPPER_STATUS::is_opening;
    } else{
        ((ScorpioCommand*)_cmd)->gripper_cmd = GRIPPER_STATUS::idle;
    }   
}

void GripperCtrl::_task_setup(){
    // =========================================================================
    // Joint Task
    // =========================================================================
    Eigen::VectorXd jpos_des = Eigen::VectorXd::Zero(Scorpio::n_adof);
    for (int i = 0; i < Scorpio::n_adof; ++i) {
        jpos_des[i] = ini_pos_q_[active_joint_idx_[i]];
    }
    Eigen::VectorXd jvel_des = Eigen::VectorXd::Zero(Scorpio::n_adof);
    Eigen::VectorXd jacc_des = Eigen::VectorXd::Zero(Scorpio::n_adof);
    joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // =========================================================================
    // Stack Task List
    // =========================================================================
    task_list_.push_back(joint_task_);
}

void GripperCtrl::_compute_torque(Eigen::VectorXd & gamma){
    _build_constraint_matrix();
    osc_->updateSetting(A_, Ainv_, coriolis_, grav_, &Jc_);
    osc_->makeTorque(task_list_, contact_list_, gamma);
}


void GripperCtrl::firstVisit() {
    state_machine_time_= 0.;
    ctrl_start_time_ = sp_->curr_time;
    ini_pos_q_ = robot_->getQ();
}

void GripperCtrl::lastVisit() {
}

bool GripperCtrl::endOfPhase() {
    if (sp_->is_moving) {
        return true;
    }
    return false;
}

void GripperCtrl::ctrlInitialization(const YAML::Node& node) {
    // try {
    //     myUtils::readParameter(node, "q_kp", q_kp_);
    //     myUtils::readParameter(node, "q_kd", q_kd_);
    //     joint_task_->setGain(q_kp_, q_kd_);
    // } catch (std::runtime_error& e) {
    //     std::cout << "Error reading parameter [" << e.what() << "] at file: ["
    //         << __FILE__ << "]" << std::endl
    //         << std::endl;
    //     exit(0);
    // }
    //std::vector<double> q_kp = {100, 100  , 100 , 100 , 100 , 100 , 100};
    q_kp_ << 100, 100, 100, 100, 100, 100, 100;
    //std::vector<double> q_kd = {20     , 20   , 20  , 20  , 20  , 20  , 20};
    q_kd_ << 20, 20, 20, 20, 20, 20, 20;
    joint_task_->setGain(q_kp_, q_kd_);
}
