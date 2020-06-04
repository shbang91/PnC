#include <PnC/ScorpioPnC/CtrlSet/OSCPosCtrl.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Utils/IO/DataManager.hpp>

OSCPosCtrl::OSCPosCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "OSC POS Ctrl");
    end_time_ = 0;
    ctrl_count_ = 0;
    task_dim_ = 6;
    target_pos_ = Eigen::VectorXd::Zero(3);
    ini_pos_ = Eigen::VectorXd::Zero(3);
    ini_vel_ = Eigen::VectorXd::Zero(3);
    ini_pos_q = Eigen::VectorXd::Zero(robot_->getNumDofs());
    ini_vel_q = Eigen::VectorXd::Zero(robot_->getNumDofs());
    q_kp_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    q_kd_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    end_effector_kp_ = Eigen::VectorXd::Zero(task_dim_);
    end_effector_kd_ = Eigen::VectorXd::Zero(task_dim_);
    ini_ori_ = Eigen::Quaternion<double> (1,0,0,0);
    target_ori_ = Eigen::Quaternion<double> (1,0,0,0);
    ori_err_ = Eigen::VectorXd::Zero(3);

    des_pos_data_ = Eigen::VectorXd::Zero(7);
    act_pos_data_ = Eigen::VectorXd::Zero(7);
    des_vel_data_ = Eigen::VectorXd::Zero(6);
    act_vel_data_ = Eigen::VectorXd::Zero(6);

    ori_err_data_ = Eigen::VectorXd::Zero(3);

    _build_active_joint_idx();
    Jc_ = Eigen::MatrixXd::Zero(6,robot_->getNumDofs());
    _build_constraint_matrix();


    //Save Data
    DataManager* data_manager = DataManager::GetDataManager();     
    data_manager->RegisterData(&des_pos_data_,VECT,"des_pos",7);
    data_manager->RegisterData(&act_pos_data_,VECT,"act_pos",7);
    data_manager->RegisterData(&des_vel_data_,VECT,"des_vel",6);
    data_manager->RegisterData(&act_vel_data_,VECT,"act_vel",6);

    data_manager->RegisterData(&ori_err_data_,VECT,"ori_error",3);
}

OSCPosCtrl::~OSCPosCtrl() {}

void OSCPosCtrl::_build_active_joint_idx(){
    active_joint_idx_.resize(Scorpio::n_adof);
    active_joint_idx_[0] = 0;
    active_joint_idx_[1] = 1;
    active_joint_idx_[2] = 4;
    active_joint_idx_[3] = 5;
    active_joint_idx_[4] = 8;
    active_joint_idx_[5] = 9;
    active_joint_idx_[6] = 10;
}

void OSCPosCtrl::_build_constraint_matrix(){
    Eigen::MatrixXd J_body_1 = robot_->getBodyNodeJacobian("link2").block(3,0,3,robot_->getNumDofs()); 
    Eigen::MatrixXd J_constriant_1 = robot_->getBodyNodeJacobian("link4_end").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_constriant_diff_1 = J_constriant_1 - J_body_1;
    Jc_.block(0,0,3,robot_->getNumDofs()) =  J_constriant_diff_1;

    Eigen::MatrixXd J_body_2 = robot_->getBodyNodeJacobian("link6").block(3,0,3,robot_->getNumDofs()); 
    Eigen::MatrixXd J_constriant_2 = robot_->getBodyNodeJacobian("link8_end").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_constriant_diff_2 = J_constriant_2 - J_body_2;
    Jc_.block(3,0,3,robot_->getNumDofs()) =  J_constriant_diff_2;

}

void OSCPosCtrl::oneStep(void* _cmd) {
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Scorpio::n_adof);
    //construct selection matrix
    Eigen::MatrixXd S= Eigen::MatrixXd::Zero(Scorpio::n_adof,robot_->getNumDofs());
    std::vector<bool> act_list;
    act_list.resize(Scorpio::n_dof,true);
    act_list[2] = false;
    act_list[3] = false;
    act_list[6] = false;
    act_list[7] = false;
    int j(0);
    int i(0);
    for (int i = 0; i < Scorpio::n_dof; ++i) {
       if(act_list[i]){ 
           S(j,i) = 1.;
            ++j;
       }
    }
    //Eigen::JacobiSVD<Eigen::MatrixXd> svd1(
            //J_constraints, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //std::cout << "J_c singularValues" << std::endl;
    //std::cout << svd1.singularValues() << std::endl;
    //std::cout << "============================" << std::endl;

    Eigen::MatrixXd Jc_bar = Eigen::MatrixXd::Zero(Scorpio::n_dof,6);
    myUtils::weightedInverse(Jc_,robot_->getInvMassMatrix(),Jc_bar);
    Eigen::MatrixXd N_c = Eigen::MatrixXd::Identity(Scorpio::n_dof,Scorpio::n_dof) - Jc_bar * Jc_;
    //myUtils::pretty_print(N_c, std::cout, "N_c");

     //Eigen::JacobiSVD<Eigen::MatrixXd> svd2(
     //N_c, Eigen::ComputeThinU | Eigen::ComputeThinV);
     //std::cout << "N_c singularValues" << std::endl;
     //std::cout << svd2.singularValues() << std::endl;
     //std::cout << "============================" << std::endl;

    Eigen::MatrixXd b_c = N_c.transpose() * (robot_->getCoriolisGravity());
    Eigen::MatrixXd SN_c = S * N_c;
    //PRINT
     //Eigen::JacobiSVD<Eigen::MatrixXd> svd3(
     //SN_c, Eigen::ComputeThinU | Eigen::ComputeThinV);
     //std::cout << "SN_c singularValues" << std::endl;
     //std::cout << svd3.singularValues() << std::endl;
     //std::cout << "============================" << std::endl;
     //PRINT
    Eigen::MatrixXd SN_c_bar = Eigen::MatrixXd::Zero(Scorpio::n_dof,Scorpio::n_adof);
    myUtils::weightedInverse(SN_c, robot_->getInvMassMatrix(), SN_c_bar);

    //Eigen::JacobiSVD<Eigen::MatrixXd> svd4(
    //SN_c_bar, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //std::cout << "SN_c_bar singularValues" << std::endl;
    //std::cout << svd4.singularValues() << std::endl;
    //std::cout << "============================" << std::endl;

    // Formulating task space desired acceleration
    Eigen::VectorXd end_effector_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd end_effector_ori_error = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd end_effector_vel_des = Eigen::VectorXd::Zero(task_dim_);
    Eigen::VectorXd end_effector_acc_ff = Eigen::VectorXd::Zero(task_dim_);
    Eigen::VectorXd end_effector_acc_des = Eigen::VectorXd::Zero(task_dim_);

    Eigen::VectorXd end_effector_pos_act = robot_->getBodyNodeIsometry("end_effector").translation();
    Eigen::Quaternion<double> end_effector_ori_act = Eigen::Quaternion<double> (robot_->getBodyNodeIsometry("end_effector").linear());
    Eigen::Quaternion<double> end_effector_ori_des = Eigen::Quaternion<double> (1,0,0,0);

    Eigen::VectorXd end_effector_vel_act = robot_->getBodyNodeSpatialVelocity("end_effector"); 

    for (int i = 0; i < 3; ++i) {
            end_effector_pos_des[i] = myUtils::smooth_changing(ini_pos_[i], target_pos_[i],
                                                             end_time_, state_machine_time_);
            end_effector_vel_des[3+i] = myUtils::smooth_changing_vel(ini_pos_[i], target_pos_[i],
                                                                    end_time_, state_machine_time_);
            end_effector_acc_ff[3+i] = myUtils::smooth_changing_acc(ini_pos_[i], target_pos_[i],
                                                                    end_time_, state_machine_time_);
            end_effector_acc_des[3+i] =
                 end_effector_acc_ff[3+i] +  end_effector_kp_[3+i] * (end_effector_pos_des[i] - end_effector_pos_act[i]) 
                + end_effector_kd_[3+i] * (end_effector_vel_des[3+i] - end_effector_vel_act[3+i]);
        }
    double t = myUtils::smooth_changing(0,1,end_time_,state_machine_time_);
    end_effector_ori_des = dart::math::expToQuat(ori_err_ * t) * ini_ori_; 
    end_effector_ori_error = dart::math::quatToExp(end_effector_ori_des * (end_effector_ori_act.inverse())); 

    double tdot = myUtils::smooth_changing_vel(0,1,end_time_,state_machine_time_);
    end_effector_vel_des.head(3) = ori_err_ * tdot;

    for (int i = 0; i < 3; ++i) {
       end_effector_acc_des[i] = end_effector_kp_[i] * end_effector_ori_error[i] 
                                + end_effector_kd_[i] * (end_effector_vel_des[i] - end_effector_vel_act[i]); 
    }

    //Save Data
    des_pos_data_[0] = end_effector_ori_des.w();
    des_pos_data_[1] = end_effector_ori_des.x();
    des_pos_data_[2] = end_effector_ori_des.y();
    des_pos_data_[3] = end_effector_ori_des.z();

    act_pos_data_[0] = end_effector_ori_act.w();
    act_pos_data_[1] = end_effector_ori_act.x();
    act_pos_data_[2] = end_effector_ori_act.y();
    act_pos_data_[3] = end_effector_ori_act.z();

    for (int i = 0; i < 3; ++i) {
        des_pos_data_[i+4] = end_effector_pos_des[i];
        act_pos_data_[i+4] = end_effector_pos_act[i];
    }

    des_vel_data_ = end_effector_vel_des;
    act_vel_data_ = end_effector_vel_act; 

    ori_err_data_ = end_effector_ori_error;
    //Done saving Data
    
    //TEST
    //Eigen::MatrixXd J_end_effector = robot_->getBodyNodeJacobian("end_effector").block(3,0,3,robot_->getNumDofs());
    ////J_end_effector = J_end_effector* N_c;
    //Eigen::MatrixXd J_end_effector_bar = Eigen::MatrixXd::Zero(robot_->getNumDofs(),3);
    //myUtils::weightedInverse(J_end_effector,robot_->getInvMassMatrix(),J_end_effector_bar);
    //TEST

    Eigen::MatrixXd J_end_effector = robot_->getBodyNodeJacobian("end_effector");
    J_end_effector = J_end_effector* N_c;
    Eigen::MatrixXd J_end_effector_bar = Eigen::MatrixXd::Zero(robot_->getNumDofs(),6);
    myUtils::weightedInverse(J_end_effector,robot_->getInvMassMatrix(),J_end_effector_bar);

     //Eigen::JacobiSVD<Eigen::MatrixXd> svd(
     //J_end_effector, Eigen::ComputeThinU | Eigen::ComputeThinV);
     //std::cout << " J_end_effector" << std::endl; 
     //std::cout << svd.singularValues() << std::endl;
     //std::cout << "============================" << std::endl;

     Eigen::JacobiSVD<Eigen::MatrixXd> svd1(
     J_end_effector_bar, Eigen::ComputeThinU | Eigen::ComputeThinV);
     std::cout << " J_end_effector_bar" << std::endl; 
     std::cout << svd1.singularValues() << std::endl;
     std::cout << "============================" << std::endl;
    
    Eigen::MatrixXd N_end_effector = Eigen::MatrixXd::Identity(Scorpio::n_dof,Scorpio::n_dof) - J_end_effector_bar * J_end_effector; 
    Eigen::VectorXd qddot_des_end_effector = Eigen::VectorXd::Zero(robot_->getNumDofs());

    qddot_des_end_effector = J_end_effector_bar * (end_effector_acc_des);

   Eigen::MatrixXd J_q = S;
   J_q = J_q*N_end_effector;
   Eigen::VectorXd qddot_des_q = Eigen::VectorXd::Zero(Scorpio::n_dof); 
   Eigen::VectorXd joint_des_acc = Eigen::VectorXd::Zero(Scorpio::n_adof);
   Eigen::MatrixXd J_q_N_end_effector = J_q * N_end_effector;
   Eigen::MatrixXd J_q_N_end_effector_bar = Eigen::MatrixXd::Zero(Scorpio::n_dof,Scorpio::n_adof);
    myUtils::weightedInverse(J_q_N_end_effector,robot_->getInvMassMatrix(),J_q_N_end_effector_bar);

   for (int i = 0; i < Scorpio::n_adof; ++i) {
      joint_des_acc[i] = q_kp_[i] * (ini_pos_q[active_joint_idx_[i]] - robot_->getQ()[active_joint_idx_[i]])
                        + q_kd_[i] * (ini_vel_q[active_joint_idx_[i]] - robot_->getQdot()[active_joint_idx_[i]]);
   }

    qddot_des_q = J_q_N_end_effector_bar * (joint_des_acc); 

    myUtils::pretty_print(joint_des_acc, std::cout, "joint_des_acc");
    myUtils::pretty_print(qddot_des_q , std::cout, "qddot_des_q");

     Eigen::JacobiSVD<Eigen::MatrixXd> svd3(
     J_end_effector, Eigen::ComputeThinU | Eigen::ComputeThinV);
     std::cout << " J_q_N_end_effector" << std::endl;
     std::cout << svd3.singularValues() << std::endl;
     std::cout << "============================" << std::endl;
 
    Eigen::MatrixXd SN_c_J_q = SN_c_bar.transpose()* J_q_N_end_effector_bar;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd5(
     J_end_effector * SN_c.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
     std::cout << "J_end_SN_c_transpose" << std::endl; 
     std::cout << svd5.singularValues() << std::endl;
     std::cout << "============================" << std::endl;

     Eigen::JacobiSVD<Eigen::MatrixXd> svd6(
     J_q_N_end_effector * SN_c.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
     std::cout << "posture task feasibility" << std::endl; 
     std::cout << svd6.singularValues() << std::endl;
     std::cout << "============================" << std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd4(
     SN_c_J_q, Eigen::ComputeThinU | Eigen::ComputeThinV);
     std::cout << "S_N_J_q" << std::endl; 
     std::cout << svd4.singularValues() << std::endl;
     std::cout << "============================" << std::endl;


 
    gamma = SN_c_bar.transpose() *(robot_->getMassMatrix() * (qddot_des_end_effector + qddot_des_q) + b_c);
    //gamma = SN_c_bar.transpose() * b_c;
    //gamma.setZero();
    ((ScorpioCommand*)_cmd)->jtrq = gamma;

    ++ctrl_count_;
    state_machine_time_= ctrl_count_ * ScorpioAux::ServoRate;

}

void OSCPosCtrl::firstVisit() {
    state_machine_time_= 0.;
    ctrl_count_ = 0;
    ini_pos_ = robot_->getBodyNodeIsometry("end_effector").translation();
    //TEST
    //target_pos_ = ini_pos_;
    //TEST
    ini_vel_ = robot_->getBodyNodeSpatialVelocity("end_effector").tail(3); 
    //std::cout << "=========================" << std::endl;
    //std::cout << "initial end effector pos" << std::endl;
    //std::cout << ini_pos_ << std::endl;
    //std::cout << "=========================" << std::endl;
    ini_pos_q = robot_->getQ();
    ini_vel_q = robot_->getQdot();
    ini_ori_ = Eigen::Quaternion<double> (robot_->getBodyNodeIsometry("end_effector").linear());
    //std::cout << "=========================" << std::endl;
    //std::cout << "initial end effector ori" << std::endl;
    //std::cout << ini_ori_.w() << std::endl;
    //std::cout << ini_ori_.x() << std::endl;
    //std::cout << ini_ori_.y() << std::endl;
    //std::cout << ini_ori_.z() << std::endl;
    //std::cout << "=========================" << std::endl;
    ori_err_ = dart::math::quatToExp(target_ori_*(ini_ori_.inverse()));
    // TEST
    //ori_err_.setZero();
    // TEST
}
void OSCPosCtrl::lastVisit() {
}

bool OSCPosCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
            return true;
        }
    return false;
}

void OSCPosCtrl::ctrlInitialization(const YAML::Node& node) {
    try {
            myUtils::readParameter(node, "q_kp", q_kp_);
            myUtils::readParameter(node, "q_kd", q_kd_);
            myUtils::readParameter(node, "end_effector_kp", end_effector_kp_);
            myUtils::readParameter(node, "end_effector_kd", end_effector_kd_);
        } catch (std::runtime_error& e) {
                std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                          << __FILE__ << "]" << std::endl
                          << std::endl;
                exit(0);
            }
}
