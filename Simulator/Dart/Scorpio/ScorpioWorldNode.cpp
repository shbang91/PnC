#include <Configuration.h>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Simulator/Dart/Scorpio/ScorpioWorldNode.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>

#include <fstream>


ScorpioWorldNode::ScorpioWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0),  t_(0.0), servo_rate_(0.001){
    world_ = _world;
    scorpio_ = world_->getSkeleton("Scorpio_Kin");
    mGround_ = world_->getSkeleton("ground_skeleton");
    trq_lb_scorpio_ = scorpio_->getForceLowerLimits();
    trq_ub_scorpio_ = scorpio_->getForceUpperLimits();
    n_dof_scorpio_ = scorpio_->getNumDofs();
    p_dof_scorpio_ = 4;
    a_dof_scorpio_ = n_dof_scorpio_ - p_dof_scorpio_;

    q_init_ = scorpio_->getPositions();

    active_joint_idx_.resize(a_dof_scorpio_);
    active_joint_idx_[0] = scorpio_->getDof("joint1")->getIndexInSkeleton();
    active_joint_idx_[1] = scorpio_->getDof("joint2")->getIndexInSkeleton();
    active_joint_idx_[2] = scorpio_->getDof("joint5")->getIndexInSkeleton();
    active_joint_idx_[3] = scorpio_->getDof("joint6")->getIndexInSkeleton();
    active_joint_idx_[4] = scorpio_->getDof("joint9")->getIndexInSkeleton();
    active_joint_idx_[5] = scorpio_->getDof("joint10")->getIndexInSkeleton();
    active_joint_idx_[6] = scorpio_->getDof("joint11")->getIndexInSkeleton();

    passive_joint_idx_.resize(p_dof_scorpio_);
    passive_joint_idx_[0] = scorpio_->getDof("joint3")->getIndexInSkeleton();
    passive_joint_idx_[1] = scorpio_->getDof("joint4")->getIndexInSkeleton();
    passive_joint_idx_[2] = scorpio_->getDof("joint7")->getIndexInSkeleton();
    passive_joint_idx_[3] = scorpio_->getDof("joint8")->getIndexInSkeleton();

    active1_ = scorpio_->getJoint("joint1");
    active2_ = scorpio_->getJoint("joint2");
    active3_ = scorpio_->getJoint("joint5");
    active4_ = scorpio_->getJoint("joint6");
    active5_ = scorpio_->getJoint("joint9");
    active6_ = scorpio_->getJoint("joint10");
    active7_ = scorpio_->getJoint("joint11");

    scorpio_interface_ = new ScorpioInterface();
    scorpio_sensordata_ = new ScorpioSensorData();
    scorpio_cmd_ = new ScorpioCommand();

    scorpio_trq_cmd_ = Eigen::VectorXd::Zero(a_dof_scorpio_);

    SetParams_();
}


ScorpioWorldNode::~ScorpioWorldNode() {
    delete scorpio_interface_;
    delete scorpio_sensordata_;
    delete scorpio_cmd_;
}

void ScorpioWorldNode::GetActiveJointInfo(Eigen::VectorXd & cur_pos, Eigen::VectorXd & cur_vel){
    Eigen::VectorXd j_pos(n_dof_scorpio_);
    Eigen::VectorXd j_vel(n_dof_scorpio_);
    j_pos = scorpio_->getPositions();
    j_vel = scorpio_->getVelocities();

    cur_pos.resize(a_dof_scorpio_);
    cur_pos.setZero();
    cur_vel.resize(a_dof_scorpio_);
    cur_vel.setZero();

    for(int i(0); i< a_dof_scorpio_; ++i){
        cur_pos[i] = j_pos[active_joint_idx_[i]];
        cur_vel[i] = j_vel[active_joint_idx_[i]];
    }
}

void ScorpioWorldNode::SetActivePosition(const Eigen::VectorXd & des_pos){
    for(int i(0); i<a_dof_scorpio_; ++i)
        scorpio_->setPosition(active_joint_idx_[i], des_pos[i]);
}

void ScorpioWorldNode::SetActiveForce(const Eigen::VectorXd & des_force){
 
    active1_->setCommand(0,des_force[0]);
    active2_->setCommand(0,des_force[1]);
    active3_->setCommand(0,des_force[2]);
    active4_->setCommand(0,des_force[3]);
    active5_->setCommand(0,des_force[4]);
    active6_->setCommand(0,des_force[5]);
    active7_->setCommand(0,des_force[6]);
}

void ScorpioWorldNode::UpdateSystem(const Eigen::VectorXd & q_activate){
    Eigen::VectorXd q_full(n_dof_scorpio_);

    q_full[0] = q_activate[0];
    q_full[1] = q_activate[1];
    q_full[2] = -q_activate[1];
    q_full[3] = q_activate[1];
    q_full[4] = q_activate[2];
    q_full[5] = q_activate[3];
    q_full[6] = -q_activate[3];
    q_full[7] = q_activate[3];
    q_full[8] = q_activate[4];
    q_full[9] = q_activate[5];
    q_full[10] = q_activate[6];

    scorpio_->setPositions(q_full);
    scorpio_->computeForwardKinematics(true, false, false);
}

void ScorpioWorldNode::SetActiveVelocity(const Eigen::VectorXd & des_vel){

    active1_->setCommand(0,des_vel[0]);
    active2_->setCommand(0,des_vel[1]);
    active3_->setCommand(0,des_vel[2]);
    active4_->setCommand(0,des_vel[3]);
    active5_->setCommand(0,des_vel[4]);
    active6_->setCommand(0,des_vel[5]);
    active7_->setCommand(0,des_vel[6]);

}

void ScorpioWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    //Eigen::VectorXd qdot =  scorpio_->getVelocities();
    //Eigen::VectorXd zerotorque(n_dof_scorpio_);
    //zerotorque.setZero();
    //zerotorque =  -0.1*qdot;
    //scorpio_->setForces(zerotorque);

    // =============
    // Scorpio
    // =============
    scorpio_sensordata_->q = scorpio_->getPositions();
    scorpio_sensordata_->qdot = scorpio_->getVelocities();
    scorpio_interface_->getCommand(scorpio_sensordata_, scorpio_cmd_);

    scorpio_trq_cmd_ = scorpio_cmd_->jtrq;
    //scorpio_->setForces(scorpio_trq_cmd_);
    SetActiveForce(scorpio_trq_cmd_);

    //std::ofstream myout;
    //myout.open("/home/seunghyeonbang/Desktop/jpos.txt"); 
    //myout << scorpio_->getPositions() << std::endl;
    //myout.close();

    count_++;
}

void ScorpioWorldNode::SetJointSpaceControlCmd(int ctrl_case){
    if(ctrl_case != 0){
        std::cout<<"[error] Control parameter is not for joint space control in yaml file." <<std::endl;
        exit(0);
    }

    Eigen::VectorXd pos_des(a_dof_scorpio_);
    Eigen::VectorXd pos_cur(a_dof_scorpio_);
    Eigen::VectorXd vel_cur(a_dof_scorpio_);
    Eigen::VectorXd torque_des(a_dof_scorpio_);

    GetActiveJointInfo(pos_cur, vel_cur);
    switch(sim_case_){
        case 0: {
            pos_des[0] = q_init_[active_joint_idx_[0]]+0.;
            pos_des[1] = q_init_[active_joint_idx_[1]]+Amp_[1]/180.0*M_PI*sin((2*M_PI)*Freq_[1]*t_);
            pos_des[2] = q_init_[active_joint_idx_[2]]+0.;
            pos_des[3] = q_init_[active_joint_idx_[3]]+Amp_[3]/180.0*M_PI*sin((2*M_PI)*Freq_[3]*t_);
            pos_des[4] = q_init_[active_joint_idx_[4]]+0.;
            pos_des[5] = q_init_[active_joint_idx_[5]]+0.;
            pos_des[6] = q_init_[active_joint_idx_[6]]+0.;
            break;
        }
        case 1:{
            pos_des[0] = q_init_[active_joint_idx_[0]]+Amp_[0]/180.0*M_PI*sin(Freq_[0]*(2*M_PI)*t_);
            pos_des[1] = q_init_[active_joint_idx_[1]]+0.;
            pos_des[2] = q_init_[active_joint_idx_[2]]-Amp_[2]/180.0*M_PI*sin(Freq_[2]*((2*M_PI)*t_-M_PI/2))-Amp_[2]/180.0*M_PI;
            pos_des[3] = q_init_[active_joint_idx_[3]]+0.;
            pos_des[4] = q_init_[active_joint_idx_[4]]-Amp_[4]/180.0*M_PI*sin(Freq_[4]*((2*M_PI)*t_-M_PI/2))-Amp_[4]/180.0*M_PI;
            pos_des[5] = q_init_[active_joint_idx_[5]]+0.;
            pos_des[6] = q_init_[active_joint_idx_[6]]+0.;
            break;
        }
        case 2:{
            pos_des[0] = q_init_[active_joint_idx_[0]]+Amp_[0]/180.0*M_PI*sin(Freq_[0]*(2*M_PI)*t_);
            pos_des[1] = q_init_[active_joint_idx_[1]]+Amp_[1]/180.0*M_PI*sin((2*M_PI)*Freq_[1]*t_);
            pos_des[2] = q_init_[active_joint_idx_[2]]-Amp_[2]/180.0*M_PI*sin(Freq_[2]*((2*M_PI)*t_-M_PI/2))-Amp_[2]/180.0*M_PI;
            pos_des[3] = q_init_[active_joint_idx_[3]]+Amp_[3]/180.0*M_PI*sin((2*M_PI)*Freq_[3]*t_);
            pos_des[4] = q_init_[active_joint_idx_[4]]-Amp_[4]/180.0*M_PI*sin(Freq_[4]*((2*M_PI)*t_-M_PI/2))-Amp_[4]/180.0*M_PI;
            pos_des[5] = q_init_[active_joint_idx_[5]]+0.;
            pos_des[6] = q_init_[active_joint_idx_[6]]+0.;
            break;
        }
        default:
            pos_des.setZero();
    }

    switch(actuator_type_){
        case 0:{
            Eigen::VectorXd vel_des(a_dof_scorpio_);
            vel_des.setZero();

            vel_des = (pos_des- pos_cur)/servo_rate_;
            SetActiveVelocity(vel_des);
            break;
        }
        case 1:{     
            // SetActiveForce(torque_des);
            torque_des.resize(a_dof_scorpio_);
            torque_des.setZero();

            for( int i(0); i< a_dof_scorpio_; ++i ){
               torque_des[i] = scorpio_kp_[i]*(pos_des[i] - pos_cur[i]) - scorpio_kd_[i]*vel_cur[i];
            } 
            SetActiveForce(torque_des);
            break;
        }
        default:{
            UpdateSystem(pos_des);
            break;
        }
    }

    myUtils::pretty_print(pos_des, std::cout, "pos_des");
    myUtils::pretty_print(pos_cur, std::cout, "pos_cur");
   count_++;
}

void ScorpioWorldNode::SetParams_(){
    YAML::Node simulation_cfg = 
         YAML::LoadFile(THIS_COM "Config/Scorpio/SIMULATION.yaml");
     //0: servo, 1: force
    myUtils::readParameter(simulation_cfg, "actuator_type", actuator_type_);
    YAML::Node control_cfg = 
        YAML::LoadFile(THIS_COM "Config/Scorpio/CONTROL.yaml");
    myUtils::readParameter(control_cfg, "Amp", Amp_ );
    myUtils::readParameter(control_cfg, "Freq", Freq_);
    myUtils::readParameter(control_cfg, "sim_case", sim_case_);
    myUtils::readParameter(control_cfg, "control_type", control_type_);
    myUtils::readParameter(control_cfg, "kp", scorpio_kp_);
    myUtils::readParameter(control_cfg, "kd", scorpio_kd_);

    myUtils::pretty_print(Amp_, std::cout, "Amp");
    myUtils::pretty_print(Freq_, std::cout, "Freq");
    std::cout<<"sim_case: "<<sim_case_ <<std::endl;
    
    if(actuator_type_ == 0)
        std::cout<<"actuator_type: servo "<<std::endl;
    else
        std::cout<<"actuator_type: force "<<std::endl;

    if(control_type_ == 0)
        std::cout<<"control_type: joint space " <<std::endl;
    else if(control_type_ == 1)
        std::cout<<"control_type: operational space " <<std::endl;
    else
        std::cout<<"control_type: not defined " <<std::endl;

}



