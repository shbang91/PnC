#include <Configuration.h>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Simulator/Dart/Scorpio/ScorpioWorldNode.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>


ScorpioWorldNode::ScorpioWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0),  t_(0.0), servo_rate_(0.001){
    world_ = _world;
    scorpio_ = world_->getSkeleton("Scorpio_Kin");
    mGround_ = world_->getSkeleton("ground_skeleton");
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

    SetParams_();
}


ScorpioWorldNode::~ScorpioWorldNode() {
    delete scorpio_interface_;
    delete scorpio_sensordata_;
    delete scorpio_cmd_;
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


void ScorpioWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    scorpio_sensordata_->q = scorpio_->getPositions();
    scorpio_sensordata_->qdot = scorpio_->getVelocities();


    // ================================================
    // Scorpio grasp the box and hand it to Draco
    // ================================================

    static bool b_move_cmd(true);
    if (((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_move_cmd) {
        std::cout << "First Moving Command Received" << std::endl;
        //((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(-0.5, 0.3, 1.3, 0.7071, 0.000316, -0.7071 ,0.00025);
        ((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(-0.5, -0.3, 1.3, 0.7071, 0.000316, -0.7071 ,0.00025);
        //((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(-0.5, 0.4, 1.5, 0.7071, 0.000316, -0.7071 ,0.00025);
        b_move_cmd = false;
    }

    //static bool b_grasp_cmd(true);
    //if (((ScorpioInterface*)scorpio_interface_)->IsReadyToGrasp() && b_grasp_cmd) {
        //std::cout << "First Grasping Command Received" << std::endl;
        //((ScorpioInterface*)scorpio_interface_)->Grasp();
        //b_grasp_cmd = false;
    //}

    //static bool b_move_while_hold_cmd(true);
    //if (((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_move_while_hold_cmd) {
        //std::cout << "First Moving While Holding Command Received" << std::endl;
        //print_position();
        //Eigen::Vector3d des_rel_pos(0.1,0.1,0.06);
        //((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(des_rel_pos[0], des_rel_pos[2], des_rel_pos[3]);
        //b_move_while_hold_cmd = false;
    //}

    //static bool b_release_cmd(true);
    //if (((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_release_cmd) {
        //std::cout << "First Release Command Received" << std::endl;
        //((ScorpioInterface*)scorpio_interface_)->Release();
        //b_release_cmd = false;
    //}
    //static bool b_move_back_to_home(true);
    //if (((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_move_back_to_home) {
        //Eigen::VectorXd rel_pos =  first_scorpio_ini_pos - scorpio_->getBodyNode("end_effector")->getTransform().translation() ;
        //std::cout << "rel pos : " << rel_pos << std::endl;
        //((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(rel_pos[0], rel_pos[1], rel_pos[2]);
        //b_move_back_to_home = false;
    //}

    //if (!b_move_cmd && !b_grasp_cmd && !b_move_while_hold_cmd &&!b_release_cmd && !b_move_back_to_home) {
        //first_scorpio_done = true;
    //}

    // ======================================================================
    // Second Scorpio Pick up the box on top of Draco and put it on the table
    // ======================================================================

    //static bool b_move_second_cmd(true);
    //if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToMove() && b_move_second_cmd && ((DracoInterface*)draco_interface_)->IsReadyForNextCommand()) {
        //Eigen::VectorXd rel_pos =  mbox_->getBodyNode("baseLink")->getTransform().translation() - scorpio2_->getBodyNode("end_effector")->getTransform().translation() ;
        //((Scorpio2Interface*)scorpio_interface2_)->MoveEndEffectorTo(rel_pos[0], rel_pos[1], rel_pos[2]+0.05);
        //b_move_second_cmd = false;
    //}

    //static bool b_grasp_second_cmd(true);
    //if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToGrasp() && b_grasp_second_cmd) {
        //((Scorpio2Interface*)scorpio_interface2_)->Grasp();
        //b_grasp_second_cmd = false;
    //}

    //static bool b_move_while_grasp_second_cmd(true);
    //if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToMove() && b_move_while_grasp_second_cmd) {
        //box_ph = BoxPH::scorpio2;
        //Eigen::VectorXd rel_pos =  second_scorpio_ini_pos - scorpio2_->getBodyNode("end_effector")->getTransform().translation() ;
        //((Scorpio2Interface*)scorpio_interface2_)->MoveEndEffectorTo(rel_pos[0]-0.2, rel_pos[1]-0.1, rel_pos[2]+0.05);
        //b_move_while_grasp_second_cmd = false;
    //}


    //static bool b_release_second_cmd(true);
    //if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToMove() && b_release_second_cmd) {
        //b_release_second_cmd = false;
    //}

    //static bool b_move_to_home_second(true);
    //if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToMove() && b_move_to_home_second) {
        //box_ph = BoxPH::table2;
        //box_fin_pos = mbox_->getBodyNode("baseLink")->getTransform().translation();
        //box_fin_pos[2] = box_ini_pos[2];
        //Eigen::VectorXd rel_pos =  second_scorpio_ini_pos - scorpio2_->getBodyNode("end_effector")->getTransform().translation() ;
        //((Scorpio2Interface*)scorpio_interface2_)->MoveEndEffectorTo(rel_pos[0], rel_pos[1], rel_pos[2]);
        //b_move_to_home_second = false;
    //}

    //if (!b_move_second_cmd && !b_grasp_second_cmd && !b_move_while_grasp_second_cmd) {
        //second_scorpio_done = true;
    //} else {
        // do nothing
    //}

    scorpio_interface_->getCommand(scorpio_sensordata_, scorpio_cmd_);
    scorpio_trq_cmd_ = scorpio_cmd_->jtrq;
    SetActiveForce(scorpio_trq_cmd_);


    //if (box_ph == BoxPH::table) {
        //des_box_pos = box_ini_pos;
    //} else if (box_ph == BoxPH::scorpio) {
        //des_box_pos = scorpio_->getBodyNode("end_effector")->getTransform().translation();
        //des_box_pos[2] -= 0.05;
    //} else if (box_ph == BoxPH::draco) {
        //des_box_pos = draco_->getBodyNode("IMU")->getTransform().translation();
        //des_box_pos[2] += 0.05;
    //} else if (box_ph == BoxPH::scorpio2) {
        //des_box_pos = scorpio2_->getBodyNode("end_effector")->getTransform().translation();
        //des_box_pos[2] -= 0.05;
    //} else if (box_ph == BoxPH::table2) {
        //des_box_pos = box_fin_pos;
    //}
    //for (int i = 0; i < 3; ++i) {
        //des_box_pos_6d[i] = des_box_pos[i];
    //}

    //Eigen::VectorXd box_qddot_des = Eigen::VectorXd::Zero(6);
    //Eigen::VectorXd box_forces = Eigen::VectorXd::Zero(6);
    //for (int i = 0; i < 3; ++i) {
        //box_qddot_des[i] = box_kp[i] * (des_box_pos[i] - mbox_->getPositions()[i])
            //+ box_kd[i] * (- mbox_->getVelocities()[i]);
    //}
    //box_forces = mbox_->getMassMatrix() * box_qddot_des + mbox_->getCoriolisAndGravityForces();
    //mbox_->setForces(box_forces);
    //des_box_pos_6d += 0.00
    //mbox_->setPositions(des_box_pos_6d);

    count_++;
}

void ScorpioWorldNode::SetParams_(){
    YAML::Node simulation_cfg = 
         YAML::LoadFile(THIS_COM "Config/Scorpio/SIMULATION.yaml");
    // 0: servo, 1: force
    myUtils::readParameter(simulation_cfg, "actuator_type", actuator_type_);
    YAML::Node control_cfg = 
        YAML::LoadFile(THIS_COM "Config/Scorpio/CONTROL.yaml");
    myUtils::readParameter(control_cfg, "Amp", Amp_ );
    myUtils::readParameter(control_cfg, "Freq", Freq_);
    myUtils::readParameter(control_cfg, "sim_case", sim_case_);
    myUtils::readParameter(control_cfg, "control_type", control_type_);
    myUtils::readParameter(control_cfg, "kp", scorpio_kp_);
    myUtils::readParameter(control_cfg, "kd", scorpio_kd_);
    }



void ScorpioWorldNode::print_position(){
    Eigen::VectorXd scorpio1_;
    scorpio1_ = scorpio_->getBodyNode("end_effector")->getTransform().translation();
    myUtils::pretty_print(scorpio1_, std::cout, "Scorpio Endeffector pos");
}
