#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

PitchFootTask::PitchFootTask(RobotSystem* robot,
                           const std::string & _link_name):Task(robot, 4)
{
    myUtils::pretty_constructor(3, _link_name + " X Y Z Ry Task");

    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
    link_name_ = _link_name;
}

PitchFootTask::~PitchFootTask(){}

bool PitchFootTask::_UpdateCommand(const Eigen::VectorXd & _pos_des,
                                  const Eigen::VectorXd & _vel_des,
                                  const Eigen::VectorXd & _acc_des) {

    Eigen::Quaternion<double> des_ori(_pos_des[0], _pos_des[1], _pos_des[2], _pos_des[3]);
    Eigen::Quaternion<double> ori_act;
    if (link_name_ == "rFoot") {
        ori_act = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).linear());
    } else if (link_name_ == "lFoot") {
        ori_act = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).linear());
    } else {
        std::cout << "Wrong Link Name" << std::endl;
        exit(0);
    }
    Eigen::Quaternion<double> quat_ori_err;
    quat_ori_err = des_ori * ori_act.inverse();
    Eigen::Vector3d ori_err_so3;
    ori_err_so3 = dart::math::quatToExp(quat_ori_err);

    // (Ry)
    pos_err[0] = ori_err_so3[1];
    vel_des[0] = _vel_des[1];
    acc_des[0] = _acc_des[1];

    // (x, y, z)
    Eigen::VectorXd pos_act;
    if (link_name_ == "rFoot") {
        pos_act = robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).translation();
    } else if (link_name_ == "lFoot") {
        pos_act = robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).translation();
    } else {
        std::cout << "Wrong Link Name" << std::endl;
        exit(0);
    }
    for (int i = 0; i < 3; ++i) {
        pos_err[i+1] = _pos_des[i+4] - pos_act[i];
        vel_des[i+1] = _vel_des[i+3];
        acc_des[i+1] = _acc_des[i+3];
    }

    //myUtils::pretty_print(des_ori, std::cout, "ori_des");
    //myUtils::pretty_print(ori_act, std::cout, "ori_act");
    //myUtils::pretty_print(pos_err, std::cout, "pos_err in bodyrpz");

    return true;
}

bool PitchFootTask::_UpdateTaskJacobian(){
    Eigen::MatrixXd Jtmp;
    if (link_name_ == "rFoot") {
        Jtmp = robot_->getBodyNodeJacobian(DracoBodyNode::rFootCenter);
    } else if (link_name_ == "lFoot") {
        Jtmp = robot_->getBodyNodeJacobian(DracoBodyNode::lFootCenter);
    } else {
        std::cout << "Wrong Link Name" << std::endl;
        exit(0);
    }
    // (Ry)
    Jt_.block(0 ,0 , 1, robot_->getNumDofs()) = Jtmp.block(1, 0, 1, robot_->getNumDofs());
    // (x, y, z)
    Jt_.block(1, 0, 3, robot_->getNumDofs()) = Jtmp.block(3, 0, 3, robot_->getNumDofs());


    Jt_.block(0, 0, 4, robot_->getNumVirtualDofs()) = Eigen::MatrixXd::Zero(4, robot_->getNumVirtualDofs());
    return true;
}

bool PitchFootTask::_UpdateTaskJDotQdot(){
    Eigen::VectorXd v_tmp;
    if (link_name_ == "rFoot") {
        v_tmp = robot_->getBodyNodeJacobianDot(DracoBodyNode::rFootCenter) * robot_->getQdot();
    } else if (link_name_ == "lFoot") {
        v_tmp = robot_->getBodyNodeJacobianDot(DracoBodyNode::lFootCenter) * robot_->getQdot();
    } else {
        std::cout << "Wrong Link Name" << std::endl;
        exit(0);
    }
    JtDotQdot_.segment(0, 2) = v_tmp.segment(1, 2);
    JtDotQdot_.tail(3) = v_tmp.tail(3);

    JtDotQdot_.setZero();
    return true;
}
