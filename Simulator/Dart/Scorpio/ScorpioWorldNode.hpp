#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <unsupported/Eigen/CXX11/Tensor>

class EnvInterface;
class ScorpioSensorData;
class ScorpioCommand;

class ScorpioWorldNode : public dart::gui::osg::WorldNode {
   private:

    EnvInterface* scorpio_interface_;
    ScorpioSensorData* scorpio_sensordata_;
    ScorpioCommand* scorpio_cmd_;

    void SetParams_();

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr scorpio_;
    //dart::dynamics::SkeletonPtr mSkel_hsr_;
    dart::dynamics::SkeletonPtr mGround_;

    dart::dynamics::JointPtr active1_;
    dart::dynamics::JointPtr active2_;
    dart::dynamics::JointPtr active3_;
    dart::dynamics::JointPtr active4_;
    dart::dynamics::JointPtr active5_;
    dart::dynamics::JointPtr active6_;
    dart::dynamics::JointPtr active7_;
       
    Eigen::VectorXd trq_cmd_scorpio_;
    Eigen::VectorXd trq_lb_scorpio_;
    Eigen::VectorXd trq_ub_scorpio_;

    Eigen::VectorXd active_joint_idx_;
    Eigen::VectorXd passive_joint_idx_;
    Eigen::VectorXd q_init_;

    Eigen::VectorXd Amp_;
    Eigen::VectorXd Freq_;

    Eigen::VectorXd scorpio_kp_;
    Eigen::VectorXd scorpio_kd_;
    Eigen::VectorXd scorpio_trq_cmd_;

    int count_;
    int sim_case_;
    int control_type_;
    double servo_rate_;
    int n_dof_scorpio_;
    int a_dof_scorpio_;
    int p_dof_scorpio_;
    int actuator_type_;
    double t_;

    Eigen::VectorXd a_joint_idx_scorpio_;
    Eigen::VectorXd p_joint_idx_scorpio_;

    void GetActiveJointInfo(Eigen::VectorXd & pos_cur, Eigen::VectorXd & vel_cur);
    void SetActivePosition(const Eigen::VectorXd & des_pos);
    void SetActiveForce(const Eigen::VectorXd & des_force);
    void SetActiveVelocity(const Eigen::VectorXd & des_vel);
    void UpdateSystem(const Eigen::VectorXd & q_activate);

    void SetJointSpaceControlCmd(int ctrl_case);
    void SetOperationalSpaceControlCmd(int ctrl_case);
   public:
    ScorpioWorldNode(const dart::simulation::WorldPtr& world);
    virtual ~ScorpioWorldNode();

    void customPreStep() override;

};
