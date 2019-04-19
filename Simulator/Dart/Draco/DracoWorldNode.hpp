#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>

#include <Utils/General/Clock.hpp>
#include <unsupported/Eigen/CXX11/Tensor>

class EnvInterface;
class DracoSensorData;
class DracoCommand;
class DracoLedPosAnnouncer;

class DracoWorldNode : public dart::gui::osg::WorldNode {
   private:
    EnvInterface* mInterface;
    DracoSensorData* mSensorData;
    DracoCommand* mCommand;

    void _get_imu_data(Eigen::VectorXd& ang_vel, Eigen::VectorXd& acc);
    void _check_foot_contact(bool& rfoot_contact, bool& lfoot_contact);
    void _check_collision();
    void _hold_xy();
    void _hold_rot();
    void UpdateLedData_();
    void PlotTargetLocation_();
    void UpdateCameraPos_();
    void PlotAdjustedFootLocation_();
    void PlotGuidedFootLocation_();
    void SetParameters_();
    void CalculateZMP_();

    dart::dynamics::SkeletonPtr mSkel;
    dart::dynamics::SkeletonPtr mGround;
    dart::dynamics::SkeletonPtr mStar;
    dart::dynamics::SkeletonPtr mTorus;
    Eigen::VectorXd mTorqueCommand;
    Eigen::VectorXd q_init_;
    int mDof;
    double mReleaseTime;
    Eigen::VectorXd upper_cart_;
    Eigen::VectorXd lower_cart_;
    Eigen::VectorXi num_cart_;
    double delta_cart_;

    int count_;
    double waiting_time_;
    int mpi_idx_;
    int env_idx_;
    double t_;
    bool b_check_collision_;
    bool b_print_computation_time;
    double servo_rate_;
    bool b_plot_target_;
    bool b_plot_guided_foot_;
    bool b_plot_adjusted_foot_;
    bool b_camera_manipulator_;
    bool b_show_viewer_;
    bool b_parallel_;
    bool b_calculate_zmp_;

    void UpdateSystem(Eigen::VectorXd&);
    void UpdateWorkspace(Eigen::Vector3d&, Eigen::Tensor<double, 3>&);
    void ComputeWorkspace(double);
    void PrepareWorkspaceAnalysis(double);

   public:
    DracoWorldNode(const dart::simulation::WorldPtr& world,
                   osgShadow::MinimalShadowMap*);
    DracoWorldNode(const dart::simulation::WorldPtr& world,
                   osgShadow::MinimalShadowMap*, int mpi_idx, int env_idx);
    virtual ~DracoWorldNode();

    void customPreStep() override;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
    Eigen::VectorXd q_sim_;

    // workspace
    Eigen::VectorXd q_limit_u_;
    Eigen::VectorXd q_limit_l_;
    double dq_input_;
    double dx_input_;

    dart::simulation::WorldPtr world_;

    DracoLedPosAnnouncer* led_pos_announcer_;
    Clock clock_;
};
