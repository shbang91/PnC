#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Eigen/Dense>

class Interface;
class CartPoleSensorData;

class CartPoleWorldNode : public dart::gui::osg::WorldNode
{
private:
    Interface* mInterface;
    CartPoleSensorData* mSensorData;

    dart::dynamics::SkeletonPtr mSkel;
    Eigen::VectorXd mTorqueCommand;
    int mDof;

    Eigen::VectorXd mInit;
    bool mIsVisualizeTrajectory;
    std::string mTrajectoryFile;
    std::vector<std::string> mFile;
    std::string* mLine;
    Eigen::VectorXd mPos;
    Eigen::VectorXd mVel;

public:
    CartPoleWorldNode(const dart::simulation::WorldPtr & world,
                      osgShadow::MinimalShadowMap *);
    virtual ~CartPoleWorldNode();

    void customPreStep() override;
};
