#include <Configuration.h>
#include <Simulator/Dart/Draco/DracoWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

void displayJointFrames(const dart::simulation::WorldPtr& world,
                        const dart::dynamics::SkeletonPtr& robot) {
    for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        for (std::size_t j = 0; j < bn->getNumChildJoints(); ++j) {
            const dart::dynamics::Joint* joint = bn->getChildJoint(j);
            const Eigen::Isometry3d offset =
                joint->getTransformFromParentBodyNode();

            dart::gui::osg::InteractiveFramePtr frame =
                std::make_shared<dart::gui::osg::InteractiveFrame>(
                    bn, joint->getName() + "/frame", offset);

            for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                                    dart::gui::osg::InteractiveTool::PLANAR})
                for (std::size_t i = 0; i < 3; ++i)
                    frame->getTool(type, i)->setEnabled(false);

            world->addSimpleFrame(frame);
        }
    }
}

void addTargetFrame(const dart::simulation::WorldPtr& world) {
    dart::gui::osg::InteractiveFramePtr frame =
        std::make_shared<dart::gui::osg::InteractiveFrame>(
            dart::dynamics::Frame::World(), "target_frame");
    for (int i = 0; i < 3; ++i) {
        frame->getTool(dart::gui::osg::InteractiveTool::PLANAR, i)
            ->setEnabled(false);
        frame->getTool(dart::gui::osg::InteractiveTool::ANGULAR, i)
            ->setEnabled(false);
    }
    world->addSimpleFrame(frame);
}

class OneStepProgress : public osgGA::GUIEventHandler {
   public:
    OneStepProgress(DracoWorldNode* worldnode) : worldnode_(worldnode) {}

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& /*aa*/) {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
            if (ea.getKey() == 'f') {
                int numStepProgress(50);
                for (int i = 0; i < numStepProgress; ++i) {
                    worldnode_->customPreStep();
                    worldnode_->getWorld()->step();
                    worldnode_->customPostStep();
                }
                return true;
            }
        }
        return false;
    }
    DracoWorldNode* worldnode_;
};

void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        // std::cout << i << "th" << std::endl;
        // std::cout << joint->isPositionLimitEnforced() << std::endl;
        joint->setPositionLimitEnforced(true);
    }
}

void _printRobotModel(dart::dynamics::SkeletonPtr robot) {
    // for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
    // dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
    // std::cout << i << "th" << std::endl;
    // std::cout << bn->getName() << std::endl;
    // std::cout << bn->getMass() << std::endl;
    //}

    // for (int i = 0; i < robot->getNumJoints(); ++i) {
    // dart::dynamics::Joint* joint = robot->getJoint(i);
    // std::cout << i << "th" << std::endl;
    // std::cout << joint->getNumDofs() << std::endl;
    //}

    // for (int i = 0; i < robot->getNumDofs(); ++i) {
    // dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
    // std::cout << i << "th" << std::endl;
    // std::cout << dof->getName() << std::endl;
    // std::cout << "child body node name : " <<
    // dof->getChildBodyNode()->getName() << std::endl; std::cout <<
    // dof->getCoulombFriction() << std::endl;
    //}

    // std::cout << "num dof" << std::endl;
    // std::cout << robot->getNumDofs() << std::endl;
    // std::cout << robot->getNumJoints() << std::endl;
    // std::cout << "mass mat row" << std::endl;
    // std::cout << robot->getMassMatrix().rows() << std::endl;
    // std::cout << robot->getMassMatrix().cols() << std::endl;
    // std::cout << "q" << std::endl;
    // std::cout << robot->getPositions() << std::endl;
    // std::cout << "robot total mass" << std::endl;
    // std::cout << robot->getMass() << std::endl;
    //std::cout << "robot position" << std::endl;
    //std::cout << robot->getPositions() << std::endl;

    exit(0);
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
    int lKneeIdx = robot->getDof("lKnee")->getIndexInSkeleton();
    int lHipPitchIdx = robot->getDof("lHipPitch")->getIndexInSkeleton();
    int rKneeIdx = robot->getDof("rKnee")->getIndexInSkeleton();
    int rHipPitchIdx = robot->getDof("rHipPitch")->getIndexInSkeleton();
    int lAnkleIdx = robot->getDof("lAnkle")->getIndexInSkeleton();
    int rAnkleIdx = robot->getDof("rAnkle")->getIndexInSkeleton();

    int initPos(1);  // 0 : Home, 1 : Simulation, 2 : Experiment
    Eigen::VectorXd q = robot->getPositions();

    switch (initPos) {
        case 0: {
            q[2] = 1.425;
            q[lAnkleIdx] = 0.;
            q[rAnkleIdx] = 0.;
            // q[lAnkleIdx] = M_PI/2;
            // q[rAnkleIdx] = M_PI/2;
            break;
        }
        case 1: {
            q[2] = 0.9;
            double alpha(-M_PI / 4.);
            double beta(M_PI / 5.5);
            q[lHipPitchIdx] = alpha;
            q[lKneeIdx] = beta - alpha;
            q[rHipPitchIdx] = alpha;
            q[rKneeIdx] = beta - alpha;
            q[lAnkleIdx] = M_PI / 2 - beta;
            q[rAnkleIdx] = M_PI / 2 - beta;
            break;
        }
        case 2: {
            YAML::Node simulation_cfg =
                YAML::LoadFile(myUtils::GetCurrentWorkingDirecotry() + "/../Config/Draco/SIMULATION.yaml");
            double hanging_height(0.0);
            myUtils::readParameter(simulation_cfg, "hanging_height",
                                   hanging_height);
            Eigen::VectorXd init_config;
            myUtils::readParameter(simulation_cfg, "initial_configuration",
                                   init_config);
            q[2] = hanging_height;
            q.tail(10) = init_config;
            break;
        }
        default:
            std::cout << "[wrong initial pos case] in Draco/Main.cpp"
                      << std::endl;
    }

    robot->setPositions(q);
}

int main(int argc, char** argv) {
    // =========================================================================
    // Parse Yaml for Simulator
    // =========================================================================
    bool isRecord;
    bool b_display_joint_frame;
    bool b_display_target_frame;
    bool b_joint_limit_enforced;
    double servo_rate;
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(myUtils::GetCurrentWorkingDirecotry() + "/../Config/Draco/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "is_record", isRecord);
        myUtils::readParameter(simulation_cfg, "display_joint_frame",
                               b_display_joint_frame);
        myUtils::readParameter(simulation_cfg, "display_target_frame",
                               b_display_target_frame);
        myUtils::readParameter(simulation_cfg, "joint_limit_enforced",
                               b_joint_limit_enforced);
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

    // =========================================================================
    // Generate world and add skeletons
    // =========================================================================
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
        myUtils::GetCurrentWorkingDirecotry() + "/../RobotModel/Ground/ground_terrain.urdf");
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
        myUtils::GetCurrentWorkingDirecotry() + "/../RobotModel/Robot/Draco/DracoSim_Dart.urdf");
    world->addSkeleton(ground);
    world->addSkeleton(robot);

    // =========================================================================
    // Friction & Restitution Coefficient
    // =========================================================================
    double friction(10.);
    double restit(0.0);
    ground->getBodyNode("ground_link")->setFrictionCoeff(friction);
    robot->getBodyNode("Torso")->setFrictionCoeff(friction);
    ground->getBodyNode("ground_link")->setRestitutionCoeff(restit);
    robot->getBodyNode("Torso")->setRestitutionCoeff(restit);

    // robot->getBodyNode("rFootFront")->setFrictionCoeff(friction);
    // robot->getBodyNode("rFootBack")->setFrictionCoeff(friction);
    // robot->getBodyNode("lFootFront")->setFrictionCoeff(friction);
    // robot->getBodyNode("lFootBack")->setFrictionCoeff(friction);
    // robot->getBodyNode("rFootFront2")->setFrictionCoeff(friction);
    // robot->getBodyNode("rFootBack2")->setFrictionCoeff(friction);
    // robot->getBodyNode("lFootFront2")->setFrictionCoeff(friction);
    // robot->getBodyNode("lFootBack2")->setFrictionCoeff(friction);
    // robot->getBodyNode("rFootFront")->setRestitutionCoeff(restit);
    // robot->getBodyNode("rFootBack")->setRestitutionCoeff(restit);
    // robot->getBodyNode("lFootFront")->setRestitutionCoeff(restit);
    // robot->getBodyNode("lFootBack")->setRestitutionCoeff(restit);
    // robot->getBodyNode("rFootFront2")->setRestitutionCoeff(restit);
    // robot->getBodyNode("rFootBack2")->setRestitutionCoeff(restit);
    // robot->getBodyNode("lFootFront2")->setRestitutionCoeff(restit);
    // robot->getBodyNode("lFootBack2")->setRestitutionCoeff(restit);

    robot->getBodyNode("rAnkle")->setFrictionCoeff(friction);
    robot->getBodyNode("lAnkle")->setFrictionCoeff(friction);
    robot->getBodyNode("lAnkle")->setRestitutionCoeff(restit);
    robot->getBodyNode("rAnkle")->setRestitutionCoeff(restit);

    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(servo_rate);

    // =========================================================================
    // Display Joints Frame
    // =========================================================================
    if (b_display_joint_frame) displayJointFrames(world, robot);

    // =========================================================================
    // Display Target Frame
    // =========================================================================
    if (b_display_target_frame) addTargetFrame(world);

    // =========================================================================
    // Initial configuration
    // =========================================================================
    _setInitialConfiguration(robot);

    // =========================================================================
    // Enabel Joit Limits
    // =========================================================================
    if (b_joint_limit_enforced) _setJointLimitConstraint(robot);

    // =========================================================================
    // Print Model Info
    // =========================================================================
    //_printRobotModel(robot);

    // =========================================================================
    // Wrap a worldnode
    // =========================================================================
    osg::ref_ptr<DracoWorldNode> node;
    node = new DracoWorldNode(world);
    node->setNumStepsPerCycle(30);

    // =========================================================================
    // Create and Set Viewer
    // =========================================================================
    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.simulate(false);
    viewer.switchHeadlights(false);
    ::osg::Vec3 p1(1.0, 0.2, 1.0);
    p1 = p1 * 0.7;
    viewer.getLightSource(0)->getLight()->setPosition(
        ::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
    viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
    viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    viewer.addEventHandler(new OneStepProgress(node));

    if (isRecord) {
        std::cout << "[Video Record Enable]" << std::endl;
        viewer.record(THIS_COM "/ExperimentVideo");
    }

    // viewer.setUpViewInWindow(0, 0, 2880, 1800);
    viewer.setUpViewInWindow(1440, 0, 500, 500);
    viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3(5.14, 2.28, 3.0) * 1.5, ::osg::Vec3(0.0, 0.2, 0.5),
        ::osg::Vec3(0.0, 0.0, 1.0));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();

}
