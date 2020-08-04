#include <PnC/ScorpioPnC/CtrlSet/OSCCtrl.hpp>
#include <PnC/ScorpioPnC/CtrlSet/GripperCtrl.hpp>
#include <PnC/ScorpioPnC/TestSet/GraspingTest.hpp>
#include <PnC/ScorpioPnC/ScorpioStateProvider.hpp>

GraspingTest::GraspingTest(RobotSystem* robot) : Test(robot) {
    std::cout << "==================== GRASPING TEST STARTING CONSTRUCTING ===================" << std::endl;
    myUtils::pretty_constructor(1, "OSC Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Scorpio/TEST/GRASPING_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    gripper_ctrl_ = new GripperCtrl(robot);
    std::cout << "==================== gripper control done ===================" << std::endl;
    move_ctrl_ = new OSCCtrl(robot);
    std::cout << "==================== osc control done ===================" << std::endl;

    state_list_.push_back(gripper_ctrl_);
    state_list_.push_back(move_ctrl_);

    std::cout << "==================== states done ===================" << std::endl;
    _ParameterSetting();
    std::cout << "==================== parameter setting done ===================" << std::endl;
    sp_ = ScorpioStateProvider::getStateProvider(robot_);
    std::cout << "==================== GRASPING TEST DONE CONSTRUCTING ===================" << std::endl;
}
GraspingTest::~GraspingTest() {
    delete move_ctrl_;
    delete gripper_ctrl_;
}

void GraspingTest::TestInitialization() {
    std::cout << "===============test initialization beginning==========================" << std::endl;
    move_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["osc_ctrl"]);
    std::cout << "===============move ctrl done==========================" << std::endl;
    gripper_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["grasping_ctrl"]);
    std::cout << "===============gripper ctrl done==========================" << std::endl;
    std::cout << "===============test initialization done==========================" << std::endl;
}
int GraspingTest::_NextPhase(const int& phase) {
    int nx_phase = phase + 1;
    //printf("[GRASPING TEST] next phase: %d\n", nx_phase);
    if (nx_phase == NUM_GRASPING_PH) {
        nx_phase = HOLD_PH;
    }
    sp_->phase_copy = nx_phase;
    return nx_phase;
}

void GraspingTest::_ParameterSetting() {
    try {
        // YAML::Node test_cfg = cfg_["test_configuration"];

        // Eigen::VectorXd tmp_vec;
        // double tmp_val;
        // myUtils::readParameter(test_cfg, "moving_duration", tmp_val);
        ((OSCCtrl*)move_ctrl_)->setEndTime(10.0f);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
            << __FILE__ << "]" << std::endl
            << std::endl;
        exit(0);
    }
}

void GraspingTest::SetMovingTarget(const Eigen::VectorXd& pos){
    ((OSCCtrl*)move_ctrl_)->setRelativeTargetPosition(pos);
}
