#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class ContactSpec;
class KinWBC;
class WBLC;
class WBLC_ExtraData;

class BodyLiftCtrl: public Controller{
    public:
        BodyLiftCtrl(RobotSystem* );
        virtual ~BodyLiftCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const std::string & setting_file_name);

        void setStanceTime(double stance_time){ end_time_ = stance_time; }
        void setStanceHeight(double height) {
            des_base_height_ = height;
            b_set_height_target_ = true;
        }

    protected:
        bool b_set_height_target_;
        double des_base_height_;
        double ini_base_height_;
        double max_rf_z_;
        double min_rf_z_;
        int dim_contact_;

        std::vector<int> selected_jidx_;
        Task* body_rpz_task_;
        Task* selected_joint_task_;

        ContactSpec* rfoot_contact_;
        ContactSpec* lfoot_contact_;
        ContactSpec* double_contact_;
        KinWBC* kin_wbc_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        Eigen::VectorXd base_pos_ini_;
        Eigen::Quaternion<double> base_ori_ini_;

        Eigen::VectorXd ini_jpos_;
        Eigen::VectorXd des_jpos_;
        Eigen::VectorXd des_jvel_;
        Eigen::VectorXd des_jacc_;

        Eigen::VectorXd Kp_;
        Eigen::VectorXd Kd_;

        double end_time_;
        void _task_setup();
        void _contact_setup();
        void _compute_torque_wblc(Eigen::VectorXd & gamma);
        double ctrl_start_time_;

        DracoStateProvider* sp_;
};
