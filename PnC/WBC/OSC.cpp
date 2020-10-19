#include <PnC/WBC/OSC.hpp>

OSC::OSC(const std::vector<bool> & act_list, const Eigen::MatrixXd * Jci)
    : WBC(act_list, Jci){
}

void OSC::updateSetting(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Ainv,
                        const Eigen::VectorXd& cori,
                        const Eigen::VectorXd& grav, void* extra_setting) {
    A_ = A;
    Ainv_ = Ainv;
    cori_ = cori;
    grav_ = grav;
    b_updatesetting_ = true;
    Jci_ = *((Eigen::MatrixXd*)extra_setting);
}

void OSC::makeTorque(const std::vector<Task*> & task_list,
                     const std::vector<ContactSpec*> & contact_list,
                     Eigen::VectorXd & cmd, void* extra_input){

    Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(num_qdot_);

    // Internal Constraint Check
    Nci_ = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);
    Eigen::MatrixXd SN_c, SN_c_bar;

    if (b_internal_constraint_) {
        Eigen::MatrixXd JciBar;

        myUtils::weightedInverse(Jci_,Ainv_, JciBar);
        Nci_ -= JciBar * Jci_;
        SN_c = Sa_*Nci_;
        myUtils::weightedInverse(SN_c, Ainv_, SN_c_bar);
    }
    //Eigen::MatrixXd test = SN_c.transpose()*SN_c_bar.transpose();
    //Eigen::MatrixXd test1 = Nci_.transpose()-test;
    //myUtils::pretty_print(test, std::cout, "UNC_1");
    //myUtils::pretty_print(test1,  std::cout, "UNC_2");

    // First Task
    Task* task = task_list[0];
    Eigen::MatrixXd Jt, JtPre, JtPreBar, N_pre, N_x;
    Eigen::VectorXd JtDotQdot, xddot, qddot_pre;
    task->getTaskJacobian(Jt);
    //task->getTaskJacobianDotQdot(JtDotQdot);
    JtPre = Jt * Nci_;
    myUtils::weightedInverse(JtPre, Ainv_, JtPreBar);
    N_pre = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_)
        - JtPreBar * JtPre;
    N_pre = Nci_ * N_pre;
    task->getCommand(xddot);
    //std::cout << "+++++++++++++++++++++++++++" << std::endl;
    //myUtils::pretty_print(xddot, std::cout, "first task xddot");
    qddot_des = JtPreBar * xddot;
    //myUtils::pretty_print(qddot_des, std::cout, "first task qddot");
    //myUtils::pretty_print(JtPreBar, std::cout, "J_ori_bar");
    //Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    //JtPreBar, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //std::cout << "Jt1 singularvalues" << std::endl; 
    //std::cout << svd.singularValues() << std::endl;

    for (int i = 1; i < task_list.size(); ++i) {
        task = task_list[i];
        task->getTaskJacobian(Jt);
        JtPre = Jt * N_pre;
        myUtils::weightedInverse(JtPre, Ainv_, JtPreBar);
        N_x = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_)
            - JtPreBar * JtPre;
        N_pre *= N_x;
        task->getCommand(xddot);

        //TODO:TEST
        //if (i ==1) {

        //Eigen::MatrixXd Jt1;
        //Eigen::VectorXd r1;
        //task = task_list[0];
        //task->getTaskJacobian(Jt1);

        //r1 = Jt1 * JtPreBar * xddot;
        //std::cout << "===============TEST=============="<<std::endl;
        //myUtils::pretty_print(r1, std::cout, "first taskjacobian times second qddot");
        //} 

        //if (i ==2) {

        //std::cout << "===============TEST=============="<<std::endl;
        //myUtils::pretty_print(xddot, std::cout, "joint task xddot");

        //Eigen::MatrixXd Jt1, Jt2;
        //Eigen::VectorXd r1, r2;

        //task = task_list[0];
        //task->getTaskJacobian(Jt1);

        //r1 = Jt1 * JtPreBar * xddot;
        //std::cout << "===============TEST=============="<<std::endl;
        //myUtils::pretty_print(r1, std::cout, "first taskjacobian times last qddot");

        //task = task_list[1];
        //task->getTaskJacobian(Jt2);
        //r2 = Jt2 * JtPreBar * xddot;
        //std::cout << "===============TEST=============="<<std::endl;
        //myUtils::pretty_print(r2, std::cout, "second taskjacobian times last qddot");
        //}
        //

        //if (i==2) {
        //Eigen::JacobiSVD<Eigen::MatrixXd> svd1(
        //JtPreBar, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //std::cout << "Jt3_bar" << std::endl; 
        //std::cout << svd1.singularValues() << std::endl;

        //Eigen::MatrixXd SN_c_J_q = SN_c_bar.transpose()*JtPreBar;
        //Eigen::JacobiSVD<Eigen::MatrixXd> svd4(
        //SN_c_J_q, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //std::cout << "S_N_J_q" << std::endl; 
        //std::cout << svd4.singularValues() << std::endl;
        //std::cout << "============================" << std::endl;
        //}

        //myUtils::pretty_print(JtPreBar, std::cout, "J_pos_bar");
        //Eigen::JacobiSVD<Eigen::MatrixXd> svd1(
        //JtPreBar, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //std::cout << "Jt1_bar" << std::endl; 
        //std::cout << svd1.singularValues() << std::endl;

        qddot_des += JtPreBar * xddot;
    }

    if (false) {
        Eigen::MatrixXd SN_c_J_q = SN_c_bar.transpose()*JtPre;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd4(
        SN_c_J_q, Eigen::ComputeThinU | Eigen::ComputeThinV);
        std::cout << "S_N_J_q" << std::endl; 
        std::cout << svd4.singularValues() << std::endl;
        std::cout << "============================" << std::endl;
    }

    cmd = SN_c_bar.transpose() * (A_ * qddot_des + Nci_.transpose() * (cori_ + grav_));
    //myUtils::pretty_print(cmd, std::cout, "command");
}
