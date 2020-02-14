#include <PnC/MPC/CMPC.hpp>
#include <Utils/Math/minjerk_one_dim.hpp>

Eigen::Vector3d des_com_xy_pos_given_feet(const Eigen::MatrixXd & r_feet){
    Eigen::Vector3d rfoot_avg = 0.5*(r_feet.col(0) + r_feet.col(1));
    Eigen::Vector3d lfoot_avg = 0.5*(r_feet.col(2) + r_feet.col(3));
    return 0.5*(rfoot_avg + lfoot_avg);
}

Eigen::VectorXd get_mpc_Xref(CMPC & convex_mpc,
                             double t_start, double mpc_dt, int mpc_horizon,
                             std::vector<MinJerk_OneDimension> & com_min_jerk_ref,
                             std::vector<MinJerk_OneDimension> & ori_min_jerk_ref){
    int n = convex_mpc.getStateVecDim(); // This is always size 13.
    Eigen::VectorXd mpc_Xref = Eigen::VectorXd::Zero(n*mpc_horizon); // Create the desired state vector evolution


    double t_predict = t_start;
    double com_x, com_y, com_z, rx, ry, rz;
    double dcom_x, dcom_y, dcom_z, drx, dry, drz;

    for(int i = 0; i < mpc_horizon; i++){
        // Time 
        t_predict = t_start + (i+1)*mpc_dt;

        // Get CoM value
        com_min_jerk_ref[0].getPos(t_predict, com_x); com_min_jerk_ref[0].getVel(t_predict, dcom_x);
        com_min_jerk_ref[1].getPos(t_predict, com_y); com_min_jerk_ref[1].getVel(t_predict, dcom_y);
        com_min_jerk_ref[2].getPos(t_predict, com_z); com_min_jerk_ref[2].getVel(t_predict, dcom_z);

        // Get Ori value
        ori_min_jerk_ref[0].getPos(t_predict, rx); ori_min_jerk_ref[0].getVel(t_predict, drx);
        ori_min_jerk_ref[1].getPos(t_predict, ry); ori_min_jerk_ref[1].getVel(t_predict, dry);
        ori_min_jerk_ref[2].getPos(t_predict, rz); ori_min_jerk_ref[2].getVel(t_predict, drz);

        // Desired RPY
        mpc_Xref[i*n + 0] = rx; // Desired Roll
        mpc_Xref[i*n + 1] = ry; // Desired Pitch
        mpc_Xref[i*n + 2] = rz; // Desired Yaw

        // Desired COM x,y,z
        mpc_Xref[i*n + 3] = com_x; // Desired com x
        mpc_Xref[i*n + 4] = com_y; // Desired com y
        mpc_Xref[i*n + 5] = com_z; // Desired com z

        // Desired wx, wy, wz
        mpc_Xref[i*n + 6] = drx; // Desired Roll
        mpc_Xref[i*n + 7] = dry; // Desired Pitch
        mpc_Xref[i*n + 8] = drz; // Desired Yaw

        // Desired COM vel x,y,z
        mpc_Xref[i*n + 9] = dcom_x; // Desired com x vel
        mpc_Xref[i*n + 10] = dcom_y; // Desired com y vel
        mpc_Xref[i*n + 11] = dcom_z; // Desired com z vel
    }

    return mpc_Xref;
}

int main(int argc, char ** argv){
    CMPC convex_mpc;
    // convex_mpc.simulate_toy_mpc();

    // System Params
    convex_mpc.setRobotMass(50);  // (kilograms)
    Eigen::MatrixXd I_robot_body =
        1.0 * Eigen::MatrixXd::Identity(3, 3);  // Body Inertia matrix
    // I_robot_body(0,0) = 07;
    // I_robot_body(1,1) = 0.26;
    // I_robot_body(2,2) = 0.242;
    convex_mpc.setRobotInertia(I_robot_body);

    // MPC Params
    int mpc_horizon = 15;
    double mpc_dt = 0.025;
    convex_mpc.setHorizon(mpc_horizon);  // horizon timesteps
    convex_mpc.setDt(mpc_dt);    // (seconds) per horizon
    convex_mpc.setMu(0.9);      //  friction coefficient
    convex_mpc.setMaxFz(500);   // (Newtons) maximum vertical reaction force per foot.

    // mpc smoothing options
    convex_mpc.setSmoothFromPrevResult(true);
    convex_mpc.setDeltaSmooth(1e-7);

    // Eigen::VectorXd mpc_cost_vec = Eigen::VectorXd::Zero(13);
    // mpc_cost_vec << 2.5, 2.5, 2.5, 1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0;        
    // convex_mpc.setCostVec(mpc_cost_vec);

    // Set custom gait cycle
    double swing_time = 0.2;
    double transition_time = 0.2;
    double biped_walking_offset = swing_time + transition_time;
    double total_gait_duration = 2.0*swing_time + 2.0*transition_time;
    std::shared_ptr<GaitCycle> no_gait(new GaitCycle());
    std::shared_ptr<GaitCycle> gait_steps(new GaitCycle(swing_time, total_gait_duration, {0.0, 0.0, biped_walking_offset, biped_walking_offset}));
    convex_mpc.setCustomGaitCycle(gait_steps);
    convex_mpc.setPreviewStartTime(0.0);

    // Feet Params
    double foot_length = 0.05;   // 5cm distance between toe and heel
    double nominal_width = 0.1;  // 10cm distance between left and right feet

    // Initial Feet Configuration
    // Set Foot contact locations w.r.t world
    Eigen::MatrixXd r_feet(3, 4);  // Each column is a reaction force in x,y,z
    r_feet.setZero();

    // Flat ground contact, z = 0.0
    // 4 Contact Configuration
    // Right Foot Front
    r_feet(0, 0) = foot_length / 2.0;     // x
    r_feet(1, 0) = -nominal_width / 2.0;  // y
    // Right Foot Back
    r_feet(0, 1) = -foot_length / 2.0;    // x
    r_feet(1, 1) = -nominal_width / 2.0;  // y
    // Left Foot Front
    r_feet(0, 2) = foot_length / 2.0;    // x
    r_feet(1, 2) = nominal_width / 2.0;  // y
    // Left Foot Back
    r_feet(0, 3) = -foot_length / 2.0;   // x
    r_feet(1, 3) = nominal_width / 2.0;  // y

    // Foot landing configuration
    Eigen::MatrixXd r_feet_land = r_feet;
    Eigen::Vector3d rfoot_translate(0.125, 0.0, 0.0);
    Eigen::Vector3d lfoot_translate(0.125, 0.0, 0.0);

    std::cout << "r_feet start location:" << std::endl;
    std::cout << r_feet << std::endl;

    r_feet_land.col(0) += rfoot_translate; r_feet_land.col(1) += rfoot_translate;
    r_feet_land.col(2) += lfoot_translate; r_feet_land.col(3) += lfoot_translate;

    std::cout << "r_feet landing location:" << std::endl;
    std::cout << r_feet_land << std::endl;

    // Initialize MPC input for the r_feet 
    Eigen::MatrixXd r_feet_input = r_feet;

    int n_Fr = r_feet.cols();  // Number of contacts
    int n = 13;
    int m = 3 * n_Fr;

    // Initialize force containers
    Eigen::VectorXd f_vec_out(m * mpc_horizon);
    Eigen::MatrixXd f_Mat(3, n_Fr);
    f_Mat.setZero();

    // Starting robot state ---------------------------------------------------------------------------------------------------------------
    // Current reduced state of the robot
    // x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
    Eigen::VectorXd x0(13);

    double init_roll(0), init_pitch(0), init_yaw(0.0), init_com_x(0),
           init_com_y(0), init_com_z(0.75), init_roll_rate(0), init_pitch_rate(0),
           init_yaw_rate(0), init_com_x_rate(0), init_com_y_rate(0),
           init_com_z_rate(0);

    x0 = convex_mpc.getx0(init_roll, init_pitch, init_yaw, init_com_x, init_com_y,
                          init_com_z, init_roll_rate, init_pitch_rate, init_yaw_rate,
                          init_com_x_rate, init_com_y_rate, init_com_z_rate);

    // Set initial and terminal com pos and orientation
    Eigen::Vector3d ini_com_pos(init_com_x, init_com_y, init_com_z);
    Eigen::Vector3d ini_ori_pos(init_roll, init_pitch, init_yaw);
    Eigen::Vector3d des_end_com_pos = des_com_xy_pos_given_feet(r_feet_land);
    des_end_com_pos += Eigen::Vector3d(0.0, 0.0, init_com_z);
    Eigen::Vector3d des_end_ori(0.0, 0.0, 0.0); // roll, pitch, yaw

    // Initialize reference minimum jerk trajectory
    std::vector<MinJerk_OneDimension> com_min_jerk_ref;
    std::vector<MinJerk_OneDimension> ori_min_jerk_ref;

    double time_start = 0.0;
    for(int i = 0; i < 3; i++){
        com_min_jerk_ref.push_back(MinJerk_OneDimension(Eigen::Vector3d(ini_com_pos[i], 0.0, 0.0), 
                                                        Eigen::Vector3d(des_end_com_pos[i], 0.0, 0.0), 
                                                        time_start, 
                                                        total_gait_duration) );        
        ori_min_jerk_ref.push_back(MinJerk_OneDimension(Eigen::Vector3d(ini_ori_pos[i], 0.0, 0.0), 
                                                        Eigen::Vector3d(des_end_ori[i], 0.0, 0.0), 
                                                        time_start, 
                                                        total_gait_duration) ); 
    }

    // -----------------------------------------------------
    // std::cout << "Min jerk reference" << std::endl;
    // // Test minimum jerk trajectory
    // double test_time = 0.0;
    // double test_time_dur = 1.0;
    // double test_dt = 0.01;
    // int n_test = static_cast<int>(test_time_dur/test_dt);
    // printf("t, r, p, y, x, y, z, rx, ry, rz, dx, dy, dz\n");

    // double com_x, com_y, com_z, rx, ry, rz;
    // double dcom_x, dcom_y, dcom_z, drx, dry, drz;
    // for(int i = 0; i < n_test; i++){
    //     // Get CoM value
    //     com_min_jerk_ref[0].getPos(test_time, com_x); com_min_jerk_ref[0].getVel(test_time, dcom_x);
    //     com_min_jerk_ref[1].getPos(test_time, com_y); com_min_jerk_ref[1].getVel(test_time, dcom_y);
    //     com_min_jerk_ref[2].getPos(test_time, com_z); com_min_jerk_ref[2].getVel(test_time, dcom_z);

    //     // Get Ori value
    //     ori_min_jerk_ref[0].getPos(test_time, rx); ori_min_jerk_ref[0].getVel(test_time, drx);
    //     ori_min_jerk_ref[1].getPos(test_time, ry); ori_min_jerk_ref[1].getVel(test_time, dry);
    //     ori_min_jerk_ref[2].getPos(test_time, ry); ori_min_jerk_ref[2].getVel(test_time, drz);

    //     printf("%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\n",
    //            test_time, rx, ry, rz, com_x, com_y, com_z, rx, ry, rz, dcom_x, dcom_y, dcom_z);
    //     // Increment test time
    //     test_time += test_dt;
    // }

    Eigen::VectorXd x_pred(n);  // Container to hold the predicted state after 1 horizon timestep
    Eigen::VectorXd X_ref(n * mpc_horizon);
    // Update reference trajectory
    X_ref = get_mpc_Xref(convex_mpc, time_start, mpc_dt, mpc_horizon, com_min_jerk_ref, ori_min_jerk_ref);

    // std::cout << "X reference" << std::endl;
    // // Print out the predicted reference trajectory
    // for(int i = 0; i < mpc_horizon; i++){
    //     x_pred = X_ref.segment(i*n, n);
    //     printf("%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\n",
    //             x_pred[0],x_pred[1],x_pred[2],x_pred[3],x_pred[4],x_pred[5],x_pred[6],x_pred[7],x_pred[8],x_pred[9],x_pred[10],x_pred[11]);
    // }
 


    // Get constant desired reference
    // Eigen::VectorXd x_des(n);
    // x_des.setZero();
    // x_des[0] = 0.0;        // M_PI/8; //des roll orientation
    // x_des[1] = 0.0;        //-M_PI/8; //des pitch orientation
    // x_des[2] = M_PI / 12;  // Yaw orientation
    // x_des[3] = 0.0;  //-0.1;//;0.75; // Set desired z height to be 0.75m from
    //                  //the ground
    // x_des[5] = 0.75;  //;0.75; // Set desired z height to be 0.75m from the ground

    // Eigen::VectorXd X_des(n * mpc_horizon);
    // convex_mpc.get_constant_desired_x(x_des, X_des);

    // -----------------------------------------
    // Solve the MPC
    convex_mpc.solve_mpc(x0, X_ref, r_feet, x_pred, f_vec_out);
    convex_mpc.print_f_vec(n_Fr, f_vec_out);
    // Populate force output from 1 horizon.
    convex_mpc.assemble_vec_to_matrix(3, n_Fr, f_vec_out.head(3 * n_Fr), f_Mat);

    // Simulate MPC for one time step
    double sim_dt = 1e-3;
    Eigen::VectorXd x_prev(n);
    x_prev = x0;
    Eigen::VectorXd x_next(n); x_next.setZero();
    convex_mpc.integrate_robot_dynamics(sim_dt, x_prev, f_Mat, r_feet, x_next);

    // Simulate MPC for x seconds
    double total_sim_time = 7.0;
    int sim_steps = static_cast<int>(total_sim_time / sim_dt);
    std::cout << "sim_steps:" << sim_steps << std::endl;

    double last_control_time = 0.0;
    double cur_time = 0.0;

    std::cout << "x_start:" << x0.transpose() << std::endl;
    Eigen::VectorXd f_cmd(12);
    f_cmd.head(m) = f_vec_out.head(m);

    bool set_once = false;
    for (int i = 0; i < sim_steps; i++) {

        // Do the feet update if we are stepping
        if (cur_time < total_gait_duration){
            // update r_feet if we are in the flight phase
            gait_steps->updateContactStates(0.0, cur_time);
            // Check if right foot is in flight
            // std::cout << " right foot " << i << " : " << (gait_steps->getContactState(1) ? std::string("ACTIVE") : std::string("FLIGHT")) << std::endl;
            if (gait_steps->getContactState(1) == 0){
                // If so, update the location of the feet
                r_feet_input.col(0) = r_feet_land.col(0);
                r_feet_input.col(1) = r_feet_land.col(1);
            }

            // Check if left foot is in flight
            // std::cout << " left foot " << i << " : " << (gait_steps->getContactState(2) ? std::string("ACTIVE") : std::string("FLIGHT"))  << std::endl;
            if (gait_steps->getContactState(2) == 0){
                // If so, update the location of the feet
                r_feet_input.col(2) = r_feet_land.col(2);
                r_feet_input.col(3) = r_feet_land.col(3);
            }
        }else{
            if (!set_once){
                convex_mpc.setCustomGaitCycle(no_gait);
                set_once;
            }
        }


        // Solve new MPC every mpc control tick
        if ((cur_time - last_control_time) > mpc_dt) {
            convex_mpc.setPreviewStartTime(cur_time);
            // compute the new reference
            X_ref = get_mpc_Xref(convex_mpc, cur_time, mpc_dt, mpc_horizon, com_min_jerk_ref, ori_min_jerk_ref);
            convex_mpc.solve_mpc(x_prev, X_ref, r_feet_input, x_pred, f_vec_out);
            convex_mpc.assemble_vec_to_matrix(3, n_Fr, f_vec_out.head(3 * n_Fr), f_Mat);
            last_control_time = cur_time;
            f_cmd.head(m) = f_vec_out.head(m);
        }

        // Integrate the robot dynamics
        convex_mpc.integrate_robot_dynamics(sim_dt, x_prev, f_Mat, r_feet_input, x_next);
        // Update x
        x_prev = x_next;
        // Increment time
        cur_time += sim_dt;

        printf(
            "%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, "
            "%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f,%0.3f,%0.3f, "
            "%0.3f,%0.3f,%0.3f, %0.3f,%0.3f,%0.3f, %0.3f,%0.3f,%0.3f \n",
            cur_time, x_next[0], x_next[1], x_next[2], x_next[3], x_next[4],
            x_next[5], x_next[6], x_next[7], x_next[8], x_next[9],
            x_next[10], x_next[11], f_cmd[0], f_cmd[1], f_cmd[2], f_cmd[3],
            f_cmd[4], f_cmd[5], f_cmd[6], f_cmd[7], f_cmd[8], f_cmd[9],
            f_cmd[10], f_cmd[11]);
    }





}