/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <array>
#include <math.h>
// #include <pybind11/operators.h>

// #include <Eigen/Core>


#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
// #include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

class RobotInterfaceGo1
{
public:
    RobotInterfaceGo1() : safe(LeggedType::Go1), udp(LOWLEVEL){
        // InitEnvironment();

        InitializeAllFieldsToZero(); // amarco

        // TODO: Maybe read this from a Yaml file here directly
        // But because this class will be always handled from Python, we can do the yaml reading on the python side, easier
        // set_PD_gains(P_gains,D_gains);


        // TODO:
        // Increase efficiency by creating a type: https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
        // using typedef Matrix<double, 12, 1> Matrix12d (recommended by Eigen only for sizes smaller than roughly 32; larger sue dynamic allocation)

        this->clean_format = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");

        // amarco: Can this be avoided?
        this->joint_pos_curr.setZero();
        this->joint_vel_curr.setZero();
        this->P_gains_.setZero();
        this->D_gains_.setZero();
        this->body_linear_velocity.setZero();
        this->body_angular_velocity.setZero();
        this->body_orientation.setZero();
        this->joint_pos_des_hold.setZero();

        joint_pos_init_read.setZero();
        joint_pos_init_target.setZero();


        joint_pos_init_target << 0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944;

        std::cout << "Safety stuff\n";
        std::cout << "safe.WattLimit: " << safe.WattLimit << "\n";
        std::cout << "safe.Wcount: " << safe.Wcount << "\n";
        std::cout << "safe.Hip_max: " << safe.Hip_max << "\n";
        std::cout << "safe.Hip_min: " << safe.Hip_min << "\n";
        std::cout << "safe.Thigh_max: " << safe.Thigh_max << "\n";
        std::cout << "safe.Thigh_min: " << safe.Thigh_min << "\n";
        std::cout << "safe.Calf_max: " << safe.Calf_max << "\n";
        std::cout << "safe.Calf_min;: " << safe.Calf_min << "\n";

        std::cout << "deltaT: " << deltaT << "\n";
        std::cout << "Njoints: " << Njoints << "\n";

    }

    size_t Njoints = 12;
    typedef Eigen::Matrix<double, 12, 1> Vector12d; // Column vector by default

    void CollectObservations();
    void update_all_observations();
    void SendCommand(   const Eigen::Ref<Vector12d>& joint_pos_des,
                        const Eigen::Ref<Vector12d>& joint_vel_des,
                        const Eigen::Ref<Vector12d>& joint_torque_des);

    void send_desired_position(const Eigen::Ref<Vector12d>& joint_pos_des);
    
    // void set_PD_gains(const std::array<float, 12> & P_gains, const std::array<float, 12> & D_gains);
    void set_PD_gains(const Eigen::Ref<Vector12d> & P_gains, const Eigen::Ref<Vector12d> & D_gains);

    void update_joint_pos_curr();
    void update_joint_vel_curr();
    void update_body_linear_velocity();
    void update_body_angular_velocity();
    void update_body_orientation();

    void get_joint_pos_curr(Eigen::Ref<Vector12d> joint_pos_curr);

    void set_deltaT(double deltaT);
    double get_deltaT(void);

    void update_joint_pos_des_hold();

    // void ControlLoop();
    // void main();
    // void go2target_linear_interpolation( const Eigen::Ref<Vector12d>& joint_pos_init, 
    //                                     const Eigen::Ref<Vector12d>& joint_pos_final,
    //                                     Eigen::Ref<Vector12d> joint_pos_interp,
    //                                     double rate);
    // void stand_up(int Nsteps);


    // Communication:
    UDP udp;
    Safety safe;
    LowState state;
    LowCmd cmd;

    // amarco:
    Vector12d joint_pos_curr;
    Vector12d joint_vel_curr;

    Vector12d P_gains_;
    Vector12d D_gains_;

    // IMU estimation:
    Eigen::Vector3d body_linear_velocity;
    Eigen::Vector3d body_angular_velocity;
    Eigen::Vector3d body_orientation;
    double deltaT = 0.002;


    // Desired position to hold:
    Vector12d joint_pos_des_hold;


    Vector12d joint_pos_init_read;
    Vector12d joint_pos_init_target;

    // Flags:
    bool is_running_control_loop = false;
    bool use_position_protect = false;

    

protected:
    void InitializeAllFieldsToZero();
    void ensure_safety();
    Eigen::IOFormat clean_format;

};


class GymEnvironmentRealGo1 : public RobotInterfaceGo1 {

public:

    GymEnvironmentRealGo1(): RobotInterfaceGo1(){

        double action_std = 0.01;
        actionStd_.setConstant(action_std);
        actionMean_.setZero();
        observation_env_.setZero(Nobs);

        // joint_pos_init_target << 0.0, -1.5245, 0.3435, 1.0, 0.0, 0.0, 1.0, 0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944;
        // joint_pos_init_target << 0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944;

    }

    void init();
    void update_env_observation();
    void step(const Eigen::Ref<Vector12d> & action);
    void set_action_std(double action_std);
    void set_action_mean(const Eigen::Ref<Vector12d>& action_mean);

    // typedef Eigen::Matrix<double, 12, 1> Vector12d; // Column vector by default
    Eigen::VectorXd get_env_observation();

    // void isTerminalState(); // Not sure we need this...
    // void curriculumUpdate(); // Not sure we need this...
    // void reset(); // We can't really 'reset' the real robot to the initial position
    // void observe(); // Not sure we need this...

private:
    
    void action2joint_pos_des(const Eigen::Ref<Vector12d>& action, Eigen::Ref<Vector12d> joint_pos_des);

    Vector12d actionMean_, actionStd_;

    size_t Nobs = 34;
    Eigen::VectorXd observation_env_;

    // Vector12d joint_pos_init_read;
    // Vector12d joint_pos_init_target;

};
