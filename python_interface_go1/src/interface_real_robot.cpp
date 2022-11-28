/**********************************************************************************************
 * Robot Interface inspired in
     * Unitree's example_position.cpp
     * Google/UC Berkeley repo https://github.com/erwincoumans/motion_imitation
 * We provide a set of functions to interface with the real robot using Python
**********************************************************************************************/

#include <interface_real_robot.hpp>

void RobotInterfaceGo1::InitializeAllFieldsToZero(){

    // LowState:
    state.levelFlag = 0;
    state.commVersion = 0;
    state.robotID = 0;
    state.SN = 0; 
    state.bandWidth = 0;

    state.imu.quaternion.fill(0.0);
    state.imu.gyroscope.fill(0.0);
    state.imu.accelerometer.fill(0.0);
    state.imu.rpy.fill(0.0);
    state.imu.temperature = 0;

    for (int ii = 0; ii < state.motorState.size(); ii++) {

        state.motorState[ii].mode = 0;
        state.motorState[ii].q = 0.0;
        state.motorState[ii].dq = 0.0;
        state.motorState[ii].ddq = 0.0;
        state.motorState[ii].tauEst = 0.0;
        state.motorState[ii].q_raw = 0.0;
        state.motorState[ii].dq_raw = 0.0;
        state.motorState[ii].ddq_raw = 0.0;
        state.motorState[ii].temperature = 0;
        state.motorState[ii].reserve.fill(0);
    }

    state.footForce.fill(0);
    state.footForceEst.fill(0);

    state.bms.version_h = 0;
    state.bms.version_l = 0;
    state.bms.bms_status = 0;
    state.bms.SOC = 0;
    state.bms.current = 0;
    state.bms.cycle = 0;

    state.tick = 0;

    state.wirelessRemote.fill(0);        // wireless commands
    state.reserve = 0;
    state.crc = 0;


    // LowCmd
    cmd.levelFlag = 0;
    cmd.commVersion = 0;
    cmd.robotID = 0;
    cmd.SN = 0;
    cmd.bandWidth = 0;
    cmd.bms.off = 0;
    // cmd.bms.reserve;

    // MotorCmd motorCmd[20];
    for (int ii = 0; ii < cmd.motorCmd.size(); ii++) {

        cmd.motorCmd[ii].mode = 0;
        cmd.motorCmd[ii].q = 0.0;
        cmd.motorCmd[ii].dq = 0.0;
        cmd.motorCmd[ii].tau = 0.0;
        cmd.motorCmd[ii].Kp = 0.0;
        cmd.motorCmd[ii].Kd = 0.0;
        cmd.motorCmd[ii].reserve.fill(0);
    }

}

void RobotInterfaceGo1::CollectObservations() {
    udp.Recv();
    udp.GetRecv(state);
    return;
}

void RobotInterfaceGo1::update_all_observations() {

    this->update_joint_pos_curr();
    this->update_joint_vel_curr();
    this->update_body_linear_velocity();
    this->update_body_angular_velocity();
    this->update_body_orientation();

    return;
}

void RobotInterfaceGo1::update_joint_pos_curr(){

    for (int jj = 0; jj < Njoints; jj++){ // amarco: std::array<MotorState, 20> motorState; we only need the first 12 elements, corresponding to the joints
        joint_pos_curr[jj] = state.motorState[jj].q;
    }


    return;
}

void RobotInterfaceGo1::update_joint_vel_curr(){

    for (int jj = 0; jj < Njoints; jj++)
        joint_vel_curr[jj] = state.motorState[jj].dq;

    return;
}


void RobotInterfaceGo1::update_body_linear_velocity(){

    // Numerical integration:
    for (int jj = 0; jj < body_linear_velocity.size(); jj++)
        body_linear_velocity[jj] += deltaT*state.imu.accelerometer[jj];

    return;
}

void RobotInterfaceGo1::update_body_angular_velocity(){

    // Numerical integration:
    for (int jj = 0; jj < body_angular_velocity.size(); jj++)
        body_angular_velocity[jj] = state.imu.gyroscope[jj];

    return;
}


void RobotInterfaceGo1::update_body_orientation(){

    // Numerical integration:
    for (int jj = 0; jj < body_orientation.size(); jj++)
        body_orientation[jj] = state.imu.rpy[jj];

    return;
}

void RobotInterfaceGo1::get_joint_pos_curr(Eigen::Ref<Vector12d> joint_pos_curr){
    joint_pos_curr = joint_pos_curr;

    // joint_pos_curr[0] = 100.0; // dbg, this works

    return;
}

void RobotInterfaceGo1::ensure_safety(){

    // Saturate positions:
    safe.PositionLimit(cmd);

    int trigger_pow = safe.PowerProtect(cmd,state,1);
    if (trigger_pow < 0){
        std::cout << "Power protection triggered exit (!!)\n";
        exit(-1);
    }

    if (this->use_position_protect == true){
        int trigger_pos = safe.PositionProtect(cmd,state,0.087);
        if (trigger_pos < 0){
            std::cout << "Position protection triggered exit (!!)\n";
            exit(-1);
        }
    }

    return;
}

void RobotInterfaceGo1::SendCommand(const Eigen::Ref<Vector12d>& joint_pos_des,
                                    const Eigen::Ref<Vector12d>& joint_vel_des,
                                    const Eigen::Ref<Vector12d>& joint_torque_des){

    cmd.levelFlag = 0xff; // amarco: What is this?
    for (int ii = 0; ii < this->Njoints; ii++) { // amarco: std::array<MotorCmd, 20> motorCmd; we only need the first 12 dimensions, corresponding to the joints
        cmd.motorCmd[ii].mode = 0x0A; // amarco: What is this?
        cmd.motorCmd[ii].q = joint_pos_des[ii];
        cmd.motorCmd[ii].dq = joint_vel_des[ii];
        cmd.motorCmd[ii].tau = joint_torque_des[ii];
        cmd.motorCmd[ii].Kp = P_gains_[ii];
        cmd.motorCmd[ii].Kd = D_gains_[ii];
    }


    this->ensure_safety(); // Will modify the contents of cmd to ensure that joint/torque limits are not exceeded

    udp.SetSend(cmd);
    udp.Send();

    return;
}


void RobotInterfaceGo1::send_desired_position(const Eigen::Ref<Vector12d>& joint_pos_des){
    /*

    Send only desired position, while setting the desired velocity to zero and torque to appropriate values

    */

    // TODO: Fix this: differentiate the desired position to get the velocity?
    Vector12d joint_vel_des; // How do we get this? What do we set it to?
    joint_vel_des.setZero(); // By setting to zero, we basically bypass the influence of the D gain
    
    // Gravity compensation (as in example_position.cpp):
    Vector12d joint_torque_des; // How do we get this? What do we set it to?
    joint_torque_des.setZero();
    joint_torque_des[FR_0] = -0.65f;
    joint_torque_des[FL_0] = +0.65f;
    joint_torque_des[RR_0] = -0.65f;
    joint_torque_des[RL_0] = +0.65f;

    this->SendCommand(joint_pos_des,joint_vel_des,joint_torque_des);

    return;
}



// void RobotInterfaceGo1::set_PD_gains(const std::array<float, 12> & P_gains, const std::array<float, 12> & D_gains){
void RobotInterfaceGo1::set_PD_gains(const Eigen::Ref<Vector12d>& P_gains, const Eigen::Ref<Vector12d>& D_gains){

    // std::copy(std::begin(P_gains), std::end(P_gains), std::begin(this->P_gains_));
    // std::copy(std::begin(D_gains), std::end(D_gains), std::begin(this->D_gains_));

    this->P_gains_ = P_gains;
    this->D_gains_ = D_gains;

    std::cout << "Setting PD gains for position control:\n";

    std::cout << "P_gains: " << this->P_gains_.transpose().format(this->clean_format) << "\n";
    std::cout << "D_gains: " << this->D_gains_.transpose().format(this->clean_format) << "\n";

    return;
}


void RobotInterfaceGo1::set_deltaT(double deltaT){
    this->deltaT = deltaT;
    std::cout << "Setting deltaT: " << deltaT << "\n";
    std::cout << "deltaT will only we used for state estimation using IMUs\n";
    return;
}

double RobotInterfaceGo1::get_deltaT(void){
    return this->deltaT;
}




/* --------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------ Gym Environment ---------------------------------------------------- */
/* --------------------------------------------------------------------------------------------------------------- */


void GymEnvironmentRealGo1::step(const Eigen::Ref<Vector12d>& action){


    // This updates joint_pos_des_
    Vector12d joint_pos_des;
    this->action2joint_pos_des(action,joint_pos_des); // We have 12 actuators; action is 12-dimensional and it's transformed here into desired positions

    this->send_desired_position(joint_pos_des);

    // Update:
    this->update_env_observation();

    return;
}

void GymEnvironmentRealGo1::set_action_std(double action_std){

    this->actionStd_.setConstant(action_std);

    return;
}

void GymEnvironmentRealGo1::set_action_mean(const Eigen::Ref<Vector12d>& action_mean){

    this->actionMean_ = action_mean;

    return;
}

void GymEnvironmentRealGo1::action2joint_pos_des(const Eigen::Ref<Vector12d>& action, Eigen::Ref<Vector12d> joint_pos_des){

    joint_pos_des = action;

    // action scaling
    joint_pos_des.cwiseProduct(actionStd_); // amarco: element-wise product; https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
    joint_pos_des += actionMean_;

    return;

}


void GymEnvironmentRealGo1::update_env_observation(){

    this->CollectObservations();
    this->update_all_observations();

    this->observation_env_ << 3.0, // body height // TODO: change this using IK ...
                        body_orientation, // body orientation [3]
                        joint_pos_curr, // euler angle（unit: rad) [12]
                        body_linear_velocity, // m/s [3]
                        body_angular_velocity, // rad/s (numerically-integrated raw data, will drift over time) [3]
                        joint_vel_curr; // rad/s [12]

    return;
}


Eigen::VectorXd GymEnvironmentRealGo1::get_env_observation(){
    return observation_env_;
}

void GymEnvironmentRealGo1::init() { 


    // Have here a selector of different initialization behaviors:
    // int Nsteps = 200;
    // this->stand_up(Nsteps);

    return; 
}


/* -------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------ Python Wrapper ---------------------------------------------------- */
/* -------------------------------------------------------------------------------------------------------------- */



namespace py = pybind11;

PYBIND11_MODULE(robot_interface_go1, m) {
    m.doc() = R"pbdoc(
          Go1 Robot Interface Python Bindings
          -----------------------
          .. currentmodule:: robot_interface_go1
          .. autosummary::
             :toctree: _generate
      )pbdoc";

    py::class_<Cartesian>(m, "Cartesian")
        .def(py::init<>())
        .def_readwrite("x", &Cartesian::x)
        .def_readwrite("y", &Cartesian::y)
        .def_readwrite("z", &Cartesian::z);

    py::class_<IMU>(m, "IMU")
        .def(py::init<>())
        .def_readwrite("quaternion", &IMU::quaternion)
        .def_readwrite("gyroscope", &IMU::gyroscope)
        .def_readwrite("accelerometer", &IMU::accelerometer)
        .def_readwrite("rpy", &IMU::rpy)
        .def_readwrite("temperature", &IMU::temperature);

    py::class_<LED>(m, "LED")
        .def(py::init<>())
        .def_readwrite("r", &LED::r)
        .def_readwrite("g", &LED::g)
        .def_readwrite("b", &LED::b);

    py::class_<MotorState>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("mode", &MotorState::mode)
        .def_readwrite("q", &MotorState::q)
        .def_readwrite("dq", &MotorState::dq)
        .def_readwrite("ddq", &MotorState::ddq)
        .def_readwrite("tauEst", &MotorState::tauEst)
        .def_readwrite("q_raw", &MotorState::q_raw)
        .def_readwrite("dq_raw", &MotorState::dq_raw)
        .def_readwrite("ddq_raw", &MotorState::ddq_raw)
        .def_readwrite("temperature", &MotorState::temperature)
        .def_readwrite("reserve", &MotorState::reserve);

    py::class_<MotorCmd>(m, "MotorCmd")
        .def(py::init<>())
        .def_readwrite("mode", &MotorCmd::mode)
        .def_readwrite("q", &MotorCmd::q)
        .def_readwrite("dq", &MotorCmd::dq)
        .def_readwrite("tau", &MotorCmd::tau)
        .def_readwrite("Kp", &MotorCmd::Kp)
        .def_readwrite("Kd", &MotorCmd::Kd)
        .def_readwrite("reserve", &MotorCmd::reserve);

    py::class_<LowState>(m, "LowState")
        .def(py::init<>())
        .def_readwrite("levelFlag", &LowState::levelFlag)
        .def_readwrite("commVersion", &LowState::commVersion)
        .def_readwrite("robotID", &LowState::robotID)
        .def_readwrite("SN", &LowState::SN)
        .def_readwrite("bandWidth", &LowState::bandWidth)
        .def_readwrite("imu", &LowState::imu)
        .def_readwrite("motorState", &LowState::motorState)
        .def_readwrite("footForce", &LowState::footForce)
        .def_readwrite("footForceEst", &LowState::footForceEst)
        .def_readwrite("tick", &LowState::tick)
        .def_readwrite("wirelessRemote", &LowState::wirelessRemote)
        .def_readwrite("reserve", &LowState::reserve)
        .def_readwrite("crc", &LowState::crc);

    py::class_<LowCmd>(m, "LowCmd")
        .def(py::init<>())
        .def_readwrite("levelFlag", &LowCmd::levelFlag)
        .def_readwrite("commVersion", &LowCmd::commVersion)
        .def_readwrite("robotID", &LowCmd::robotID)
        .def_readwrite("SN", &LowCmd::SN)
        .def_readwrite("bandWidth", &LowCmd::bandWidth)
        .def_readwrite("motorCmd", &LowCmd::motorCmd)
        // .def_readwrite("led", &LowCmd::led)
        .def_readwrite("wirelessRemote", &LowCmd::wirelessRemote)
        .def_readwrite("reserve", &LowCmd::reserve)
        .def_readwrite("crc", &LowCmd::crc);

    py::class_<HighState>(m, "HighState")
        .def(py::init<>())
        .def_readwrite("levelFlag", &HighState::levelFlag)
        .def_readwrite("commVersion", &HighState::commVersion)
        .def_readwrite("robotID", &HighState::robotID)
        .def_readwrite("SN", &HighState::SN)
        .def_readwrite("bandWidth", &HighState::bandWidth)
        .def_readwrite("mode", &HighState::mode)
        .def_readwrite("imu", &HighState::imu)
        .def_readwrite("position", &HighState::position)
        .def_readwrite("velocity", &HighState::velocity)
        // .def_readwrite("forwardSpeed", &HighState::forwardSpeed)
        // .def_readwrite("sideSpeed", &HighState::sideSpeed)
        // .def_readwrite("rotateSpeed", &HighState::rotateSpeed)
        .def_readwrite("bodyHeight", &HighState::bodyHeight)
        // .def_readwrite("updownSpeed", &HighState::updownSpeed)
        // .def_readwrite("forwardPosition", &HighState::forwardPosition)
        // .def_readwrite("sidePosition", &HighState::sidePosition)
        .def_readwrite("footPosition2Body", &HighState::footPosition2Body)
        .def_readwrite("footSpeed2Body", &HighState::footSpeed2Body)
        .def_readwrite("footForce", &HighState::footForce)
        .def_readwrite("footForceEst", &HighState::footForceEst)
        // .def_readwrite("tick", &HighState::tick)
        .def_readwrite("wirelessRemote", &HighState::wirelessRemote)
        .def_readwrite("reserve", &HighState::reserve)
        .def_readwrite("crc", &HighState::crc);

    py::class_<HighCmd>(m, "HighCmd")
        .def(py::init<>())
        .def_readwrite("levelFlag", &HighCmd::levelFlag)
        .def_readwrite("commVersion", &HighCmd::commVersion)
        .def_readwrite("robotID", &HighCmd::robotID)
        .def_readwrite("SN", &HighCmd::SN)
        .def_readwrite("bandWidth", &HighCmd::bandWidth)
        .def_readwrite("mode", &HighCmd::mode)
        .def_readwrite("position", &HighCmd::postion)
        .def_readwrite("velocity", &HighCmd::velocity)
        // .def_readwrite("forwardSpeed", &HighCmd::forwardSpeed)
        // .def_readwrite("sideSpeed", &HighCmd::sideSpeed)
        // .def_readwrite("rotateSpeed", &HighCmd::rotateSpeed)
        .def_readwrite("bodyHeight", &HighCmd::bodyHeight)
        .def_readwrite("footRaiseHeight", &HighCmd::footRaiseHeight)
        // .def_readwrite("yaw", &HighCmd::yaw)
        // .def_readwrite("pitch", &HighCmd::pitch)
        // .def_readwrite("roll", &HighCmd::roll)
        .def_readwrite("euler", &HighCmd::euler)
        .def_readwrite("yawSpeed", &HighCmd::yawSpeed)
        .def_readwrite("led", &HighCmd::led)
        .def_readwrite("wirelessRemote", &HighCmd::wirelessRemote)
        // .def_readwrite("AppRemote", &HighCmd::AppRemote)
        .def_readwrite("reserve", &HighCmd::reserve)
        .def_readwrite("crc", &HighCmd::crc);

    py::class_<UDPState>(m, "UDPState")
        .def(py::init<>())
        .def_readwrite("TotalCount", &UDPState::TotalCount)
        .def_readwrite("SendCount", &UDPState::SendCount)
        .def_readwrite("RecvCount", &UDPState::RecvCount)
        .def_readwrite("SendError", &UDPState::SendError)
        .def_readwrite("FlagError", &UDPState::FlagError)
        .def_readwrite("RecvCRCError", &UDPState::RecvCRCError)
        .def_readwrite("RecvLoseError", &UDPState::RecvLoseError);

    py::class_<RobotInterfaceGo1>(m, "RobotInterfaceGo1")
        // .def(py::init<const std::array<float, 12>, const std::array<float, 12>>())
        .def(py::init<>())
        .def("collect_observations", &RobotInterfaceGo1::CollectObservations)
        .def("update_all_observations", &RobotInterfaceGo1::update_all_observations)
        .def("set_deltaT", &RobotInterfaceGo1::set_deltaT)
        .def("get_deltaT", &RobotInterfaceGo1::get_deltaT)
        .def("send_command", &RobotInterfaceGo1::SendCommand)
        .def("send_desired_position", &RobotInterfaceGo1::send_desired_position)
        .def("set_PD_gains", &RobotInterfaceGo1::set_PD_gains)
        .def("get_joint_pos_curr", &RobotInterfaceGo1::get_joint_pos_curr)
        .def("update_joint_pos_curr", &RobotInterfaceGo1::update_joint_pos_curr)
        .def("update_joint_vel_curr", &RobotInterfaceGo1::update_joint_vel_curr)
        .def("update_body_linear_velocity", &RobotInterfaceGo1::update_body_linear_velocity)
        .def("update_body_angular_velocity", &RobotInterfaceGo1::update_body_angular_velocity)
        .def("update_body_orientation", &RobotInterfaceGo1::update_body_orientation);
        // .def("main", &RobotInterfaceGo1::main)
        // .def("stand_up", &RobotInterfaceGo1::stand_up);

    py::class_<GymEnvironmentRealGo1>(m, "GymEnvironmentRealGo1")
        .def(py::init<>())
        .def("init", &GymEnvironmentRealGo1::init)
        .def("update_env_observation", &GymEnvironmentRealGo1::update_env_observation)
        .def("step", &GymEnvironmentRealGo1::step)
        .def("set_PD_gains", &GymEnvironmentRealGo1::set_PD_gains)
        .def("set_deltaT", &GymEnvironmentRealGo1::set_deltaT)
        .def("get_deltaT", &GymEnvironmentRealGo1::get_deltaT)
        // .def("stand_up", &GymEnvironmentRealGo1::stand_up)
        .def("set_action_std", &GymEnvironmentRealGo1::set_action_std)
        .def("set_action_mean", &GymEnvironmentRealGo1::set_action_mean)
        .def("get_joint_pos_curr", &GymEnvironmentRealGo1::get_joint_pos_curr)
        .def_readonly("low_state", &GymEnvironmentRealGo1::state) // DBG
        // .def_read("low_state", &GymEnvironmentRealGo1::state) // DBG
        .def("get_env_observation", &GymEnvironmentRealGo1::get_env_observation);

    #ifdef VERSION_INFO
      m.attr("__version__") = VERSION_INFO;
    #else
      m.attr("__version__") = "dev";
    #endif

      m.attr("TEST") = py::int_(int(42));

}
