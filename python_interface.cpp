/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <array>
#include <math.h>
// #include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

class RobotInterfaceGo1
{
public:
    RobotInterfaceGo1() : safe(LeggedType::Go1), udp(LOWLEVEL){
        // InitEnvironment();

        Initialize(); // amarco

    }
    // LowState ReceiveObservation();
    void CollectObservations();
    void SendCommand(std::array<float, 60> motorcmd);
    void Initialize();

    std::array<float, 12> get_joint_pos_curr();
    std::array<float, 12> get_joint_vel_curr();

    UDP udp;
    Safety safe;
    // LowState state = {0}; // amarco: commented out
    // LowCmd cmd = {0}; // amarco: commented out
    LowState state;
    LowCmd cmd;

    // amarco:
    std::array<float, 12> joint_pos_curr;
    std::array<float, 12> joint_vel_curr;

};

void RobotInterfaceGo1::Initialize(){

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

std::array<float, 12> RobotInterfaceGo1::get_joint_pos_curr(){

    for (int jj = 0; jj < 12; jj++)
        joint_pos_curr[jj] = state.motorState[jj].q;

    return joint_pos_curr;
}

std::array<float, 12> RobotInterfaceGo1::get_joint_vel_curr(){

    for (int jj = 0; jj < 12; jj++)
        joint_vel_curr[jj] = state.motorState[jj].dq;

    return joint_vel_curr;
}

void RobotInterfaceGo1::SendCommand(std::array<float, 60> motorcmd) {
    cmd.levelFlag = 0xff;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        cmd.motorCmd[motor_id].mode = 0x0A;
        cmd.motorCmd[motor_id].q = motorcmd[motor_id * 5];
        cmd.motorCmd[motor_id].Kp = motorcmd[motor_id * 5 + 1];
        cmd.motorCmd[motor_id].dq = motorcmd[motor_id * 5 + 2];
        cmd.motorCmd[motor_id].Kd = motorcmd[motor_id * 5 + 3];
        cmd.motorCmd[motor_id].tau = motorcmd[motor_id * 5 + 4];
    }
    safe.PositionLimit(cmd);
    udp.SetSend(cmd);
    udp.Send();
}

namespace py = pybind11; // amarco: commented out, moved up

PYBIND11_MODULE(robot_interface_go1, m) {
    m.doc() = R"pbdoc(
          Go1 Robot Interface Python Bindings
          -----------------------
          .. currentmodule:: go1_robot_interface
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
        .def(py::init<>())
        .def("collect_observations", &RobotInterfaceGo1::CollectObservations)
        .def("send_command", &RobotInterfaceGo1::SendCommand)
        .def("get_joint_pos_curr", &RobotInterfaceGo1::get_joint_pos_curr)
        .def("get_joint_vel_curr", &RobotInterfaceGo1::get_joint_vel_curr);

    #ifdef VERSION_INFO
      m.attr("__version__") = VERSION_INFO;
    #else
      m.attr("__version__") = "dev";
    #endif

      m.attr("TEST") = py::int_(int(42));

}
