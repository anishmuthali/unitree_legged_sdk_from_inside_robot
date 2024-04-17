/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <exception>

using namespace UNITREE_LEGGED_SDK;

class InterruptException : public std::exception
{
public:
  InterruptException(int s) : S(s) {}
  int S;
};


void sig_to_exception(int s)
{
  throw InterruptException(s);
}

class Custom
{
public:
    Custom(uint8_t level, string fileName): 
      safe(LeggedType::Go1), 
      udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState)),
      saveFile(fileName),
      joystickSaveFile("joystick_" + fileName)
    {
        udp.InitCmdData(cmd);
        saveFile << "x,y,z,forward_speed,side_speed,rotate_speed,gait_type,"
        << "foot_raise_height,yaw_speed,body_height,"
        << "q_w,q_x,q_y,q_z,w_x,w_y,w_z,a_x,a_y,a_z,roll,pitch,yaw\n";
        joystickSaveFile << "lx,ly,rx,ry\n";
    }
    void setDt(float newDt) {
        dt = newDt;
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void processState(HighState state);
    void processJoystick(HighState state);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    std::ofstream saveFile;
    std::ofstream joystickSaveFile;
};

void Custom::processState(HighState state)
{
    saveFile << state.position[0] << "," << state.position[1] << "," << state.position[2] << ",";
    saveFile << state.velocity[0] << "," << state.velocity[1] << "," << state.velocity[2] << ",";
    saveFile << (int) (state.gaitType) << "," << state.footRaiseHeight << "," << state.yawSpeed << "," << state.bodyHeight << ",";
    IMU currIMU = state.imu;
    saveFile << currIMU.quaternion[0] << "," << currIMU.quaternion[1] << "," << currIMU.quaternion[2] << "," << currIMU.quaternion[3] << ",";
    saveFile << currIMU.gyroscope[0] << "," << currIMU.gyroscope[1] << "," << currIMU.gyroscope[2] << ",";
    saveFile << currIMU.accelerometer[0] << "," << currIMU.accelerometer[1] << "," << currIMU.accelerometer[2] << ",";
    saveFile << currIMU.rpy[0] << "," << currIMU.rpy[1] << "," << currIMU.rpy[2] << "\n";
}



void Custom::processJoystick(HighState state)
{
    xRockerBtnDataStruct joysticks;
    memcpy(&joysticks, state.wirelessRemote, 40);
    std::cout << "lx: " << joysticks.lx << ", ly: " << joysticks.ly << "\nrx: " << joysticks.rx << ", ry:" << joysticks.ry << "\n";
    joystickSaveFile << joysticks.lx << "," << joysticks.ly << "," << joysticks.rx << "," << joysticks.ry << "\n";
}

void Custom::UDPRecv()
{
    udp.Recv();
    HighState state;
    udp.GetRecv(state);
    processState(state);
    processJoystick(state);
    
}

void Custom::UDPSend()
{  
    udp.Send();
}


int main(int argc, char *argv[]) 
{
    float dt = 0.002;
    string outputFile = "output.csv";
    if (argc == 5) {
        if (strcmp(argv[1], "-r") == 0 && strcmp(argv[3], "-o") == 0) {
            dt = std::stof(argv[2]);
            outputFile = argv[4];
        } else if (strcmp(argv[1], "-o") == 0 && strcmp(argv[3], "-r") == 0) {
            dt = std::stof(argv[4]);
            outputFile = argv[2];
        }
    } else if (argc >= 2) {
        if (strcmp(argv[1], "-r") == 0) {
            dt = std::stof(argv[2]);
        }
        else if (strcmp(argv[1], "-o") == 0) {
            outputFile = argv[2];
        }
    }
    // ros::init(argc, argv, "node_example_walk", ros::init_options::NoSigintHandler);
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL, outputFile);
    custom.setDt(dt);
    std::cout << "dt is set to " << dt << std::endl;
    std::cout << "outputting state to " << outputFile 
        << " and outputting joystick data to " << "joystick_" + outputFile << std::endl;
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sig_to_exception;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // InitEnvironment();
    // LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    // loop_control.start();

    try {
        while(1){
            sleep(10);
        };
    } catch (InterruptException &e) {
        std::cout << "Caught keyboard interrupt, saving file" << std::endl;
        custom.saveFile.close();
        custom.joystickSaveFile.close();
        return 1;
    }
    

    return 0; 
}
