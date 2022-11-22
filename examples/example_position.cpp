/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

// amarco:
#include <fstream>
#include <chrono>

using namespace std;
using namespace UNITREE_LEGGED_SDK;


class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Go1), udp(level) {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};  
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    // float dt = 0.002;     // 0.001~0.01
    float dt = 0.01;     // amarco

    // amarco: data to write:
    std::array<std::string, 13> data_joint_names = {"time_stamp","FR_0","FR_1","FR_2","FL_0","FL_1","FL_2","RR_0","RR_1","RR_2","RL_0","RL_1","RL_2"};
    std::array< std::array<std::array<float, 1000>, 13> , 5 > data_fields;
    std::array<std::string, 5> name_data_fields = {"q_des","q_curr","dq_curr","u_des","u_est"};
    // data_q_des: Desired position for all the joints, [1000,12]
    // data_q_curr: Actual position for all the joints, [1000,12]
    // data_dq_curr: Actual velocity for all the joints, [1000,12]
    // data_u_des: Desired torque for all the joints, [1000,12]
    // data_u_est: Estimated output torque for all the joints, [1000,12]

    float time_elapsed;
    std::chrono::high_resolution_clock::time_point time_start;

    int ind_data = 0;

};

void Custom::UDPRecv()
{  
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void Custom::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);
    // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);
    // printf("%d  %f\n", motiontime, state.imu.quaternion[2]);

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    if(motiontime == 1)
        time_start = std::chrono::high_resolution_clock::now();

    // if( motiontime >= 100){
    if( motiontime >= 0){
        // std::cout << "here [custom] 1: " << "\n";
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if( motiontime >= 0 && motiontime < 10){
            qInit[0] = state.motorState[FR_0].q;
            qInit[1] = state.motorState[FR_1].q;
            qInit[2] = state.motorState[FR_2].q;

            // std::cout << "here [custom] 2: " << "\n";
        }
        // second, move to the origin point of a sine movement with Kp Kd
        // if( motiontime >= 500 && motiontime < 1500){
        if( motiontime >= 10 && motiontime < 400){
            rate_count++;
            double rate = rate_count/200.0;                       // needs count to 200
            Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0; 
            Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
            
            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);

            // std::cout << "here [custom] 3: " << "\n";
        }
        double sin_joint1, sin_joint2;
        // last, do sine wave
        if( motiontime >= 400){
            sin_count++;
            sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/1000.0);
            qDes[0] = sin_mid_q[0];
            qDes[1] = sin_mid_q[1];
            qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];

            // std::cout << "here [custom] 4: " << "\n";
        }

        cmd.motorCmd[FR_0].q = qDes[0];
        cmd.motorCmd[FR_0].dq = 0;
        cmd.motorCmd[FR_0].Kp = Kp[0];
        cmd.motorCmd[FR_0].Kd = Kd[0];
        cmd.motorCmd[FR_0].tau = -0.65f;

        cmd.motorCmd[FR_1].q = qDes[1];
        cmd.motorCmd[FR_1].dq = 0;
        cmd.motorCmd[FR_1].Kp = Kp[1];
        cmd.motorCmd[FR_1].Kd = Kd[1];
        cmd.motorCmd[FR_1].tau = 0.0f;

        cmd.motorCmd[FR_2].q =  qDes[2];
        cmd.motorCmd[FR_2].dq = 0;
        cmd.motorCmd[FR_2].Kp = Kp[2];
        cmd.motorCmd[FR_2].Kd = Kd[2];
        cmd.motorCmd[FR_2].tau = 0.0f;

    }

    if(motiontime > 10){
        safe.PositionLimit(cmd);
        int res1 = safe.PowerProtect(cmd, state, 1);
        // You can uncomment it for position protection
        // int res2 = safe.PositionProtect(cmd, state, 0.087);
        if(res1 < 0) exit(-1);

        // std::cout << "here [custom] 5: " << "\n";
    }

    udp.SetSend(cmd);

    if(ind_data < 1000){

        // std::cout << "ind_data: " << std::to_string(ind_data) << "\n";

        std::chrono::duration<float, std::nano> time_dur = std::chrono::high_resolution_clock::now() - time_start;
        time_elapsed = time_dur.count() / 1000000000.0; // NOTE: time_dur.count() returns a double

        for (std::size_t jj = 0 ; jj < data_fields[0].size() ; jj++){

            // std::cout << "jj: " << std::to_string(jj) << "\n";

            // {"time_stamp",q_des","q_curr","dq_curr","u_des","u_est"}
            if (jj == 0){
                data_fields[0][jj][ind_data] = time_elapsed;
                data_fields[1][jj][ind_data] = time_elapsed;
                data_fields[2][jj][ind_data] = time_elapsed;
                data_fields[3][jj][ind_data] = time_elapsed;
                data_fields[4][jj][ind_data] = time_elapsed;
            }
            else{
                data_fields[0][jj][ind_data] = cmd.motorCmd[jj].q;
                data_fields[1][jj][ind_data] = state.motorState[jj].q;
                data_fields[2][jj][ind_data] = state.motorState[jj].dq;
                data_fields[3][jj][ind_data] = cmd.motorCmd[jj].tau;
                data_fields[4][jj][ind_data] = state.motorState[jj].tauEst;

            }
            // data_q_des[ind_data][jj] = cmd.motorCmd[jj].q;
            // data_q_curr[ind_data][jj] = state.motorState[jj].q;
            
            // data_dq_curr[ind_data][jj] = state.motorState[jj].dq;
            
            // data_u_des[ind_data][jj] = cmd.motorCmd[jj].tau;
            // data_u_est[ind_data][jj] = state.motorState[jj].tauEst;
        }
    }


    ind_data++;

    // Record here the sent data and the received data with a timestamp
    // Fill in a vector and write it into a file
    // std::vector files


    // consider protobuf: https://developers.google.com/protocol-buffers/docs/reference/overview
    // what about ROS? I can leverage rviz and so on...

}



class Write2File
{
public:
    Write2File(std::string filepath, std::string filename_base): filepath_(filepath), filename_base_(filename_base){}
    void dump(Custom &custom);

    std::string filepath_;
    std::string filename_base_;
    std::array<std::string, 5> file_path_named;
    std::array<std::ofstream, 5> files_vec;

};

void Write2File::dump(Custom &custom){

    // std::array<float, 30> q_des;
    // std::array<float, 30> q_pos;
    // std::array<std::string, 2> names;

    
    
    // for (std::size_t jj = 0 ; jj < custom.q_des.size() ; jj++)
    //     custom.q_des[jj] = -7.957393;

    // for (std::size_t jj = 0 ; jj < custom.q_pos.size() ; jj++)
    //     custom.q_pos[jj] = +5.0947392;

    // custom.names[0] = "q_des";
    // custom.names[1] = "q_pos";


    // char buffer[8096]; // larger = faster (within limits)
    ;
    // file.rdbuf()->pubsetbuf(buffer, sizeof(buffer));

    // file.open("/home/ubuntu/mounted_home/work/code_projects_WIP/unitree_legged_sdk_from_inside_robot/examples/data_robot.csv", std::ofstream::out | std::ofstream::app);
    // file.open("/home/ubuntu/mounted_home/work/code_projects_WIP/unitree_legged_sdk_from_inside_robot/examples/data_robot.csv", std::ofstream::out | std::ofstream::trunc);

     // std::array<std::string> var_names = {"q_des","q_curr","dq_curr","u_des","u_est"};

    // Append names:
    // std::cout << "here1" << "\n";


    for (std::size_t ff = 0 ; ff < custom.name_data_fields.size() ; ff++){
        file_path_named[ff] = filepath_ + filename_base_ + "_" + custom.name_data_fields[ff] + ".csv";
        std::cout << "Dumping data to file: " << file_path_named[ff] << "!!!\n";
        files_vec[ff].open(file_path_named[ff], std::ofstream::out | std::ofstream::trunc);
    }

    // std::cout << "here2\n";


    for (std::size_t ff = 0 ; ff < custom.name_data_fields.size() ; ff++) {
    
        // names:
        for (std::size_t jj = 0 ; jj < custom.data_joint_names.size() ; jj++) {
            if (jj < custom.data_joint_names.size()-1)
                files_vec[ff] << custom.data_joint_names[jj] << ",";
            else
                files_vec[ff] << custom.data_joint_names[jj];
        }
        files_vec[ff] << ";" << "\n";

        // std::cout << "here2.1\n";

        // values:
        size_t Nrows = custom.data_fields[0][0].size();
        size_t Ncols = custom.data_fields[0].size();
        // std::cout << "here2.2\n";
        // std::cout << "Nrows: " << std::to_string(Nrows) << "\n";
        // std::cout << "Ncols: " << std::to_string(Ncols) << "\n";
        for (std::size_t ii = 0 ; ii < Nrows ; ii++) {

            // std::cout << "here2.3\n";

            for (std::size_t jj = 0 ; jj < Ncols ; jj++) {

                // std::cout << "here2.4\n";
                
                if (jj < Ncols-1)
                    files_vec[ff] << custom.data_fields[ff][jj][ii] << ',';
                else
                    files_vec[ff] << custom.data_fields[ff][jj][ii] << ';' << "\n";
            
            }
        }

        files_vec[ff].close();

        std::cout << "Done!!! " << "\n";

    }

    // std::cout << "here3\n";
}



int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();


    // Log data out:
    std::string filepath("../examples");
    std::string filename_base("/data_robot");
    Write2File write2file(filepath,filename_base);


    Custom custom(LOWLEVEL);
    
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    // while(1){
    //     sleep(1);
    // };


    float time_sleep = 11.0;
    std::cout << "Sleeping for " << std::to_string(int(time_sleep)) << " seconds ...\n";
    sleep(time_sleep);
    // std::cout << "Here finally!!! " <<  "!!!\n";

    write2file.dump(custom);


    loop_udpSend.shutdown();
    loop_udpRecv.shutdown();
    loop_control.shutdown();

    sleep(0.1);


    return 0; 
}
