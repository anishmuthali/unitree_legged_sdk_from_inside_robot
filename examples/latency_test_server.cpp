/* amarco */

// Notice: This exemple should running on another PC, and make sure the Ethernet is stable.

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include "multi_pc_type.h"
#include <iostream>

// amarco:
#include <chrono>

using namespace UNITREE_LEGGED_SDK;

class Custom
{

public:
    Custom(): udp(8117, "localhost", 8118, sizeof(StampedSequence), sizeof(StampedSequence)), time_elapsed_vec(10000){}
    void UDPRecv();
    void UDPSend();
    void Calc();

    UDP udp;
    float dt = 0.01;

    // amarco:
    StampedSequence sseq_server;
    StampedSequence sseq_client;
    bool send_new_number = false;
    std::vector<double> time_elapsed_vec;
    long ind_global = 0;
    // auto clock = std::chrono::high_resolution_clock{};
    // using Clock = std::chrono::high_resolution_clock;
    // using TimePoint = std::chrono::time_point<Clock>;
    std::chrono::steady_clock::time_point time_start;


};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::Calc() 
{
    // udp.GetRecv((char*)&bbb);
    // printf("%f\n", bbb.yaw);

    // printf("Server\n");
    
    if(send_new_number == true && ind_global < 10000){

        // Update server data:


        // Provide the time stamp in nanoseconds already:

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        sseq_server.time_stamp = double(std::chrono::duration_cast<std::chrono::microseconds>(end - time_start).count() / 1000000.0);


        // sseq_server.time_stamp = clock.now(); // Record current timestamp
        sseq_server.sequence_nr += 1; // Increase dequence

        // Update statistical variables:
        time_elapsed_vec[ind_global] = sseq_server.time_stamp - sseq_client.time_stamp;


        std::cout << "elapsed_time: " << time_elapsed_vec[ind_global] << " seconds\n";

        // Others:
        ind_global += 1;
        send_new_number = false;

    }


    
    std::cout << "sseq_server.sequence_nr: "  << sseq_server.sequence_nr << " (before)\n";

    // Keep sending whatever is inside sseq
    udp.SetSend((char*)&sseq_server);

    std::cout << "sseq_server.sequence_nr: "  << sseq_server.sequence_nr << "(after)\n\n";




    std::cout << "sseq_client.sequence_nr: "  << sseq_client.sequence_nr << "(before)\n";


    // Keep reading whatever comes back
    udp.GetRecv((char*)&sseq_client);

    // TODO: This function sets the fileds to zero...?

    std::cout << "sseq_client.sequence_nr: "  << sseq_client.sequence_nr << "(after)\n\n";
    

    if(sseq_client.sequence_nr == sseq_server.sequence_nr){
        send_new_number = true;

        // TODO: Use "sseq_client.sequence_nr == sseq_server.sequence_nr" as the condition for the above if-else
    }
    
}

int main(void)
{
    Custom custom;
    // InitEnvironment();


    custom.time_start = std::chrono::steady_clock::now();

    custom.sseq_client.sequence_nr = -1;
    custom.sseq_server.sequence_nr = 0;

    // Send a sequence of numbers and see when the numbers are getting back by comparing timestamps
    // Inside the robot, we should have another script which only purpose is to return the sequence back in order


    // Keep sending a message on the loop until it's received back. Store the time stamp and repeat the operation with
    // another number


    // amarco: these loops do not guarantee real-time, they're just
    LoopFunc loop_calc("calc_loop",   custom.dt,    boost::bind(&Custom::Calc,    &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_calc.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
