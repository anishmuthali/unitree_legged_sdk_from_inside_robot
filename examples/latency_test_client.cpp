/* amarco */

// Notice: This exemple should running on another PC, and make sure the Ethernet is stable.

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include "multi_pc_type.h"

// amarco:
#include <chrono>

using namespace UNITREE_LEGGED_SDK;

class Custom
{

public:
    Custom(): udp(8118, "localhost", 8117, sizeof(StampedSequence), sizeof(StampedSequence)){}
    void UDPRecv();
    void UDPSend();
    void Calc();

    UDP udp;
    float dt = 0.01;

    // amarco:
    StampedSequence sseq_server;
    StampedSequence sseq_client;
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

    // printf("Client\n");

    // Read whatever is being received from the server:
    udp.GetRecv((char*)&sseq_server);

    // Copy the data and send it back:
    sseq_client.sequence_nr = sseq_server.sequence_nr;
    sseq_client.time_stamp = sseq_server.time_stamp;
    udp.SetSend((char*)&sseq_client);

}

int main(void)
{
    Custom custom;
    // InitEnvironment();

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
