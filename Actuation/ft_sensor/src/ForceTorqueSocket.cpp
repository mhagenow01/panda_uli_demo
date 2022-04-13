#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h> 

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

using namespace std;

/* FT Sensor - Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef struct response_struct {
    uint32 rdt_sequence;
    uint32 ft_sequence;
    uint32 status;
    int32 FTData[6];
} RESPONSE;

byte request[8];            /* The request data sent to the Net F/T. */
int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */


array<double, 6> read_ft(){
    // Get the current FT reading from the sensor
    double cpf = 1000000;
    int i;						/* Generic loop/array index. */
    RESPONSE resp;				/* The structured response received from the Net F/T. */
    byte response[36];			/* The raw response data received from the Net F/T. */
    
    send(socketHandle, request, 8, 0 );

    /* Receiving the response. */
    recv( socketHandle, response, 36, 0 );
    resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
    resp.ft_sequence = ntohl(*(uint32*)&response[4]);
    resp.status = ntohl(*(uint32*)&response[8]);
    for( i = 0; i < 6; i++ ) {
        resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
    }

    array<double, 3> force_sensor = {0.0, 0.0, 0.0};
    array<double, 3> torque_sensor = {0.0, 0.0, 0.0};
    force_sensor[0]=resp.FTData[0]/cpf;
    force_sensor[1]=resp.FTData[1]/cpf;
    force_sensor[2]=resp.FTData[2]/cpf;
    torque_sensor[0]=resp.FTData[3]/cpf;
    torque_sensor[1]=resp.FTData[4]/cpf;
    torque_sensor[2]=resp.FTData[5]/cpf;

    // Eigen::VectorXd f = Eigen::Map<Eigen::VectorXd>(force_sensor.data(),3);
    // Eigen::VectorXd t = Eigen::Map<Eigen::VectorXd>(torque_sensor.data(),3);

    // // Force torque sensor axes do not align with panda base frame. Rotate into that frame.
    // Eigen::Matrix3d m;
    // m = Eigen::AngleAxisd(5*M_PI/6, Eigen::Vector3d::UnitZ());
    // f = m * f;
    // t = m * t;
    array<double, 6> ft_sensor ={force_sensor[0], force_sensor[1], force_sensor[2],
                                torque_sensor[0], torque_sensor[1], torque_sensor[2]};
    return ft_sensor;
}

void setup_ft(){
    double cpt = 1000000;
    struct sockaddr_in addr;	/* Address of Net F/T. */
    struct hostent *he;			/* Host entry for Net F/T. */
    int err;					/* Error status of operations. */

    /* Calculate number of samples, command code, and open socket here. */
    socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketHandle == -1) {
        cout << "Can't Get Socket Handle. Exiting." << endl;
        exit(1);
    }
    
    *(uint16*)&request[0] = htons(0x1234); /* standard header. */
    *(uint16*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
    *(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
    
    /* Sending the request. */
    he = gethostbyname("192.168.2.2");
    memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    
    err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
    if (err == -1) {
        cout << "Can't Connect to Socket. Exiting." << endl;
        exit(2);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "FTraw");
    ros::NodeHandle n("~");  
    ros::Publisher ft_pub = n.advertise<geometry_msgs::Wrench>("/ft/wrench_raw", 1);

    ros::Duration(0.5).sleep(); // sleep for publisher

    setup_ft();

    array<double, 6> wrench_arr;
    geometry_msgs::Wrench wrench;
    ros::Rate loop_rate(1000);
    while (ros::ok()){
        wrench_arr = read_ft();
        wrench.force.x = wrench_arr[0];
        wrench.force.y = wrench_arr[1];
        wrench.force.z = wrench_arr[2];
        wrench.torque.x = wrench_arr[3];
        wrench.torque.y = wrench_arr[4];
        wrench.torque.z = wrench_arr[5];
        ft_pub.publish(wrench);
        loop_rate.sleep();
    }

    return 0;
}
