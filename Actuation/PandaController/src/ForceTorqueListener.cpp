
#include "PandaController.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <franka/robot_state.h>
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


#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

using namespace std;
namespace PandaController {
    namespace {
        boost::mutex mutex;
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

        array<double, 6> ft_sensor;
        array<double, 6> ft_bias;
        
        byte request[8];            /* The request data sent to the Net F/T. */
        int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
    }

    // These are returned in the global frame to avoid issues with different
    // end effectors
    array<double, 6> readFTForces() {
        Eigen::Quaterniond orientation = getFTOrientation();

        // forces and torques are in the local frame according to the FT
        // transform -> turn into global
        boost::lock_guard<boost::mutex> guard(mutex);
        Eigen::Vector3d forces_local;
        forces_local << ft_sensor[0],ft_sensor[1],ft_sensor[2];
        Eigen::Vector3d torques_local;
        torques_local << ft_sensor[3],ft_sensor[4],ft_sensor[5];
        Eigen::Vector3d forces_global = orientation*forces_local;
        Eigen::Vector3d torques_global = orientation*torques_local;

         

        // i.e., bias is in the global frame (e.g., weight)
        array<double,6> biased_FT;
        biased_FT[0] = forces_global[0]-ft_bias[0];
        biased_FT[1] = forces_global[1]-ft_bias[1];
        biased_FT[2] = forces_global[2]-ft_bias[2];
        biased_FT[3] = torques_global[0]-ft_bias[3];
        biased_FT[4] = torques_global[1]-ft_bias[4];
        biased_FT[5] = torques_global[2]-ft_bias[5];

        return biased_FT;
    }
    void writeFTForces(array<double, 6> data){
        boost::lock_guard<boost::mutex> guard(mutex);
        ft_sensor = data;
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

    void bias_ft(){
        // Read once
        double cpf = 1000000;
        int i;						/* Generic loop/array index. */
        RESPONSE resp;				/* The structured response received from the Net F/T. */
        byte response[36];			/* The raw response data received from the Net F/T. */
        
        // Transform into the correct frame based on Panda Pose
        franka::RobotState state = PandaController::readRobotState();

        // Get feedback from FT sensor - THIS NEEDS TO BE MOVED TO SHARED MEMORY
        send(socketHandle, request, 8, 0 );

        /* Receiving the response. */
        recv( socketHandle, response, 36, 0 );
        resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
        resp.ft_sequence = ntohl(*(uint32*)&response[4]);
        resp.status = ntohl(*(uint32*)&response[8]);
        for( i = 0; i < 6; i++ ) {
            resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
        }

        array<double, 6> ft_sensor = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        ft_sensor[0]=resp.FTData[0]/cpf;
        ft_sensor[1]=resp.FTData[1]/cpf;
        ft_sensor[2]=resp.FTData[2]/cpf;
        ft_sensor[3]=resp.FTData[3]/cpf;
        ft_sensor[4]=resp.FTData[4]/cpf;
        ft_sensor[5]=resp.FTData[5]/cpf;
        ft_bias = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        cout << "Force Torque Sensor Bias: " << endl <<
            ft_sensor[0] << endl <<
            ft_sensor[1] << endl <<
            ft_sensor[2] << endl <<
            ft_sensor[3] << endl <<
            ft_sensor[4] << endl <<
            ft_sensor[5] << endl;
    }

    array<double, 6> read_ft(){
        // Get the current FT reading from the sensor
        double cpf = 1000000;
        int i;						/* Generic loop/array index. */
        RESPONSE resp;				/* The structured response received from the Net F/T. */
        byte response[36];			/* The raw response data received from the Net F/T. */
        
        // Transform into the correct frame based on Panda Pose
        franka::RobotState state = PandaController::readRobotState();

        // Get feedback from FT sensor - THIS NEEDS TO BE MOVED TO SHARED MEMORY
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

        Eigen::VectorXd f = Eigen::Map<Eigen::VectorXd>(force_sensor.data(),3);
        Eigen::VectorXd t = Eigen::Map<Eigen::VectorXd>(torque_sensor.data(),3);

        // Force torque sensor axes do not align with panda base frame. Rotate into that frame.
        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(5*M_PI/6, Eigen::Vector3d::UnitZ());
        f = m * f;
        t = m * t;
        array<double, 6> ft_sensor ={f[0], -f[1], -f[2], t[0], -t[1], -t[2]};
        return ft_sensor;
    }

    void forceTorqueListener() {
        setup_ft();
        bias_ft();
        while(PandaController::isRunning()) {
            auto ft_sensor = read_ft();
            writeFTForces(ft_sensor);
        }
    }

}