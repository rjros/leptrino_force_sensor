/*
 * Copyright 2024, Ricardo Rosales Martinez
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file leptrino_node.hpp
 * @brief Header file for the publisher node of the Leptrino 6-axis force/torque sensor
 *
 * @author Ricardo Rosales Martinez
 */


//Leptrino official libraries 
#include <pCommon.h>
#include <rs_comm.h>
#include <pComResInternal.h>

// ROS2 libraries 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class LeptrinoForceSensor : public rclcpp::Node {
    public:
        // Declare parameters
        LeptrinoForceSensor();
        void StopSensor();


    private:
        void DeclareParameters();
        bool InitializeSensor();
        void App_Init(void);
        void App_Close(void);
        ULONG SendData(UCHAR *pucInput, USHORT usSize);
        void GetProductInfo(void);
        void GetLimit(void);
        void GetWrench(void);
        void SerialStart(void);
        void SerialStop(void);

        void publishMessage();
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        //Parameters
        typedef struct ST_SystemInfo {
            int com_ok;
        } SystemInfo;

        SystemInfo gSys;
        UCHAR CommRcvBuff[256];
        UCHAR CommSendBuff[1024];
        UCHAR SendBuff[512];

        UCHAR strprm[256];
        ST_RES_HEAD *stCmdHead;
        ST_R_DATA_GET_F *stForce;
        ST_R_GET_INF *stGetInfo;
        ST_R_LEP_GET_LIMIT* stGetLimit;
        double conversion_factor[FN_Num];

        
        std::string com_port;
        int rate;
        bool initRead_{true};
        std::vector <double> offset_=std::vector<double>(6,0);
        std::vector <double> wrenchRef_=std::vector<double>(6,0);

};