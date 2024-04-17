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
 * @file leptrino_node.cpp
 * @brief ROS2 Publisher node for Leptrino 6 axis force/torque sensor
 *
 * @author Ricardo Rosales Martinez
 */

#include <cstdio>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iomanip> // Include the header for std::setprecision

#include "leptrino_node.hpp"


LeptrinoForceSensor::LeptrinoForceSensor(): Node("leptrino_wrench")
{
    
    declareParameters();
   // Initialize the sensor
    if (!initializeSensor()) {
        RCLCPP_ERROR(this->get_logger(), "Sensor initialization failed. Shutting down node.");
        // rclcpp::shutdown();
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Sensor Initialized");
    if (!this->get_parameter("rate",rate)) {
    RCLCPP_WARN(this->get_logger(), "Rate  is not defined, using default value ");
    rate= 100;
    } 
    RCLCPP_INFO(this->get_logger(), "Publishing rate %d Hz",rate );

    int ms_time= 1000/rate; // time in ms for desired frequency

    publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("sensor_wrench", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(ms_time), std::bind(&LeptrinoForceSensor::publishMessage, this));
}

void LeptrinoForceSensor :: declareParameters() {
     // Declare communication parameters
        this->declare_parameter("com_port","/dev/ttyACM0");
        this->declare_parameter("rate",100);
}

void LeptrinoForceSensor :: publishMessage() {
    
    auto message = std::make_unique<geometry_msgs::msg::WrenchStamped>(); 
    int rt=0;
    Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//��M�f�[�^�L
    
    memset(CommRcvBuff,0,sizeof(CommRcvBuff)); 
    rt = Comm_GetRcvData( CommRcvBuff );
    if (rt > 0) {
      stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
      message->header.stamp = this->now();
      message->header.frame_id = "leptrino_link"; // Change this to your desired frame_id
      message->wrench.force.x = std::round(stForce->ssForce[0] * conversion_factor[0]*10000)/10000; 
      message->wrench.force.y = std::round(stForce->ssForce[1] * conversion_factor[1]*10000)/10000; 
      message->wrench.force.z = std::round(stForce->ssForce[2] * conversion_factor[2]*10000)/10000;
      message->wrench.torque.x = std::round(stForce->ssForce[3] * conversion_factor[3]*10000)/10000;
      message->wrench.torque.y = std::round(stForce->ssForce[4] * conversion_factor[4]*10000)/10000;
      message->wrench.torque.z = std::round(stForce->ssForce[5] * conversion_factor[5]*10000)/10000;
      publisher_->publish(std::move(message));
    }
  
    }

}


void LeptrinoForceSensor :: StopSensor(){
  SerialStop();
}


bool LeptrinoForceSensor::initializeSensor(){

  int rt=0;
  int endFlag=0;

  //Start the communication
  
  if (!this->get_parameter("com_port", com_port))
  {
    RCLCPP_WARN(this->get_logger(), "Port is not defined, trying /dev/ttyACM0");
    com_port = "/dev/ttyACM0";
  }

  App_Init();
	if (gSys.com_ok == NG) {
		printf("Communication port fail to open \n");
		return (false);
	}

  GetProductInfo();
  while(!endFlag) { 
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {	
			CommRcvBuff[0]=0; 
			
			rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
				stGetInfo->scFVer[F_VER_SIZE] = 0;
        RCLCPP_INFO(this->get_logger(), "Version:%s\n", stGetInfo->scFVer);
				stGetInfo->scSerial[SERIAL_SIZE] = 0;
        RCLCPP_INFO(this->get_logger(), "SerialNo:%s\n", stGetInfo->scSerial);
				stGetInfo->scPName[P_NAME_SIZE] = 0;
        RCLCPP_INFO(this->get_logger(), "Type:%s\n", stGetInfo->scPName);
				endFlag = 1;
			}
			
		}
	}	
  endFlag=0;
  GetLimit();
  while(rclcpp::ok()) {
      Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { //受信データ有
      CommRcvBuff[0] = 0;

      rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff;
        for (int i = 0; i < FN_Num; i++)
        {
          RCLCPP_INFO(this->get_logger(), "Limit[%d]: %f \n", i, stGetLimit->fLimit[i]);
          conversion_factor[i] = stGetLimit->fLimit[i] * 1e-4;
        }
        endFlag = 1;
      }
    }
    if ( endFlag==1 ) break;
  }
  SerialStart();// Get Data from the sensor
  return (true);
}

void LeptrinoForceSensor :: App_Init(void)
{
  int rt=0;

  //Commポート初期化
  gSys.com_ok = NG;
  rt = Comm_Open(com_port.c_str());
  if (rt == OK)
  {
    Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
    gSys.com_ok = OK;
  }

}

void LeptrinoForceSensor :: App_Close(void)
{
  RCLCPP_DEBUG(this->get_logger(), "Application close\n");
  if (gSys.com_ok == OK)
  {
    Comm_Close();
  }
}

ULONG LeptrinoForceSensor :: SendData(UCHAR *pucInput, USHORT usSize)
{
  USHORT usCnt;
  UCHAR ucWork;
  UCHAR ucBCC = 0;
  UCHAR *pucWrite = &CommSendBuff[0];
  USHORT usRealSize;

  // データ整形
  *pucWrite = CHR_DLE; // DLE
  pucWrite++;
  *pucWrite = CHR_STX; // STX
  pucWrite++;
  usRealSize = 2;

  for (usCnt = 0; usCnt < usSize; usCnt++)
  {
    ucWork = pucInput[usCnt];
    if (ucWork == CHR_DLE)
    { // データが0x10ならば0x10を付加
      *pucWrite = CHR_DLE; // DLE付加
      pucWrite++; // 書き込み先
      usRealSize++; // 実サイズ
      // BCCは計算しない!
    }
    *pucWrite = ucWork; // データ
    ucBCC ^= ucWork; // BCC
    pucWrite++; // 書き込み先
    usRealSize++; // 実サイズ
  }

  *pucWrite = CHR_DLE; // DLE
  pucWrite++;
  *pucWrite = CHR_ETX; // ETX
  ucBCC ^= CHR_ETX; // BCC計算
  pucWrite++;
  *pucWrite = ucBCC; // BCC付加
  usRealSize += 3;

  Comm_SendData(&CommSendBuff[0], usRealSize);

  return OK;
}

void LeptrinoForceSensor :: GetProductInfo(void)
{
  USHORT len;

  RCLCPP_INFO(this->get_logger(), "Get sensor information");
  // printf("Get sensor information\n");
  len = 0x04; // データ長
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_GET_INF; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}

void LeptrinoForceSensor :: GetLimit(void)
{
  USHORT len;
  RCLCPP_INFO(this->get_logger(), "Get sensor limit");
  len = 0x04;
  SendBuff[0] = len; // レングス length
  SendBuff[1] = 0xFF; // センサNo. Sensor no.
  SendBuff[2] = CMD_GET_LIMIT; // コマンド種別 Command type
  SendBuff[3] = 0; // 予備 reserve

  SendData(SendBuff, len);
}

void LeptrinoForceSensor :: SerialStart(void)
{
  USHORT len;

  RCLCPP_INFO(this->get_logger(), "Start sensor");
  // printf("Start\n");
  len = 0x04; // データ長
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_DATA_START; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}

void LeptrinoForceSensor :: SerialStop(void)
{
  USHORT len;

  RCLCPP_INFO(this->get_logger(), "Stop sensor\n");
  len = 0x04; // データ長
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_DATA_STOP; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<LeptrinoForceSensor> node= std::make_shared<LeptrinoForceSensor>();
  rclcpp::spin(node);
  node->StopSensor();
  rclcpp::shutdown();

  return 0;
}


