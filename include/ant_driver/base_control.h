#ifndef MSG_HANDLER_H_
#define MSG_HANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include<can2serial/can2serial.h>
#include<serial/serial.h>
#include<arpa/inet.h>

#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include<std_msgs/UInt64.h>

#include <ant_msgs/State.h>
#include <ant_msgs/State1.h>
#include <ant_msgs/State2.h>
#include <ant_msgs/State3.h>
#include <ant_msgs/State4.h>

#include <ant_msgs/ControlCmd1.h>
#include <ant_msgs/ControlCmd2.h>


#define ID_CMD_1 0x2C5
#define	ID_CMD_2 0x1C5

#define ID_STATE1 0x151
#define ID_STATE2 0x300
#define ID_STATE3 0x4D1
#define ID_STATE4 0x1D5

#ifndef MAX_SPEED
#define MAX_SPEED 60.0
#endif


#ifndef PACK
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

//for stm32
#define STM32_MAX_NOUT_SIZE 200
#define STM32_MAX_PKG_BUF_LEN 20

static unsigned char stm32_data_buf[STM32_MAX_NOUT_SIZE+1];
static unsigned char stm32_pkg_buf[STM32_MAX_PKG_BUF_LEN];
static unsigned char send_to_stm32_buf[8] = {0x66,0xCC,0x00,0x04,0x5A};

enum{Stm32MsgHeaderByte0=0x66,Stm32MsgHeaderByte1=0xCC};

PACK(
typedef struct 
{
	uint8_t header0;
	uint8_t header1;
	unsigned short pkgLen;
	uint8_t id;
	uint8_t is_start :1;
	uint8_t is_emergency_brake :1;
	uint8_t reserved;
	uint8_t checkNum;
	
}) stm32Msg1_t;
// end for stm32

class BaseControl
{
public:
	BaseControl();
	~BaseControl();
	bool init(int ,char**);
	void run();
	
	void parse_obdCanMsg();
	void read_stm32_port();

	void callBack1(const ant_msgs::ControlCmd1::ConstPtr msg);
	void callBack2(const ant_msgs::ControlCmd2::ConstPtr msg);
	void timer_callBack(const ros::TimerEvent& event);
	
private:
	void Stm32BufferIncomingData(unsigned char *message, unsigned int length);
	void parse_stm32_msgs();
	uint8_t generateCheckNum(const void* voidPtr,size_t len);
	
private:
	Can2serial can2serial;
	serial::Serial * stm32_serial_port_;
	
	const stm32Msg1_t *stm32_msg1Ptr_;
	
	bool manualCtrlDetected_; //是否检测到驾驶员介入
	bool is_driverless_mode_; //是否为自动驾驶模式(实际值)
	bool allow_driverless_;   //是否允许进入自动驾驶模式
	uint8_t stm32_brake_;
	bool default_drive_gear_; //是否默认进入自动驾驶模式后为前进档
	
	boost::shared_ptr<boost::thread> readFromStm32_thread_ptr_; 
	
	ros::Subscriber cmd1_sub;
	ros::Subscriber cmd2_sub;
	
	ros::Publisher state1_pub;
	ros::Publisher state2_pub;
	ros::Publisher state3_pub;
	ros::Publisher state4_pub;
	ros::Publisher state_pub;
	
	ros::Timer timer_;
	
	std::string obd_can_port_name_;
	
	std::string stm32_port_name_;
	int stm32_baudrate_;
	
	float max_steering_speed_;  //Front and rear frame maximun steering angle difference
	int steering_offset_; 
	
	
	CanMsg_t canMsg_cmd1;
	CanMsg_t canMsg_cmd2;
	
	ant_msgs::State1 state1;
	ant_msgs::State2 state2;
	ant_msgs::State3 state3;
	ant_msgs::State4 state4;
	
	boost::mutex mutex_;

};


#endif
