#include<ant_driver/base_control.h>
#include<assert.h>

static float g_steering_gearRatio = 540.0/25.0;

static bool openSerial(serial::Serial* & port_ptr, std::string port_name,int baud_rate)
{
	try
	{
		port_ptr = new serial::Serial(port_name,baud_rate,serial::Timeout::simpleTimeout(10)); 

		if (!port_ptr->isOpen())
		{
			std::stringstream output;
			output << "Serial port: " << port_name << " failed to open." << std::endl;
			delete port_ptr;
			port_ptr = NULL;
			return false;
		}
		else 
		{
	        std::stringstream output;
	        output << "Serial port: " << port_name << " opened successfully." << std::endl;
	        std::cout << output.str() <<std::endl;
		}

		port_ptr->flush();
	} 
	catch (std::exception &e) 
	{
	    std::stringstream output;
	    output << "Error  " << port_name << ": " << e.what();
	    std::cout << output.str() <<std::endl;
	    return false;
	}
	
	return true;
}

BaseControl::BaseControl():
	allow_driverless_(false),
	canMsg_cmd1_valid_(false),
	canMsg_cmd2_valid_(false),
	stm32_brake_(0)
{
	is_driverless_mode_ = false;
	stm32_serial_port_ = NULL;
	
	canMsg_cmd1.ID = ID_CMD_1;
    canMsg_cmd1.len = 8;
    canMsg_cmd1.type = Can2serial::STD_DATA_FRAME; //standard frame;
    memset(canMsg_cmd1.data, canMsg_cmd1.len, 0);

    canMsg_cmd2.ID = ID_CMD_2;
    canMsg_cmd2.len = 8;
    canMsg_cmd2.type = Can2serial::STD_DATA_FRAME;//standard frame;
	memset(canMsg_cmd2.data, canMsg_cmd1.len, 0);
    
    *(long *)canMsg_cmd2.data = 0;
    canMsg_cmd2.data[4] = 0xFF;
    canMsg_cmd2.data[5] = 0xFF; //set the steer angle value invalid
}

BaseControl::~BaseControl()
{
	stm32_serial_port_->close();
	if(stm32_serial_port_!=NULL)
	{
		delete stm32_serial_port_;
		stm32_serial_port_ = NULL;
	}
}

bool BaseControl::init(int argc,char**argv)
{
	ros::init(argc,argv,"base_control");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<std::string>("obd_can_port_name", obd_can_port_name_, "");
	nh_private.param<std::string>("stm32_port_name", stm32_port_name_, "");
	nh_private.param<float>("max_steering_speed",max_steering_speed_,2.0); //通过限制前后帧转角命令差值 控制转向最大速度**
	nh_private.param<int>("steering_offset",steering_offset_,0);
	nh_private.param<bool>("default_drive_gear", default_drive_gear_, true);//是否默认为前进档
	
	nh_private.param<int>("stm32_baudrate",stm32_baudrate_,115200);
	
	assert(!obd_can_port_name_.empty() && !stm32_port_name_.empty());
	assert(max_steering_speed_>0);

#if _USE_ANT_MESSAGES	
	subscribers_.push_back(nh.subscribe("/controlCmd1",1,&BaseControl::callBack1,this));
	subscribers_.push_back(nh.subscribe("/controlCmd2",1,&BaseControl::callBack2,this));
	
	state1_pub = nh.advertise<ant_msgs::State1>("vehicleState1",10);
	state2_pub = nh.advertise<ant_msgs::State2>("vehicleState2",10);
	state3_pub = nh.advertise<ant_msgs::State3>("vehicleState3",10);
	state4_pub = nh.advertise<ant_msgs::State4>("vehicleState4",10);
	state_pub = nh.advertise<ant_msgs::State>("vehicleState",10);
#else
	subscribers_.push_back(nh.subscribe("/vehicleCmdSet",1,&BaseControl::cmd_CB,this));
	state_pub = nh.advertise<driverless_common::VehicleState>("/vehicleStateSet",5);
#endif
	
	timer_ = nh.createTimer(ros::Duration(0.01), &BaseControl::timer10ms_CB, this);
	
	if(!openSerial(stm32_serial_port_,stm32_port_name_,stm32_baudrate_))
		return false;
	
	if(!can2serial.configure_port(obd_can_port_name_.c_str()))
	{
		ROS_INFO("open port %s failed",obd_can_port_name_.c_str());
		return false;
	}
	else
		ROS_INFO("open port %s successfully",obd_can_port_name_.c_str());
	/*
	can2serial.clearCanFilter();
	
	can2serial.setCanFilter_alone(0x01,ID_STATE1); usleep(10000);
	can2serial.setCanFilter_alone(0x02,ID_STATE2); usleep(10000);
	can2serial.setCanFilter_alone(0x03,ID_STATE3); usleep(10000);
	can2serial.setCanFilter_alone(0x04,ID_STATE4); usleep(10000);
	
	can2serial.configBaudrate(500);
	*/
	can2serial.StartReading();
		
	ROS_INFO("System initialization completed");
	
	//usleep(1500000);
	
	//can2serial.clearCanFilter();
	return true;
}

void BaseControl::run()
{
	readFromStm32_thread_ptr_ = boost::shared_ptr<boost::thread > 
		(new boost::thread(boost::bind(&BaseControl::read_stm32_port, this)));
		
	boost::thread parse_msg(boost::bind(&BaseControl::parse_obdCanMsg,this)); 
	
	ros::spin();
}

void BaseControl::parse_obdCanMsg()
{
	CanMsg_t canMsg;

	while(ros::ok())
	{
		//ROS_INFO("parse_obdCanMsg  ing.....");
		usleep(3000);
		if(!can2serial.getCanMsg(canMsg))
			continue;
			
		//ROS_INFO("ID:%x",canMsg.ID);
	#if _USE_ANT_MESSAGES		
		switch(canMsg.ID)
		{
			case ID_STATE1:
				state1.act_gear = canMsg.data[0] >>4;
				state1.accel_pedal_position = canMsg.data[1] * 0.4;
				state1.brake_pedal = canMsg.data[2] & 0x01;
				state1.accel_pedal_position_valid = !(canMsg.data[2] & 0x02);
				state1.brake_pedal_valid = !(canMsg.data[2] & 0x04);
				state1.act_gear_valid = !(canMsg.data[2]&0x10);
				state1.vehicle_ready = bool(canMsg.data[2]&0x20);
				state1.driverless_mode = bool(canMsg.data[2]&0x40);
				state1.header.stamp = ros::Time::now();
				state1_pub.publish(state1);
				break;
				
			case ID_STATE2:
				state2.wheel_speed_FL_valid = !(canMsg.data[0] >>6);
				state2.wheel_speed_FL = ((canMsg.data[0]&0x3f)*256+canMsg.data[1])*0.0625;
				state2.wheel_speed_FR_valid = !(canMsg.data[1] >>6);
				state2.wheel_speed_FR = ((canMsg.data[2]&0x3f)*256+canMsg.data[3])*0.0625;
				
				state2.wheel_speed_RL_valid = !(canMsg.data[4] >>6);
				state2.wheel_speed_RL = ((canMsg.data[4]&0x3f)*256+canMsg.data[5])*0.0625;
				
				state2.wheel_speed_RR_valid = !(canMsg.data[6] >>6);
				state2.wheel_speed_RR = ((canMsg.data[6]&0x3f)*256+canMsg.data[7])*0.0625;
				
				{
				size_t i =0;
				float speed = 0.0;
				if(state2.wheel_speed_FL_valid==true) {i++; speed += state2.wheel_speed_FL;}
				if(state2.wheel_speed_FR_valid==true) {i++; speed += state2.wheel_speed_FR;}
				if(state2.wheel_speed_RL_valid==true) {i++; speed += state2.wheel_speed_RL;}
				if(state2.wheel_speed_RR_valid==true) {i++; speed += state2.wheel_speed_RR;}
				
				state2.vehicle_speed = speed/i /3.6; //m/s
				}
				state2.header.stamp = ros::Time::now();
				state2_pub.publish(state2);
				break;
				
			case ID_STATE3:
				state3.driverless_mode= bool(canMsg.data[0]&0x01);
				state3.turn_light_R = bool(canMsg.data[1]&0x01);
				state3.turn_light_L = bool(canMsg.data[1]&0x02);
				
				state3.parkTail_light = bool(canMsg.data[1]&0x08);
				state3.high_beam = bool(canMsg.data[1]&0x10);
				state3.low_beam = bool(canMsg.data[1]&0x20);
				state3.brake_light = bool(canMsg.data[2]&0x01);
				state3.horn = bool(canMsg.data[2]&0x02);
				state3.header.stamp = ros::Time::now();
				state3_pub.publish(state3);
				break;
				
			case ID_STATE4:
				state4.driverless_mode = bool(canMsg.data[0]&0x01);
				is_driverless_mode_ = state4.driverless_mode;
				state4.steeringAngle = 1080.0-(canMsg.data[1]*256+canMsg.data[2])*0.1;
				//std::cout << state4.steeringAngle << std::endl;
				state4.roadwheelAngle = state4.steeringAngle/g_steering_gearRatio;
				state4.manualCtrlDetected = bool(canMsg.data[0]&0x02);
				if(state4.manualCtrlDetected)
					manualCtrlDetected_ = true;

				if(state4.steeringAngle==6553.5)
					state4.steeringAngle_valid = 0;
				else
					state4.steeringAngle_valid = 1;
				state4.steeringAngle_speed = canMsg.data[3]*4; // deg/s
				
				state4.header.stamp = ros::Time::now();
				state4_pub.publish(state4);
				break;
			default:
				break;
		}
	#else
		std::lock_guard<std::mutex> lck(state_mutex_);
		switch(canMsg.ID)
		{
			case ID_STATE1:
				stateSet_.gear = canMsg.data[0] >>4;
				if((canMsg.data[2]&0x10))
					stateSet_.gear = stateSet_.GEAR_INVALID;

				stateSet_.accel_pedel_aperture = canMsg.data[1] * 0.4;
				stateSet_.brake_pedel_aperture = canMsg.data[2] & 0x01;
				if((canMsg.data[2] & 0x02))
					stateSet_.accel_pedel_aperture = 255;  //invalid
				if((canMsg.data[2] & 0x04))
					stateSet_.brake_pedel_aperture = 255;  //invalid
				
				stateSet_.ready = bool(canMsg.data[2]&0x20);
				stateSet_.driverless = bool(canMsg.data[2]&0x40);
				break;

			case ID_STATE2:
			{
				bool wheel_speed_FL_valid = !(canMsg.data[0] >>6);
				float wheel_speed_FL = ((canMsg.data[0]&0x3f)*256+canMsg.data[1])*0.0625;
				bool wheel_speed_FR_valid = !(canMsg.data[1] >>6);
				float wheel_speed_FR = ((canMsg.data[2]&0x3f)*256+canMsg.data[3])*0.0625;
				bool wheel_speed_RL_valid = !(canMsg.data[4] >>6);
				float wheel_speed_RL = ((canMsg.data[4]&0x3f)*256+canMsg.data[5])*0.0625;
				bool wheel_speed_RR_valid = !(canMsg.data[6] >>6);
				float wheel_speed_RR = ((canMsg.data[6]&0x3f)*256+canMsg.data[7])*0.0625;
				{
					size_t i =0;
					float speed = 0.0;
					if(wheel_speed_FL_valid) {i++; speed += wheel_speed_FL;}
					if(wheel_speed_FR_valid) {i++; speed += wheel_speed_FR;}
					if(wheel_speed_RL_valid) {i++; speed += wheel_speed_RL;}
					if(wheel_speed_RR_valid) {i++; speed += wheel_speed_RR;}
					
					stateSet_.speed = speed / i; //km/h
					if(i == 0) stateSet_.speed_validity = false;
					else stateSet_.speed_validity = true;
				}
				break;
			}
			case ID_STATE3:
			{
				stateSet_.turnlight_l = bool(canMsg.data[1]&0x02);
				stateSet_.turnlight_r = bool(canMsg.data[1]&0x01);
				stateSet_.brake_light = bool(canMsg.data[2]&0x01);
				stateSet_.low_beam = bool(canMsg.data[1]&0x20);
				stateSet_.high_beam = bool(canMsg.data[1]&0x10);
				stateSet_.horn = bool(canMsg.data[2]&0x02);
				break;
			}
			case ID_STATE4:
			{
				stateSet_.driverless = is_driverless_mode_ = bool(canMsg.data[0]&0x01);
				stateSet_.roadwheel_angle = (1080.0-(canMsg.data[1]*256+canMsg.data[2])*0.1)/g_steering_gearRatio;
				stateSet_.roadwheel_angle_validity = !(canMsg.data[1] == 0xff && canMsg.data[2] == 0xff);
				stateSet_.manualctrl_detected = bool(canMsg.data[0]&0x02);
				
				// 检测到手动控制, 此处只置位,不复位
				if(stateSet_.manualctrl_detected)
					manualCtrlDetected_ = true;

				float steeringAngle_speed = canMsg.data[3]*4; // deg/s
				break;
			}
			default:
				break;
		}
		#endif
	}
}

void BaseControl::read_stm32_port()
{
	size_t len = 0;
	stm32_serial_port_->flushInput();
	
	while(ros::ok())
	{
		ros::Duration(0.02).sleep();
		try 
		{
			len = stm32_serial_port_->read(stm32_data_buf, STM32_MAX_NOUT_SIZE);
			//ROS_INFO("read_stm32_port get %d bytes",len);
		}
		catch (std::exception &e) 
		{
	        std::stringstream output;
	        output << "Error reading from serial port: " << e.what();
	        std::cout << output.str() <<std::endl;
    	}
    	if(len == 0) continue;
    	
    	 //for(int i=0;i<len;i++)
    	 //	printf("%x\t",stm32_data_buf[i]);
    	 //std::cout << std::endl;
    	
		Stm32BufferIncomingData(stm32_data_buf, len);
	}
}

void BaseControl::Stm32BufferIncomingData(unsigned char *message, unsigned int length)
{
	static int buffer_index = 0;
	static int bytes_remaining =0;
	
	// add incoming data to buffer
	for (unsigned int ii=0; ii<length; ii++) 
	{// make sure bufIndex is not larger than buffer
		if(buffer_index >= STM32_MAX_PKG_BUF_LEN)
		{
			buffer_index = 0;
			  printf("Overflowed receive buffer. Buffer cleared.");
		}
		switch(buffer_index)
		{
			case 0: //nothing
				if(message[ii]==Stm32MsgHeaderByte0)
				{
					stm32_pkg_buf[buffer_index++] = message[ii];
				}
				bytes_remaining = 0;
				break;
			case 1:
				if(message[ii]==Stm32MsgHeaderByte1)
				{
					stm32_pkg_buf[buffer_index++] = message[ii];
					bytes_remaining =2; //2 bytes pkgLen
				}
				else
				{
					buffer_index = 0;
					bytes_remaining = 0;
				}
				break;
			case 2:
			case 3:
				stm32_pkg_buf[buffer_index++]=message[ii];
				bytes_remaining --;
				if(bytes_remaining == 0)
				{
					bytes_remaining = (stm32_pkg_buf[2] << 8) + stm32_pkg_buf[3] ;
					
					//根据实际发送的包长最大小值进行限定(多重数据正确保障) 
					if(bytes_remaining > 9 || bytes_remaining < 2)  
					{
						buffer_index = 0;
						break;
					}
				}
				break;
			default:
				stm32_pkg_buf[buffer_index++] = message[ii];
				bytes_remaining --;
				if(bytes_remaining == 0)
				{
					buffer_index = 0;
					parse_stm32_msgs();
				}
				break;
		}
	}// end for
}

void BaseControl::parse_stm32_msgs()
{
	const stm32MsgHeader* header = (const stm32MsgHeader*)stm32_pkg_buf;
	int data_len = ntohs(header->pkgLen);
	if(stm32_pkg_buf[data_len+3] != generateCheckNum(stm32_pkg_buf,data_len+4))
		return ;
	// std::cout << "pkg id: " << int(header->id) << std::endl;
	if(header->id == 0x01)
	{
		const stm32Msg1_t* msg1 = (const stm32Msg1_t*)stm32_pkg_buf;

		static bool last_allow_driverless = false;
		emergency_brake_key_ = msg1->is_emergency_brake;

	    allow_driverless_ = (msg1->is_start && !emergency_brake_key_);

		if(last_allow_driverless == false && allow_driverless_)
			manualCtrlDetected_ = false; //重新允许自动驾驶，清除历史检测到的人工介入标志
		
		last_allow_driverless = allow_driverless_;

		// std::cout << "allow_driverless: " << allow_driverless_ << std::endl
		// 		<< "s_start: " << int(msg1->is_start) << "\nis_emergency_brake:" <<  int(msg1->is_emergency_brake)
		// 		<< std::endl;;
	}
}

void BaseControl::timer10ms_CB(const ros::TimerEvent& event)
{
	static uint32_t cnt = 0;
	static bool ctrlCmdvalid = false;
	++ cnt;
	if(cnt%5 == 0) //50ms
	{
		//send cmd to stm32
		send_to_stm32_buf[5] = stm32_brake_ & 0x7f;
		if(is_driverless_mode_) send_to_stm32_buf[5] |= 0x80;
		else send_to_stm32_buf[5] &= 0x7f;

		send_to_stm32_buf[7] = generateCheckNum(send_to_stm32_buf,8);
		stm32_serial_port_->write(send_to_stm32_buf,8);

		#if _USE_ANT_MESSAGES
			//发布汽车状态信息，该信息为4部分信息的整合
			ant_msgs::State state;
			state.act_gear = state1.act_gear;
			state.driverless_mode = state4.driverless_mode;
			state.hand_brake = 0;
			state.emergency_brake = 0;
			state.vehicle_ready = state1.vehicle_ready;
			state.vehicle_speed = state2.vehicle_speed;
			state.roadwheelAngle = state4.roadwheelAngle;
			state_pub.publish(state);
		#else
			std::lock_guard<std::mutex> lck(state_mutex_);
			stateSet_.base_ready = allow_driverless_;
			stateSet_.emergency_brake = emergency_brake_key_;
			state_pub.publish(stateSet_);
		#endif
	}
	if(cnt%5 == 1) //50ms
	{
		if(ctrlCmdvalid)
		{
			std::lock_guard<std::mutex> lck(canMsg_cmd1_mutex_);
			can2serial.sendCanMsg(canMsg_cmd1);
		}
	}
	else // 10ms
	{   
		if(ctrlCmdvalid) 
		{
			std::lock_guard<std::mutex> lck(canMsg_cmd2_mutex_);
			can2serial.sendCanMsg(canMsg_cmd2);
		}
	}
	if(cnt%30 == 0) // 300ms, 有效性检测
	{
		ctrlCmdvalid = (canMsg_cmd1_valid_ && canMsg_cmd2_valid_);
		canMsg_cmd1_valid_ = canMsg_cmd2_valid_ = false;
	}
}

#if _USE_ANT_MESSAGES
void BaseControl::callBack1(const ant_msgs::ControlCmd1::ConstPtr msg)
{
	if(msg->set_driverlessMode && allow_driverless_ && !manualCtrlDetected_)
		canMsg_cmd1.data[0] |= 0x01;
	else
		canMsg_cmd1.data[0] &= 0xfe;
	
	if(msg->set_remoteStart)
		canMsg_cmd1.data[0] |= 0x02;
	else
		canMsg_cmd1.data[0] &= 0xfd;
		
	if(msg->set_handBrake)
		canMsg_cmd1.data[0] |= 0x04;
	else
		canMsg_cmd1.data[0] &= 0xfb;
	
	if(msg->set_turnLight_R)
		canMsg_cmd1.data[1] |= 0x01;
	else
		canMsg_cmd1.data[1] &= 0xfe;
		
	if(msg->set_turnLight_L)
		canMsg_cmd1.data[1] |= 0x02;
	else
		canMsg_cmd1.data[1] &= 0xfd;
		
	if(msg->set_lowBeam)
		canMsg_cmd1.data[1] |= 0x20;
	else
		canMsg_cmd1.data[1] &= 0xdf;
		
	if(msg->set_reverseLight)
		canMsg_cmd1.data[1] |= 0x40;
	else
		canMsg_cmd1.data[1] &= 0xbf;
	
	if(msg->set_brakeLight)
		canMsg_cmd1.data[2] |= 0x01;
	else
		canMsg_cmd1.data[2] &= 0xfe;
		
	if(msg->set_horn)
		canMsg_cmd1.data[2] |= 0x02;
	else
		canMsg_cmd1.data[2] &= 0xfd;
	
	canMsg_cmd1_valid_ = true;
}


void BaseControl::callBack2(const ant_msgs::ControlCmd2::ConstPtr msg)
{
	uint8_t set_gear = msg->set_gear;
	if(!is_driverless_mode_ )
		set_gear = msg->GEAR_NEUTRAL;

	float set_speed = msg->set_speed;
	float set_brake = msg->set_brake;
	float currentSpeed = state2.vehicle_speed * 3.6;
	
	if(msg->set_speed == 0)
	{
		set_brake = fabs(currentSpeed - msg->set_speed) *3 + 40;
	}
	//当设定速度低于当前速度时，制动
	else if(currentSpeed  > 2.0 + msg->set_speed)
	{
		set_brake = (currentSpeed - msg->set_speed - 2.0) *3 + 40;
	}
	
	set_brake = (set_brake > msg->set_brake) ? set_brake :msg->set_brake;
	
	if(set_brake >0.0)
		set_speed = 0.0;
	else if(set_speed > MAX_SPEED-1) 
		set_speed = MAX_SPEED-1;
	//increment越大，加速度越大
	//设定速度越低，加速越快
	float increment = 3.0/(currentSpeed/5+1);
	
	if(set_speed - currentSpeed > increment )
		set_speed = currentSpeed + increment;
			
	canMsg_cmd2.data[0] &= 0xf0; //clear least 4bits
	canMsg_cmd2.data[0] |= (set_gear)&0x0f;
	
	canMsg_cmd2.data[1] = uint8_t(set_speed * 10 * 15.0 / MAX_SPEED);
	
	if(set_brake > 100) set_brake = 100;
	
	//制动分配,电制动/外部制动,0-40电制动,40-100,机械制动+电制动
	if(set_brake>40)
	{
		canMsg_cmd2.data[2] = uint8_t(40 *2.5);
		stm32_brake_ = (set_brake - 40)/60.0 * 100;
	}
	else
	{
		canMsg_cmd2.data[2] = uint8_t(set_brake *2.5);
		stm32_brake_ = 0;
	}
	
	canMsg_cmd2.data[3] = uint8_t(msg->set_accelerate *50);
	
	static float last_set_steeringAngle = state4.steeringAngle;
	float current_set_steeringAngle = msg->set_roadWheelAngle * g_steering_gearRatio;  // -540~540deg
	
	if(current_set_steeringAngle>530.0) current_set_steeringAngle=530;
	else if(current_set_steeringAngle<-500.0) current_set_steeringAngle =-500.0;
	
	if(current_set_steeringAngle - last_set_steeringAngle > max_steering_speed_)
		current_set_steeringAngle = last_set_steeringAngle + max_steering_speed_;
	else if(current_set_steeringAngle - last_set_steeringAngle < -max_steering_speed_)
		current_set_steeringAngle = last_set_steeringAngle - max_steering_speed_;
	
	last_set_steeringAngle = current_set_steeringAngle;
	
	uint16_t steeringAngle = 10800 - (current_set_steeringAngle*10 - steering_offset_) ;
	
	canMsg_cmd2.data[4] =  uint8_t(steeringAngle / 256);
	canMsg_cmd2.data[5] = uint8_t(steeringAngle % 256);
	
	if(msg->set_emergencyBrake)
		canMsg_cmd2.data[6] |= 0x10;
	else
		canMsg_cmd2.data[6] &= 0xef;
	
	canMsg_cmd2_valid_ = true;
	//	std::cout << "stm32_brake_: " << int(stm32_brake_) << std::endl;
}
#else
void BaseControl::cmd_CB(const driverless_common::VehicleCtrlCmd::ConstPtr cmd)
{
	static double last_time = 0;
	double now_time = ros::Time::now().toSec();
	float dt = now_time - last_time;
	last_time = now_time;
	if(dt > 0.1) 
		return;

	uint8_t set_gear = CMD_GEAR_INITIAL;
	if(cmd->gear == cmd->GEAR_DRIVE) set_gear = CMD_GEAR_DRIVE;
	else if(cmd->gear == cmd->GEAR_REVERSE) set_gear = CMD_GEAR_REVERSE;
	else if(cmd->gear == cmd->GEAR_PARKING) set_gear = CMD_GEAR_PARKING;
	else if(cmd->gear == cmd->GEAR_NEUTRAL) set_gear = CMD_GEAR_NEUTRAL;

	// 当未处于自动驾驶状态时，同时置driverless=true gear=D/R, 将导致无法上档
	// 可以理解为，底层系统处于自动驾驶状态后，通过捕获上升沿进行换挡
	// 因此，当未处于自动驾驶状态时，无论命令如何，均发送空挡请求
	if(!stateSet_.driverless) set_gear = CMD_GEAR_NEUTRAL;

	// std::cout << "set_gear: " << int(set_gear) << std::endl;
	// std::cout << "cmd->driverless： " << int(cmd->driverless) <<"  " << int(allow_driverless_) << "  " 
	// 			<< int(manualCtrlDetected_) << std::endl;
	canMsg_cmd1_mutex_.lock();
	canMsg_cmd1.resetData();
	if(cmd->driverless && allow_driverless_ && !manualCtrlDetected_){
		canMsg_cmd1.data[0] |= 0x01;
	}else{
		canMsg_cmd1.data[0] &= 0xfe;
	}
		
	if(cmd->hand_brake) canMsg_cmd1.data[0] |= 0x04;
	if(cmd->turnlight_r) canMsg_cmd1.data[1] |= 0x01;
	if(cmd->turnlight_l) canMsg_cmd1.data[1] |= 0x02;
	if(cmd->low_beam) canMsg_cmd1.data[1] |= 0x20;
	if(cmd->high_beam) canMsg_cmd1.data[1] |= 0x40;
	if(cmd->brake_light) canMsg_cmd1.data[2] |= 0x01;
	if(cmd->horn) canMsg_cmd1.data[2] |= 0x02;
	canMsg_cmd1_valid_ = true;
	canMsg_cmd1_mutex_.unlock();

	canMsg_cmd2_mutex_.lock();
	canMsg_cmd2.resetData();

	float set_speed = cmd->speed;
	float set_brake = cmd->brake;

	state_mutex_.lock();
	float currentSpeed = stateSet_.speed;
	state_mutex_.unlock();

	//increment越大，加速度越大
	//设定速度越低，加速越快
	float increment = 3.0/(currentSpeed/5+1);
	
	if(set_speed == 0) 
		set_brake = max(fabs(currentSpeed - cmd->speed)*1.0 + 40, set_brake);
	else if(set_speed < currentSpeed-5.0) //当设定速度低于当前速度时，制动
		set_brake = max((currentSpeed - set_speed) + 40, set_brake);
	else if(set_speed - currentSpeed > increment)
		set_speed = currentSpeed + increment;

	if(set_brake > 100) set_brake = 100;
	if(set_brake > 0.0) set_speed = 0.0;
	if(set_speed > MAX_SPEED-1) set_speed = MAX_SPEED-1;

	canMsg_cmd2.data[0] |= set_gear&0x0f;
	canMsg_cmd2.data[1] = uint8_t(set_speed * 10 * 15.0 / MAX_SPEED);
	
	//制动分配,电制动/外部制动,0-40电制动,40-100,机械制动+电制动
	if(set_brake>40)
	{
		canMsg_cmd2.data[2] = uint8_t(40 *2.5);
		stm32_brake_ = (set_brake - 40)/60.0 * 100;
	}
	else
	{
		canMsg_cmd2.data[2] = uint8_t(set_brake *2.5);
		stm32_brake_ = 0;
	}
	
	canMsg_cmd2.data[3] = uint8_t(cmd->accelerate *50);
	
	state_mutex_.lock();
	static float last_set_steeringAngle = stateSet_.roadwheel_angle * g_steering_gearRatio;
	state_mutex_.unlock();

	float current_set_steeringAngle = cmd->roadwheel_angle * g_steering_gearRatio;  // -540~540deg
	
	if(current_set_steeringAngle>530.0) current_set_steeringAngle=530;
	else if(current_set_steeringAngle<-500.0) current_set_steeringAngle =-500.0;
	
	if(current_set_steeringAngle - last_set_steeringAngle > max_steering_speed_ * dt)
		current_set_steeringAngle = last_set_steeringAngle + max_steering_speed_ * dt;
	else if(current_set_steeringAngle - last_set_steeringAngle < -max_steering_speed_ * dt)
		current_set_steeringAngle = last_set_steeringAngle - max_steering_speed_ * dt;
	
	last_set_steeringAngle = current_set_steeringAngle;
	
	uint16_t steeringAngle = 10800 - (current_set_steeringAngle*10 - steering_offset_) ;
	
	canMsg_cmd2.data[4] =  uint8_t(steeringAngle / 256);
	canMsg_cmd2.data[5] = uint8_t(steeringAngle % 256);
	
	if(cmd->emergency_brake) canMsg_cmd2.data[6] |= 0x10;
	canMsg_cmd2_valid_ = true;
	canMsg_cmd2_mutex_.unlock();
}
#endif

uint8_t BaseControl::generateCheckNum(const void* voidPtr,size_t len)
{
	const uint8_t *ptr = (const uint8_t *)voidPtr;
    uint8_t sum=0;

    for(int i=2; i<len-1 ; i++)
        sum += ptr[i];
    return sum;
}

int main(int argc,char**argv)
{
	BaseControl base_control;
	
	if(base_control.init(argc,argv))
		base_control.run();
	
	printf("base_control_node has exited");
	return 0;
}












