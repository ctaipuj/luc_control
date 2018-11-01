#include "../include/ur_variables/ur_variables.h"
	
ur_variables::ur_variables(){
	sub=n.subscribe("/ur_driver/io_states",1000, &ur_variables::ur_variables_statusCallback,this);
	sub_status=n.subscribe("/ur_driver/robot_status",1000,&ur_variables::ur_variables_state_statusCallback,this);
	client=n.serviceClient<ur_msgs::SetIO>("/ur_driver/set_io");
	client_payload=n.serviceClient<ur_msgs::SetPayload>("/ur_driver/set_payload");
	ros::Duration(1.0).sleep();
	
	analog_in=0;
	analog_in_value=0.0;
	analog_out=0;
	analog_out_value=0.0;
	
	digital_in=0;
	digital_in_value=0;

	digital_out=0;
	digital_out_value=0;
	
	stopped=0;
	powered=0;
	motion=0;
	payload=0;
		
	setPayload();
}
		
void ur_variables::ur_variables_statusCallback(const ur_msgs::IOStates::ConstPtr& msg){

	digital_in_value=msg->digital_in_states[digital_in].state;
	digital_out_value=msg->digital_out_states[digital_out].state;
			
	analog_in_value=msg->analog_in_states[analog_in].state;
	analog_out_value=msg->analog_out_states[analog_out].state;
}
		
void ur_variables::ur_variables_state_statusCallback(const industrial_msgs::RobotStatus::ConstPtr& msg){
	
	stopped=msg->e_stopped.val;
	powered=msg->drives_powered.val;
	motion=msg->motion_possible.val;
}

void ur_variables::setHigh(int address){
	if(address>15 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-15 for digital outputs",address);
		goto end;
	}
	if(caller(1,address,1)){
		if(address<8)
			ROS_INFO("Address #%i ----> 1x0%i set high",address,address);
		else
			ROS_INFO("Address #%i ----> 2x0%i set high",address,address-8);
	}
	end:;
}
	
void ur_variables::setLow(int address){
	if(address>15 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-15 for digital outputs",address);
		goto end;
	}
	if(caller(1,address,0)){
		if(address<8)
			ROS_INFO("Address #%i ----> 1x0%i set low",address,address);
		else
			ROS_INFO("Address #%i ----> 2x0%i set low",address,address-8);
	}
	end:;
}

void ur_variables::setPayload(float mass){
	if(mass<0 || mass>3){
		ROS_ERROR("Payload for UR3 is from 0 to 3kg");
		goto end;
	}
	if(mass<0.899)
		ROS_WARN("You set a mass of %.3fkg this is less than the mass of the end effector",mass);
	srv_payload.request.payload=mass;
	payload=mass;
	if(!client_payload.call(srv_payload))
		ROS_ERROR("Unable to set payload. Something went wrong");
	else
		ROS_INFO("Payload set to %.3fkg",mass);
	end:;
}
	
void ur_variables::setOutput(int address, int state){
	if(address>15 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-15 for digital outputs.",address);
		goto end;
	}
	if(state!=0 && state!=1){
		ROS_ERROR("Invalid state %i. Only valid 0 or 1 for digital signals",state);
		goto end;
	}				
	if(caller(1,address,state)){
		if(address<8 && state==0)
			ROS_INFO("Address #%i ----> 1x0%i set low",address,address);
		if(address<8 && state==1)
			ROS_INFO("Address #%i ----> 1x0%i set high",address,address);
		if(address>=8 && state==0)
			ROS_INFO("Address #%i ----> 2x0%i set low",address,address-8);
		if(address>=8 && state==1)
			ROS_INFO("Address #%i ----> 2x0%i set high",address,address-8);
	}
	end:;
}

void ur_variables::setOutputTool(int address, int state){
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for digital tool outputs.",address);
		goto end;
	}
	if(state!=0 && state!=1){
		ROS_ERROR("Invalid state %i. Only valid 0 or 1 for digital signals",state);
		goto end;
	}
	if(caller(1,address+16,state)){
		if(state==0)
			ROS_INFO("Address #%i ----> toolx0%i set low",address,address);
		else
			ROS_INFO("Address #%i ----> toolx0%i set high",address,address);
	}
	end:;
}

void ur_variables::setAnalogCurrent(int address, float current){//in mA
	float current_normalized;

	if(current<4 || current>20){
		ROS_ERROR("Current from 4mA to 20mA");
		goto end;
	}
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for analog outputs.",address);
		goto end;
	}
	analog_out=address;
	ros::spinOnce();
	if(analog_out_value>0.025 || analog_out_value<=0.004){
		ROS_WARN("Check the analog output mode at Toolx0%i address. Appears to be in voltage mode",address);
		goto end;
	}
	current_normalized=0.0625*current-0.25;
	if(caller(3,address,current_normalized))
		ROS_INFO("Address #%i ----> analogx0%i set to %.2fmA",address,address,current);
	end:;
}

void ur_variables::setAnalogCurrentFactor(int address, float factor){//in mA
	
	if(factor<0.0 || factor>1.0){
		ROS_ERROR("Current factor from 0.0 to 1.0");
		goto end;
	}
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for analog outputs.",address);
		goto end;
	}
	analog_out=address;
	ros::spinOnce();
	if(analog_out_value>0.025 || analog_out_value<0.004){
		ROS_WARN("Check the analog output mode at Toolx0%i address. Appears to be in voltage mode",address);
		goto end;
	}
	if(caller(3,address,factor))
		ROS_INFO("Address #%i ----> analogx0%i set to %.1f%%",address,address,factor*100);
	end:;
}
		
void ur_variables::setAnalogVoltage(int address, float voltage){//in V
	
	float voltage_normalized;

	if(voltage<0 || voltage>10){
		ROS_ERROR("Voltage from 0V to 10V");
		goto end;
	}
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for analog outputs.",address);
		goto end;
	}
	analog_out=address;
	ros::spinOnce();
	if(analog_out_value<0.025 && analog_out_value>0.004){
		ROS_WARN("Check the analog output mode at Toolx0%i address. Appears to be in current mode",address);
		goto end;
	}
	voltage_normalized=0.1*voltage;
	if(caller(3,address,voltage_normalized))
		ROS_INFO("Address #%i ----> analogx0%i set to %.1fV",address,address,voltage);
	end:;
}

void ur_variables::setAnalogVoltageFactor(int address, float factor){//in V
	
	if(factor<0.0 || factor>1.0){
		ROS_ERROR("Voltage factor from 0.0 to 1.0");
		goto end;
	}
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for analog outputs.",address);
		goto end;
	}
	analog_out=address;
	ros::spinOnce();
	if(analog_out_value<0.025 && analog_out_value>0.004){
		ROS_WARN("Check the analog output mode at Toolx0%i address. Appears to be in current mode",address);
		goto end;
	}
	if(caller(3,address,factor))
		ROS_INFO("Address #%i ----> analogx0%i set to %.1f%%",address,address,factor*100);
	end:;
}
			
float ur_variables::getPayload(){
	return payload;
}
	
int ur_variables::getOutput(int address){
	if(address>15 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-15 for digital outputs.",address);
		return -1;
	}
	digital_out=address;
	ros::spinOnce();
	return digital_out_value;
}

int ur_variables::getInput(int address){
	if(address>15 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-15 for digital inputs.",address);
		return -1;
	}
	digital_in=address;
	ros::spinOnce();
	return digital_in_value;
}

int ur_variables::getOutputTool(int address){
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for digital tool outputs.",address);
		return -1;
	}
	digital_out=address+16;
	ros::spinOnce();
	return digital_out_value;
}

int ur_variables::getInputTool(int address){
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for digital tool inputs.",address);
		return -1;
	}
	digital_in=address+16;
	ros::spinOnce();
	return digital_in_value;
}

float ur_variables::getAnalogOutputCurrent(int address){
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for analog outputs.",address);
		return -1;
	}
	analog_out=address;
	ros::spinOnce();
	if(analog_out_value>0.025 || analog_out_value<0.004){
		ROS_WARN("Check the analog output mode at OutToolx0%i address. Appears to be in voltage mode",address);
		return -1;
	}
	return analog_out_value*1000; //TODO in  mA or A
}
		
float ur_variables::getAnalogOutputVoltage(int address){
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for analog outputs.",address);
		return -1;
	}
	analog_out=address;
	ros::spinOnce();
	if(analog_out_value<0.025 && analog_out_value>0.004){
		ROS_WARN("Check the analog output mode at OutToolx0%i address. Appears to be in current mode",address);
		return -1;
	}
	return analog_out_value;
}

float ur_variables::getAnalogOutputCurrentFactor(int address){
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for analog outputs.",address);
		return -1;
	}
	analog_out=address;
	ros::spinOnce();
	if(analog_out_value>0.025 || analog_out_value<0.004){
		ROS_WARN("Check the analog output mode at OutToolx0%i address. Appears to be in voltage mode",address);
		return -1;
	}
	return (analog_out_value*1000)*0.0625-0.25;
}
		
float ur_variables::getAnalogOutputVoltageFactor(int address){
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for analog outputs.",address);
		return -1;
	}
	analog_out=address;
	ros::spinOnce();
	if(analog_out_value<0.025 && analog_out_value>0.004){
		ROS_WARN("Check the analog output mode at OutToolx0%i address. Appears to be in current mode",address);
		return -1;
	}
	return analog_out_value/0.1;
}
		
float ur_variables::getAnalogInputStatus(int address){
	ROS_WARN("Verify the mode of input inToolx0%i to retrieve accurate data",address);
	if(address>1 || address<0){
		ROS_ERROR("Invalid Address %i. Only valid 0-1 for analog inputs.",address);
		return -1;
	}
	analog_in=address;
	ros::spinOnce();
	return analog_in_value;
	}		
		
bool ur_variables::isEStopped(){
	ros::spinOnce();
	return stopped;
}
		
bool ur_variables::isPowered(){
	ros::spinOnce();
	return powered;
}

bool ur_variables::isMotionPossible(){
	ros::spinOnce();
	return motion;
}



bool ur_variables::caller(int fun, int pin, float state){
	srv.request.fun=fun;
	srv.request.pin=pin;
	srv.request.state=state;
	if (!client.call(srv)){
		ROS_ERROR("Unable to update controller data");
		return 0;
	}
	ros::Duration(0.1).sleep(); //OJO AL TIEMPO
	return 1;
}
