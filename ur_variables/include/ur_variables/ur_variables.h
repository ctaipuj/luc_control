#ifndef UR_VARIABLES
#define UR_VARIABLES

#include <ros/ros.h>
#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>
#include <ur_msgs/SetPayload.h>
#include <industrial_msgs/RobotStatus.h>

/** @brief This class is used to manage the I/O screen of the UR3 robot using ros.
	
	This class uses the ROS service SetIO and SetPayload to access and edit controller's data. You can modify the analog and digital outputs and read inputs and outputs status. This class also get the robot status (is the robot emergency stopped?, is the robot powered? is the robot ready to move?).You can also set current and or voltage to an analog output. Ypu require the ur_modern_driver to use this class. Unexpected behaviour could be notice when using current-voltage functions. To retrieve data a IOStatus subscriber is used in the class. This class doesn't depend on URScript.
	@author Barrero Lizarazo, Nicolas
	@date November 2018
	*/

class ur_variables{

	private:
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Subscriber sub_status;
		ros::ServiceClient client;
		ros::ServiceClient client_payload;

		ur_msgs::SetIO srv;
		ur_msgs::SetPayload srv_payload;
		
		int analog_in;
		float analog_in_value;
		
		int analog_out;
		float analog_out_value;
		
		int digital_in, digital_in_value;
		
		int digital_out, digital_out_value;
		
		bool stopped, powered, motion;
		float payload;
		
		bool caller(int fun, int pin, float state);

	public:
	/** The constructor initializes subcriptions to io_states and robot_status topics also creates two service clients; set_io and set_payload. No topic is advertised using this class.
		*/
		ur_variables();
	/** This is the regular Callback from a ros node, this function updates I/O data.
		@param msg is the message type the nodes subscribes to: const ur_msgs::IOStates::ConstPtr&
	*/	
		
		void ur_variables_statusCallback(const ur_msgs::IOStates::ConstPtr& msg);
	/** This is the regular Callback from a ros node, this function updates robot_Status data.
		@param msg is the message type the nodes subscribes to: const industrial_msgs::RobotStatus::ConstPtr&
	*/
		
		void ur_variables_state_statusCallback(const industrial_msgs::RobotStatus::ConstPtr& msg);
	/** Sets a digital output to true/1.
		 @param address an int that represents the address(0-15)
		*/
		
		void setHigh(int address);
	/** Sets a digital output to false/0.
		 @param address an int that represents the address(0-15)
		*/	
		
		void setLow(int address);
	/** Sets robot's payload. Default is 900g (current gripper)
		 @param mass float payload (0-3)kg
		*/
			
		void setPayload(float mass=0.900);
	/** Changes a digital output state.
		 @param address an int that represents the address(0-15)
		 @param states an int 1 for true/on 0 for false/off
		*/
		
		void setOutput(int address, int state);
	/** Changes tool outputs state.
		 @param address an int that represents the address(0-1)
		 @param states an int 1 for true/on 0 for false/off
		*/	
		
		void setOutputTool(int address, int state);
	/** Sets analog output current in mA. Unexpected behaviour can show up here.
		 @param address an int that represents analog outputs(0-1)
		 @param float current in mA (4mA-20mA)
		*/	
		
		void setAnalogCurrent(int address, float current);
	/** Sets analog output current as a percentage of max current(0->4mA, 1->20mA). Data is linear interpolated. Unexpected behaviour can show up here.
		 @param address an int that represents analog outputs(0-1)
		 @param float factor(0-1)
		*/	
		
		void setAnalogCurrentFactor(int address, float factor);
	/** Sets analog output voltage in V. Unexpected behaviour can show up here.
		 @param address an int that represents analog outputs(0-1)
		 @param float voltage in V (0V-10V)
		*/	
		
		void setAnalogVoltage(int address, float voltage);
	/** Sets analog output voltage as a percentage of max voltage(0->0V, 1->10V). Data is linear interpolated. Unexpected behaviour can show up here.
		 @param address an int that represents analog outputs(0-1)
		 @param float factor(0-1)
		*/
		
		void setAnalogVoltageFactor(int address, float factor);
	/** Gets robot's payload.
		 @return payload mass (0-3)kg
		*/
		
		float getPayload();
	/** Gets a digital output state.
		 @param address an int that represents the digital output address(0-15)
		 @return state 1 for true/on 0 for false/off
		*/
			
		int getOutput(int address);
	/** Gets a digital intput state.
		 @param address an int that represents the digital intput address(0-15)
		 @return state 1 for true/on 0 for false/off
		*/	
		
		int getInput(int address);
	/** Gets a tool output state.
		 @param address an int that represents the tool output address(0-1)
		 @return state 1 for true/on 0 for false/off
		*/	
		
		int getOutputTool(int address);
	/** Gets a tool input state.
		 @param address an int that represents the tool input address(0-1)
		 @return state 1 for true/on 0 for false/off
		*/	
		
		int getInputTool(int address);
	/** Gets analog output current in mA. Unexpected behaviour can show up here.
		 @param address an int that represents analog outputs(0-1)
		 @return float current in mA (4mA-20mA)
		*/
			
		float getAnalogOutputCurrent(int address);
	/** Gets analog output voltage in V. Unexpected behaviour can show up here.
		 @param address an int that represents analog outputs(0-1)
		 @return float voltage in V (0V-10V)
		*/	
			
		float getAnalogOutputVoltage(int address);
	/** Gets analog output current as a percentage of max current(0->4mA, 1->20mA). Data is linear interpolated. Unexpected behaviour can show up here.
		 @param address an int that represents analog outputs(0-1)
		 @return float factor(0-1)
		*/
			
		float getAnalogOutputCurrentFactor(int address);
	/** Gets analog output voltage as a percentage of max voltage(0->0V, 1->10V). Data is linear interpolated. Unexpected behaviour can show up here.
		 @param address an int that represents analog outputs(0-1)
		 @return float factor(0-1)
		*/	
		
		float getAnalogOutputVoltageFactor(int address);
	/** Gets analog input status in voltage or current according what is configured in the robot's controller.
		 @param address an int that represents analog inputs(0-1)
		 @return float status in V or mA according to robot's configuration.
		*/	
		
		float getAnalogInputStatus(int address);
	/** Is the robot emergency stopped?
		 @return true or false
		*/	
		
		bool isEStopped();
	/** Is the robot powered up?
		 @return true or false
		*/	
		
		bool isPowered();
	/** Is motion posible?
		 @return true or false
		*/	
		
		bool isMotionPossible();
};
#endif
