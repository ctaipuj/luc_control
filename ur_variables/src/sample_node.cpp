#include "../include/ur_variables/ur_variables.h" //TODO You can #include <ur_variables/ur_variables.h> in your node in other package

int main(int argc, char** argv){
  ros::init(argc, argv, "ur_variables_sample");
  ros::NodeHandle node;
  ur_variables v;
  v.setHigh(0); //On
  //v.setHigh(17);
  v.setHigh(14); //on
  //v.setHigh(-2);
  v.setLow(7);//off
  v.setLow(0);//off
  v.setLow(14);//off
  v.setOutput(0,1);//on
  v.setOutput(16,2); //ERROR
  v.setOutput(16,0);//off
  v.setOutput(6,2); //ERROR
  v.setOutput(0,0);//off
  v.setOutput(1,1);//on
  v.setOutput(3,1);//on
  v.setOutput(1,0);//off
  v.setOutput(3,0);//off
  v.setOutputTool(0,1);//on
  v.setOutputTool(2,1);//on
  v.setOutputTool(1,1);//on
  v.setOutputTool(1,0);//off
  v.setOutputTool(0,0);//off
  v.setOutputTool(0,2);//ERROR
 // v.setAnalogVoltage(0,8.5);
  v.setPayload(0.9); //set to 900g
  ROS_INFO("TEST %f",v.getAnalogOutputCurrentFactor(0));
  ros::Duration(1).sleep();
  return 0;
}
