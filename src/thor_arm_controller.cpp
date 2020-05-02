#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <serial/serial.h>

serial::Serial ser;
float joint_degrees[6];
char temp[10];
std::string joint_deg_str[6];
std::string gcode;

void gcode_cb(const sensor_msgs::JointState& joint_state)
{  
  //Art 1
  joint_degrees[0] = joint_state.position[0] * 180 / 3.14;
  //Art 2
  joint_degrees[1] = joint_state.position[1] * 180 / 3.14;
  //Art 3
  joint_degrees[2] = joint_state.position[2] * 180 / 3.14;
  //Art 4
  joint_degrees[3] = joint_state.position[3] * 180 / 3.14;
  //Art 5 - differential bevel gear wrist
  joint_degrees[4] = (2*joint_state.position[5] + joint_state.position[4]) * 180 / 3.14;
  //Art 6 - differential bevel gear wrist
  joint_degrees[5] = (2*joint_state.position[5] - joint_state.position[4]) * 180 / 3.14;
  
  //Convert floats to strings for sending as a G-code command
  for ( int i = 0 ; i <= 5 ; i++ ) {
    //joint_degrees[i] = roundf(joint_degrees[i]*100)/100;
    //joint_deg_str[i] = std::to_string(joint_degrees[i]);
    sprintf(temp, "%.3f", joint_degrees[i]);
    joint_deg_str[i] = temp;
    //ROS_INFO_STREAM("Convert "<< std::to_string(joint_state.position[i]));
    //ROS_INFO_STREAM("     to " << joint_deg_str[i]);
  }
  
  gcode = "G0";
  gcode +=" A" + joint_deg_str[0]
        + " B" + joint_deg_str[1]  //B and C are both part of Art 2
        + " C" + joint_deg_str[1]
        + " D" + joint_deg_str[2]
        + " X" + joint_deg_str[3]
        + " Y" + joint_deg_str[4]
        + " Z" + joint_deg_str[5]
        + "\r\n";
  ROS_INFO_STREAM("Serial write: " << gcode);
  ser.write(gcode);

/*
  ser.write("G0");
  ser.write(" A"); ser.write(joint_deg_str[0]);
  ser.write(" B"); ser.write(joint_deg_str[1]);
  ser.write(" C"); ser.write(joint_deg_str[1]);
  ser.write(" D"); ser.write(joint_deg_str[2]);
  ser.write(" X"); ser.write(joint_deg_str[3]);
  ser.write(" Y"); ser.write(joint_deg_str[4]);
  ser.write(" Z"); ser.write(joint_deg_str[5]);
  ser.write("\r\n");
*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thor_arm_controller");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states", 1000, gcode_cb);
  //ros::Publisher grbl_response = nh.advertise<sensor_msgs::JointState>("grbl_response", 1000);
  ros::Publisher grbl_response = nh.advertise<std_msgs::String>("grbl_response", 1000);
  try{
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e){
     ROS_ERROR_STREAM("Unable to open port ");
     return -1;
  }

  if(ser.isOpen()){
    ROS_INFO_STREAM("Serial Port initialized");
  }else{
    return -1;
  }
  
  ros::Rate loop_rate(100);
  while(ros::ok()){
    ros::spinOnce();
    if(ser.available()){
      std_msgs::String result;
      result.data = ser.read(ser.available());
      ROS_INFO_STREAM("Serial read : " << result.data);
      grbl_response.publish(result);
    }
    loop_rate.sleep();
  }
  return 0;
}
