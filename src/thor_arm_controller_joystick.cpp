#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <serial/serial.h>

#define JOY_DEAD_ZONE 0.3

serial::Serial ser;
float joint_inc[7];
float joint_degrees[7];
float waypoint[11][7];
std::string joint_deg_str[7];
std::string gcode;
bool new_move = false;

//Convert floats to strings for sending as a G-code command
char temp[10];
void gcode_write(float *joint_degrees){
  for ( int i = 0 ; i <= 6 ; i++ ) {
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
        + " M3 S" + joint_deg_str[6]
	+ "\r\n";
  
  ROS_INFO_STREAM("Serial write: " << gcode);
  ser.write(gcode);
}

void save_waypoint(int button){
   for ( int i = 0 ; i <= 6 ; i++ ) {
     waypoint[button][i] = joint_degrees[i];
   }
}

void load_waypoint(int button){
   for ( int i = 0 ; i <= 6 ; i++ ) {
     joint_degrees[i] = waypoint[button][i];
   }
   gcode_write(joint_degrees);
}

void home_waypoint(){
  for ( int i = 0 ; i <= 6 ; i++ ) {
     joint_degrees[i] = 0;
  }
  gcode_write(joint_degrees);
}


void joy_cb(const sensor_msgs::Joy& joy)
{  
  //Art 1
  if (abs(joy.axes[0]) > JOY_DEAD_ZONE)
    joint_inc[0] = -joy.axes[0];
  else joint_inc[0] = 0;
  //Art 2
  if (abs(joy.axes[1]) > JOY_DEAD_ZONE)
    joint_inc[1] = joy.axes[1];
  else joint_inc[1] = 0;
  //Art 3
  if (abs(joy.axes[4]) > JOY_DEAD_ZONE)
    joint_inc[2] = joy.axes[4];
  else joint_inc[2] = 0;
  //Art 4
  if (abs(joy.axes[3]) > JOY_DEAD_ZONE)
    joint_inc[3] = joy.axes[3];
  else joint_inc[3] = 0;
  //Art 5 - differential bevel gear wrist
  joint_inc[4] = 2*(2*joy.axes[6] + joy.axes[7]);
  //Art 6 - differential bevel gear wrist
  joint_inc[5] = 2*(2*joy.axes[6] - joy.axes[7]);

  //Gripper - considered the 7th joint
  //Pull the right trigger (axes[5]): close the gripper 
  //Pull the left trigger (axes[2]): open the gripper
  joint_inc[6] = 30*((1-joy.axes[5]) - (1-joy.axes[2]));

  //Waypoint saving and loading
  //If LB + A/B/X/Y are pressed, save the current joint state as a waypoint
  //If RB + A/B/X/Y are pressed, load that point
  //If select is pressed, return home
  if (       joy.buttons[4] == 1){
         if (joy.buttons[0] == 1){save_waypoint(0);}
    else if (joy.buttons[1] == 1){save_waypoint(1);}
    else if (joy.buttons[2] == 1){save_waypoint(2);}
    else if (joy.buttons[3] == 1){save_waypoint(3);}
  }
  else if (  joy.buttons[5] == 1){
         if (joy.buttons[0] == 1){load_waypoint(0);}
    else if (joy.buttons[1] == 1){load_waypoint(1);}
    else if (joy.buttons[2] == 1){load_waypoint(2);}
    else if (joy.buttons[3] == 1){load_waypoint(3);}
  }
  else if (  joy.buttons[6] == 1){home_waypoint();}
}

int joint_limit [7] = {180,
                        90,
                        90,
                       180,
                        90,
                       180,
                      1000};

void gcode_cb(const ros::TimerEvent&){
  for ( int i = 0 ; i <= 6 ; i++ ) {

    if (joint_inc[i] != 0){
      joint_degrees[i] += joint_inc[i];
      new_move = true;
    }
  
    if (     joint_degrees[i] >  joint_limit[i]){ joint_degrees[i] =  joint_limit[i];}
    else if (joint_degrees[i] < -joint_limit[i]){ joint_degrees[i] = -joint_limit[i];}
  }  
  //Additional check for gripper
  if (joint_degrees[6] < 0) joint_degrees[6] = 0;
  
  if (new_move == true){  
    gcode_write(joint_degrees);
    new_move = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thor_arm_controller");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("joy", 1000, joy_cb);
  //ros::Publisher grbl_response = nh.advertise<sensor_msgs::JointState>("grbl_response", 1000);
  //ros::Publisher grbl_response = nh.advertise<std_msgs::String>("grbl_response", 1000);
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
  ros::Timer update_gcode = nh.createTimer(ros::Duration(0.1), gcode_cb);

  while(ros::ok()){
    ros::spinOnce();

    loop_rate.sleep();
    if(ser.available()){
      std_msgs::String result;
      result.data = ser.read(ser.available());
      ROS_INFO_STREAM("Serial read : " << result.data);
      //grbl_response.publish(result);
    }
  }
  return 0;
}
