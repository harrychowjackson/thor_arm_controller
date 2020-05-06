/*
Harry Jackson - 2020-05-04
Thor_arm_controller_joystick.cpp
Takes input from a gamepad or joystick communicating on the ROS topic "joy" from
joy/joy_node and converts them to a g-code instruction for the Thor robotic arm.

*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <serial/serial.h>

  #define JOY_NODE joy
  #define SERIAL_PORT "/dev/ttyACM0"
  #define SERIAL_BAUD 115200
  #define GAMEPAD_A JOY_NODE.buttons[0]
  #define GAMEPAD_B JOY_NODE.buttons[1]
  #define GAMEPAD_X JOY_NODE.buttons[2]
  #define GAMEPAD_Y JOY_NODE.buttons[3]
  #define GAMEPAD_LB JOY_NODE.buttons[4]
  #define GAMEPAD_RB JOY_NODE.buttons[5]
  #define GAMEPAD_LSTICK JOY_NODE.buttons[9]
  #define GAMEPAD_RSTICK JOY_NODE.buttons[10]
  #define GAMEPAD_START JOY_NODE.buttons[7]
  #define GAMEPAD_SELECT JOY_NODE.buttons[6]
  #define GAMEPAD_HOME JOY_NODE.buttons[8]

  #define GAMEPAD_LS_X JOY_NODE.axes[0]
  #define GAMEPAD_LS_Y JOY_NODE.axes[1]
  #define GAMEPAD_RS_X JOY_NODE.axes[3]
  #define GAMEPAD_RS_Y JOY_NODE.axes[4]
  #define GAMEPAD_DPAD_X JOY_NODE.axes[6]
  #define GAMEPAD_DPAD_Y JOY_NODE.axes[7]
  #define GAMEPAD_LT JOY_NODE.axes[2]
  #define GAMEPAD_RT JOY_NODE.axes[5]
  #define JOY_DEAD_ZONE 0.2
  #define SPEED_MULTIPLIER 1.3

const int joint_max [7] =  {360,  //Art 1
                             90,  //Art 2
                             90,  //Art 3
                            360,  //Art 4
                            360,  //Art 5
                             90,  //Art 6
                           1000};
const int joint_min [7] = {-360,  //Art 1
                            -90,  //Art 2
                            -90,  //Art 3
                           -360,  //Art 4
                           -360,  //Art 5
                            -90,  //Art 6
                              0}; 

serial::Serial ser;
float joint_inc[7];
float joint_state[7];
float motor_pos[8];
float waypoint[11][7];
std::string ser_read;
char temp[10];
std::string motor_pos_str[8];
std::string gcode;
bool new_move = false;
bool grbl_ok = false;

//Convert floats to strings for sending as a G-code command
void gcode_write(float *joint_state){
  motor_pos[0] = joint_state[0];                       //Art 1 - base roll
  motor_pos[1] = joint_state[1];                       //Art 2 - "shoulder" pitch (2 motors)
  motor_pos[2] = joint_state[1];                       //
  motor_pos[3] = joint_state[2];                       //Art 3 - "elbow" pitch
  motor_pos[4] = joint_state[3];                       //Art 4 - roll
  motor_pos[5] = 2 * joint_state[4] - joint_state[5];  //Art 5 - wrist pitch
  motor_pos[6] = 2 * joint_state[4] + joint_state[5];  //Art 6 - wrist roll
  motor_pos[7] = joint_state[6];                       //Gripper

  for ( int i = 0 ; i <= 7 ; i++ ) {
    sprintf(temp, "%.3f", motor_pos[i]);
    motor_pos_str[i] = temp;
    //ROS_INFO_STREAM("Convert "<< std::to_string(joint_state.position[i]));
    //ROS_INFO_STREAM("     to " << joint_deg_str[i]);
  }
  gcode = "G0";
  gcode +=" A" + motor_pos_str[0]
	+ " B" + motor_pos_str[1]  
	+ " C" + motor_pos_str[2]
	+ " D" + motor_pos_str[3]
	+ " X" + motor_pos_str[4]
	+ " Y" + motor_pos_str[5]
	+ " Z" + motor_pos_str[6]
     + " M3 S" + motor_pos_str[7]
        + "\r\n";
  
  ROS_INFO_STREAM("Serial write: " << gcode);
  ser.write(gcode);
  new_move = false;
  grbl_ok = false;
}

//Note: loading waypoints should not include gripper position. 
void save_waypoint(const int button){
   ROS_INFO_STREAM("Saving waypoint " << button);
   for ( int i = 0 ; i <= 6 ; i++ ) {
     waypoint[button][i] = joint_state[i];
   }
}

void load_waypoint(const int button){
   ROS_INFO_STREAM("Loading waypoint " << button);
   for ( int i = 0 ; i <= 5 ; i++ ) {
     joint_state[i] = waypoint[button][i]; //Don't change gripper position
   } 
   gcode_write(joint_state);
}

void home_waypoint(){
  ROS_INFO_STREAM("Returning to home upright position");
  for ( int i = 0 ; i <= 6 ; i++ ) {
     joint_state[i] = 0;
  }
  gcode_write(joint_state);
}

/* joy_cb
Sanitize input from the joystick to an amount to increment each joint position.
Also, if one of the A/B/X/Y buttons are pressed in conjunction with LB/RB, 
respectively save or load the a set of joint states.
*/
void joy_cb(const sensor_msgs::Joy& joy)
{  
  //Art 1
  if (abs(GAMEPAD_LS_X) > JOY_DEAD_ZONE)
    joint_inc[0] = -GAMEPAD_LS_X*SPEED_MULTIPLIER;
  else joint_inc[0] = 0;
  //Art 2
  if (abs(GAMEPAD_LS_Y) > JOY_DEAD_ZONE)
    joint_inc[1] = GAMEPAD_LS_Y*SPEED_MULTIPLIER;
  else joint_inc[1] = 0;
  //Art 3
  if (abs(GAMEPAD_RS_Y) > JOY_DEAD_ZONE)
    joint_inc[2] = GAMEPAD_RS_Y*SPEED_MULTIPLIER;
  else joint_inc[2] = 0;
  //Art 4
  if (abs(GAMEPAD_RS_X) > JOY_DEAD_ZONE)
    joint_inc[3] = GAMEPAD_RS_X*SPEED_MULTIPLIER;
  else joint_inc[3] = 0;

  joint_inc[4] = 4*GAMEPAD_DPAD_X*SPEED_MULTIPLIER;
  joint_inc[5] = 4*GAMEPAD_DPAD_Y*SPEED_MULTIPLIER;

  //Gripper - considered the 7th joint
  //Pull the right trigger (axes[5]): close the gripper 
  //Pull the left trigger (axes[2]): open the gripper
  joint_inc[6] = 30*((1-joy.axes[5]) - (1-joy.axes[2]));

  //Waypoint saving and loading
  //If LB + A/B/X/Y are pressed, save the current joint state as a waypoint
  //If RB + A/B/X/Y are pressed, load that point
  //If select is pressed, return home
  if (      GAMEPAD_LB == 1){
         if (GAMEPAD_A == 1){save_waypoint(0);}
    else if (GAMEPAD_B == 1){save_waypoint(1);}
    else if (GAMEPAD_X == 1){save_waypoint(2);}
    else if (GAMEPAD_Y == 1){save_waypoint(3);}
  }
  else if ( GAMEPAD_RB == 1){
         if (GAMEPAD_A == 1){load_waypoint(0);}
    else if (GAMEPAD_B == 1){load_waypoint(1);}
    else if (GAMEPAD_X == 1){load_waypoint(2);}
    else if (GAMEPAD_Y == 1){load_waypoint(3);}
  }
  else if ( GAMEPAD_SELECT == 1) home_waypoint();
/*  
  else if (GAMEPAD_HOME == 1){
    ser.write("$1 = 255"); //enable motors
    ROS_INFO_STREAM(" Enabling motors");
    grbl_ok = true; //force for debug purposes
  } 
  else if (GAMEPAD_START == 1){
    ser.write("$1 = 0"); //disable motors  
    ROS_INFO_STREAM("Disabling motors");
  }
*/
}

/*
gcode_cb
Converts the target joint state into motor positions to print as g-code. 
This gets called periodically to keep the real-life robot up to date with the 
target joint state, but not so fast as to overwhelm the GRBL command buffer.
*/
void gcode_cb(const ros::TimerEvent&){
  
  for ( int i = 0 ; i <= 6 ; i++ ) {

    if (joint_inc[i] != 0){
      joint_state[i] += joint_inc[i];
      new_move = true;
      //ROS_INFO_STREAM("non zero joint_inc " << i); 
    }
  
    if (     joint_state[i] > joint_max[i]){
      joint_state[i] = joint_max[i];}
    else if (joint_state[i] < joint_min[i]){
      joint_state[i] = joint_min[i];}
  }  
  
  if (new_move == true && grbl_ok == true){  
    gcode_write(joint_state);
  }
}

int main(int argc, char **argv)
{
//Initialize ROS network objects
  ros::init(argc, argv, "thor_arm_controller");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joy", 1000, joy_cb);
  ros::Rate loop_rate(50);

  ros::Timer update_gcode = nh.createTimer(ros::Duration(0.1), gcode_cb);

  //ros::Publisher grbl_response = nh.advertise<sensor_msgs::JointState>("grbl_response", 1000);
  //ros::Publisher grbl_response = nh.advertise<std_msgs::String>("grbl_response", 1000);

//Attemp to open Serial Port
  try{
    ser.setPort(SERIAL_PORT);
    ser.setBaudrate(SERIAL_BAUD);
    serial::Timeout to = serial::Timeout::simpleTimeout(5000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e){
     ROS_ERROR_STREAM("Unable to open port "SERIAL_PORT);
     return -1;
  }

  if(ser.isOpen()){
    ROS_INFO_STREAM("Serial port "SERIAL_PORT" initialized");
  }else{
    return -1;
  }
  
  while(ros::ok()){
    ros::spinOnce();

    loop_rate.sleep();
    if(ser.available()){
      ser_read = ser.read(ser.available());
      ROS_INFO_STREAM("Serial read : " << ser_read);
      if (grbl_ok == false){
        if (ser_read.find("ok") != std::string::npos) grbl_ok = true;
        else if (ser_read.find("Grbl 0.9") != std::string::npos) grbl_ok = true;        
      }
    }
  }
  return 0;
}
