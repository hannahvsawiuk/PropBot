#include <ros.h>
#include <std_msgs/UInt8.h>

//Creating Nodehandle
ros::NodeHandle node;

std_msgs::UInt8 pwm_left;
std_msgs::UInt8 pwm_right;


//Defining Publisher and callback
ros::Publisher talker_left("pwm_left", &pwm_left);
ros::Publisher talker_right("pwm_right", &pwm_right);

void callback_left ( const std_msgs::UInt8& msg_left){
pwm_left.data = msg_left.data;
talker_left.publish( &pwm_left );
}

void callback_right ( const std_msgs::UInt8& msg_right){
pwm_right.data = msg_right.data;
talker_right.publish( &pwm_right );
}

//Defining Subscriber
ros::Subscriber<std_msgs::UInt8> listener_left("listener_left", callback_left);
ros::Subscriber<std_msgs::UInt8> listener_right("listener_right", callback_right);

void setup()
{
//Initializing node
node.initNode();

//Configure Subscriber and Publisher
node.advertise(talker_left);
node.subscribe(listener_left);
node.advertise(talker_right);
node.subscribe(listener_right);
}
void loop()
{
node.spinOnce();
delay(2);
}
