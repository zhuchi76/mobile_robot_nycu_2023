#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 multiplied_msg;
ros::Publisher pub("from_arduino", &multiplied_msg);

void messageCb(const std_msgs::Int32& received_msg){
  multiplied_msg.data = received_msg.data * 2;
  pub.publish(&multiplied_msg);
}

ros::Subscriber<std_msgs::Int32> sub("to_arduino", messageCb);

void setup(){
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop(){
  nh.spinOnce();
  delay(50);
}
