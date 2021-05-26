#include "ros/ros.h"
#include "uml_hri_nerve_pick_and_place/AddTwoInts.h"

bool add(uml_hri_nerve_pick_and_place::AddTwoInts::Request &req, uml_hri_nerve_pick_and_place::AddTwoInts::Response &res) {
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", req.a, req.b);
  ROS_INFO("sending back response: [%ld]", res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
