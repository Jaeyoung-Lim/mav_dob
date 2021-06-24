//  November/2019, Auterion, Jaeyoung Lim, jaeyoung@auterioncom

#include "edob_controller/edob_controller.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"dob_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ErrorDisturbanceObserverCtrl edobcontroller(nh, nh_private);
  ros::spin();
  return 0;
}
