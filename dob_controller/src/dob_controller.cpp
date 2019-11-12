//  March/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "dob_controller/dob_controller.h"

using namespace Eigen;
using namespace std;
//Constructor
DisturbanceObserverCtrl::DisturbanceObserverCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  geometric_controller_(nh, nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &DisturbanceObserverCtrl::CmdLoopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &DisturbanceObserverCtrl::StatusLoopCallback, this); // Define timer for constant loop rate

   double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;

  nh_.param<double>("/dob_controller/Kp_x", Kpos_x_, 8.0);
  nh_.param<double>("/dob_controller/Kp_y", Kpos_y_, 8.0);
  nh_.param<double>("/dob_controller/Kp_z", Kpos_z_, 30.0);
  nh_.param<double>("/dob_controller/Kv_x", Kvel_x_, 2.0);
  nh_.param<double>("/dob_controller/Kv_y", Kvel_y_, 2.0);
  nh_.param<double>("/dob_controller/Kv_z", Kvel_z_, 10.0);
  nh_.param<double>("/dob_controller/dob/a0_x", a0_x, 10.0);
  nh_.param<double>("/dob_controller/dob/a0_y", a0_y, 10.0);
  nh_.param<double>("/dob_controller/dob/a0_z", a0_z, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_x", a1_x, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_y", a1_y, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_z", a1_z, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_y", a1_y, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_z", a1_z, 10.0);
  nh_.param<double>("/dob_controller/dob/tau_x", tau_x, 10.0);
  nh_.param<double>("/dob_controller/dob/tau_y", tau_y, 10.0);
  nh_.param<double>("/dob_controller/dob/tau_z", tau_z, 10.0);
  nh_.param<double>("/dob_controller/dob/max_dhat", dhat_max, 10.0);
  nh_.param<double>("/dob_controller/dob/min_dhat", dhat_min, -10.0);

  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_z_, -Kvel_z_;
  tau << tau_x, tau_y, tau_z;

  q_.resize(3);
  p_.resize(3);
  for (int i = 0; i < 3; ++i) {
    q_.at(i) << 0.0, 0.0;
    p_.at(i) << 0.0, 0.0;
  }
  a0 << a0_x, a0_y, a0_z;
  a1 << a1_x, a1_y, a1_z;

  a_fb << 0.0, 0.0, 0.0;
  a_dob << 0.0, 0.0, 0.0;

}
DisturbanceObserverCtrl::~DisturbanceObserverCtrl() {
  //Destructor
}

void DisturbanceObserverCtrl::CmdLoopCallback(const ros::TimerEvent& event){
  // /// Compute BodyRate commands using disturbance observer
  // /// From Hyuntae Kim
  geometric_controller_.getErrors(pos_error, vel_error);
  
  a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; //feedforward term for trajectory error
  a_dob = DisturbanceObserver(pos_error, a_fb - a_dob);
  a_des = a_fb - a_dob - g_;

  geometric_controller_.setFeedthrough(true);
  geometric_controller_.setDesiredAcceleration(a_des);

}

void DisturbanceObserverCtrl::StatusLoopCallback(const ros::TimerEvent& event){

}

Eigen::Vector3d DisturbanceObserverCtrl::DisturbanceObserver(Eigen::Vector3d pos_error, Eigen::Vector3d acc_setpoint){

  Eigen::Vector3d acc_input, yq, yp, d_hat;
  double control_dt = 0.01;

  for(int i = 0; i < acc_input.size(); i++){
    //Update dob states
    p_.at(i)(0) = p_.at(i)(0) + p_.at(i)(1) * control_dt;
    p_.at(i)(1) = (-a0(i) * control_dt / std::pow(tau(i),2)) * p_.at(i)(0) + (1 - a1(i) * control_dt / tau(i)) *p_.at(i)(1) + control_dt * acc_setpoint(i);
    q_.at(i)(0) = q_.at(i)(0) + control_dt * q_.at(i)(1);
    q_.at(i)(1) = (-a0(i)/std::pow(tau(i), 2)) * control_dt * q_.at(i)(0) + (1 - a1(i) * control_dt / tau(i)) * q_.at(i)(1) + control_dt * pos_error(i);
    //Calculate outputs
    yp(i) = (a0(i) / pow(tau(i), 2)) * p_.at(i)(0);
    yq(i) = (-a1(i)*a0(i) / std::pow(tau(i), 3))*q_.at(i)(0) - (std::pow(a0(i),2) / std::pow(tau(i), 4)) * q_.at(i)(1) + a0(i) / pow(tau(i),2) * pos_error(i);
    d_hat(i) = yq(i) - yp(i);
    d_hat(i) = std::max( std::min( d_hat(i), dhat_max), dhat_min);
    acc_input(i) = d_hat(i);
  }

  return acc_input;
}