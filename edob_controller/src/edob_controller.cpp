//  November/2019, Auterion, Jaeyoung Lim, jaeyoung@auterioncom

#include "edob_controller/edob_controller.h"

using namespace Eigen;
using namespace std;
//Constructor
ErrorDisturbanceObserverCtrl::ErrorDisturbanceObserverCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  geometric_controller_(nh, nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &ErrorDisturbanceObserverCtrl::CmdLoopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &ErrorDisturbanceObserverCtrl::StatusLoopCallback, this); // Define timer for constant loop rate

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

  q_.resize(3); // DOP Internal state for QPn^-1(s)
  p_.resize(3); // DOP Internal state for Q(s)
  r_.resize(3);
  for (int i = 0; i < 3; ++i) {
    q_.at(i) << 0.0, 0.0;
    p_.at(i) << 0.0, 0.0;
    r_.at(i) << 0.0, 0.0;
  }

  a0 << a0_x, a0_y, a0_z;
  a1 << a1_x, a1_y, a1_z;

  a_fb << 0.0, 0.0, 0.0;
  a_dob << 0.0, 0.0, 0.0;

}
ErrorDisturbanceObserverCtrl::~ErrorDisturbanceObserverCtrl() {
  //Destructor
}

void ErrorDisturbanceObserverCtrl::CmdLoopCallback(const ros::TimerEvent& event){
  // /// Compute BodyRate commands using disturbance observer
  // /// From Hyuntae Kim

  Eigen::Vector3d a_dob, pos_error, current_pos, vel_error, current_vel, reference, current_bodyrate;
  Eigen::Vector4d current_att;


  geometric_controller_.getErrors(pos_error, vel_error);
  geometric_controller_.getStates(current_pos, current_att, current_vel, current_bodyrate);
  geometric_controller_.getTargets(reference);
    
  a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; //feedforward term for trajectory error
  a_dob = ErrorDisturbanceObserver(reference, current_pos, a_fb - a_dob);
  a_des = a_fb - a_dob - g_;

  geometric_controller_.setFeedthrough(true);
  geometric_controller_.setDesiredAcceleration(a_des);

}

void ErrorDisturbanceObserverCtrl::StatusLoopCallback(const ros::TimerEvent& event){

}

Eigen::Vector3d ErrorDisturbanceObserverCtrl::ErrorDisturbanceObserver(Eigen::Vector3d reference, Eigen::Vector3d output, Eigen::Vector3d acc_setpoint){

  Eigen::Vector3d acc_input, yq, yp, yr, d_hat;
  double control_dt = 0.01;

  for(int i = 0; i < acc_input.size(); i++){
    //Update dob states
    // Q(s) Update
    p_.at(i)(0) = p_.at(i)(0) + p_.at(i)(1) * control_dt;
    p_.at(i)(1) = (-a0(i) * control_dt / std::pow(tau(i),2)) * p_.at(i)(0) + (1 - a1(i) * control_dt / tau(i)) *p_.at(i)(1) + control_dt * acc_setpoint(i);
    
    // QPn^-1(s) Update
    q_.at(i)(0) = q_.at(i)(0) + control_dt * q_.at(i)(1);
    q_.at(i)(1) = (-a0(i)/std::pow(tau(i), 2)) * control_dt * q_.at(i)(0) + (1 - a1(i) * control_dt / tau(i)) * q_.at(i)(1) + control_dt * output(i);
    
    // Feedforward QPn^-1(s) Update
    r_.at(i)(0) = r_.at(i)(0) + control_dt * r_.at(i)(1);
    r_.at(i)(1) = (-a0(i)/std::pow(tau(i), 2)) * control_dt * r_.at(i)(0) + (1 - a1(i) * control_dt / tau(i)) * r_.at(i)(1) + control_dt * reference(i);
 
    //Calculate outputs
    yp(i) = (a0(i) / pow(tau(i), 2)) * p_.at(i)(0);
    yq(i) = (-a1(i)*a0(i) / std::pow(tau(i), 3))*q_.at(i)(0) - (std::pow(a0(i),2) / std::pow(tau(i), 4)) * q_.at(i)(1) + a0(i) / pow(tau(i),2) * output(i);
    yr(i) = (-a1(i)*a0(i) / std::pow(tau(i), 3))*r_.at(i)(0) - (std::pow(a0(i),2) / std::pow(tau(i), 4)) * r_.at(i)(1) + a0(i) / pow(tau(i),2) * reference(i);

    d_hat(i) = yq(i) - yp(i) - yr(i); // We subtract yr as it needs to be added to a_fb
    acc_input(i) = d_hat(i);
  }

  return acc_input;
}