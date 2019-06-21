#include <cnoid/SimpleController>
#include <cnoid/RateGyroSensor>
#include <cnoid/AccelerationSensor>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <aizuspider_description/Control.h>

#include <ros/ros.h>

#include <iostream>

#define DEBUG 0

namespace aizu = aizuspider_description;
using namespace cnoid;

class InvPendulumControllerImpl
{
  double dt;
  double prev_theta_sensor;
  double pitch_comple;
  double _q;

public:
  InvPendulumControllerImpl(double _dt, double _theta_initial) {
    dt = _dt;
    pitch_comple = prev_theta_sensor = _theta_initial;
    _q = 0.0;
  }
  double control(double _dq, const Vector3d &_omega, const Vector3d &_accel,
                 ros::ServiceClient &srv)
  {
    ///
    double pitch_acc = atan2(-_accel.x(), _accel.z());
#if DEBUG
    std::cout << "theta_acc: " << pitch_acc;
    std::cout << ", x " << _accel.x();
    std::cout << ", z " << _accel.z() << std::endl;
#endif
    /// Complementary Filter
    const double Kconst        = 0.9;
    const double comple_alpha_ = Kconst / (Kconst + dt);

    double theta_sensor, d_theta_sensor, d_phi_sensor;

    pitch_comple = comple_alpha_ * (pitch_comple + _omega.y() * dt) +
      //(1 - comple_alpha_) * theta_imu; // Complementary Filter
      (1 - comple_alpha_) * pitch_acc; // Complementary Filter
    theta_sensor = pitch_comple;
    //theta_sensor = theta_imu;

    d_theta_sensor    = (theta_sensor  - prev_theta_sensor) / dt;
    d_phi_sensor      = d_theta_sensor - _dq;
    prev_theta_sensor = theta_sensor;

    double rl_torque = 0.0;
    {
      ///
      /// service-call for act theta_sensor, d_theta_sensor, q = siguma dq,  d_phi_act = dq
      ///
      aizu::Control msg;

      _q = _q + _dq * dt;
      msg.request.state.resize(4);
      msg.request.state[0] = theta_sensor;
      msg.request.state[1] = d_theta_sensor;
      msg.request.state[2] =  _q;
      msg.request.state[3] = _dq;

      if (srv.call(msg) ) {
        rl_torque = msg.response.action[0];
      } else {
        ROS_ERROR("Failed to call service ...");
        // exit
      }
    }

    /// using sensor value
    //A = [ 0.0, 1.0, 0.0; 59.8708, 0, 0; -468.554, 0.0, 0.0]
    //B = [0.0; -1.3281; 12.5678]
    //Q = diag([100 100 0.4]);
    //r =  0.002
    //[g,x,l] = lqr(A,B,Q,r)
    double u_torque = (theta_sensor   * -1119.443) +
                      (d_theta_sensor * -397.636) +
                      ((d_phi_sensor - 0) * -14.142);

    /// torque limitation
    if (u_torque >  500) u_torque =  500;
    if (u_torque < -500) u_torque = -500;
#if DEBUG
    std::cout << "sn[ " << theta_sensor;
    std::cout << " " << d_theta_sensor;
    std::cout << " " << d_phi_sensor;
    std::cout << " ] " << std::endl;
    std::cout << "u = " << u_torque << std::endl;
#endif

    return u_torque;
  }
};

class InvPendulumController : public SimpleController
{
  double dt;

  BodyPtr ioBody;
  Link *wheel_l;
  Link *wheel_r;
  Link *body;
  std::string wheel_l_name;
  std::string wheel_r_name;
  std::string body_name;

  // bush ...
  Link *bush_z;
  Link *bush_y;
  double z_pgain, z_dgain;
  double y_pgain, y_dgain;
  double z_qref, z_qold;
  double y_qref, y_qold;

  /// sensor device
  RateGyroSensor *gyro_dev;
  AccelerationSensor *accel_dev;
  std::string gyro_name;
  std::string accel_name;

  Quaterniond gyro_rotation;
  Quaterniond accel_rotation;

  double prev_theta;
  Quaterniond prev_wheel;

  InvPendulumControllerImpl *impl;
  double prev_theta_sensor;
  double pitch_comple;

  ///
  long counter;

  /// ROS
  ros::NodeHandle *nh;
  ros::Subscriber sub;
  ros::ServiceClient srv;
  //
  double vx_ref;
  double wz_ref;

public:
  void callback (const geometry_msgs::Twist::ConstPtr &msg)
  {
    vx_ref = msg->linear.x;
    wz_ref = msg->angular.z;
    //ROS_WARN("cb: %f %f", vx_ref, wz_ref);
  }
  virtual bool initialize(SimpleControllerIO* io) override
  {
    ioBody = io->body();
    dt = io->timeStep();

    nh = new ros::NodeHandle(ioBody->name());
    sub = nh->subscribe("cmd_vel", 10, &InvPendulumController::callback, this);
    vx_ref = 0.0;
    wz_ref = 0.0;

    wheel_l_name = "WHEEL_L";
    wheel_r_name = "WHEEL_R";
    body_name    = "BODY";

    gyro_name  = "RATE_GYRO_SENSOR";
    accel_name = "ACCELERATION_SENSOR";

    //std::string option = io->optionString();
    for(auto& option : io->options()) {
      int len = option.size();
      if (len > 4) {
        std::string sub = option.substr(0, 5);
        if (sub == "body:") {
          body_name = option.substr(5);
        } else if (sub == "gyro:") {
          gyro_name = option.substr(5);
        }
        if (len > 5) {
          std::string sub = option.substr(0, 6);
          if (sub == "accel:") {
            accel_name = option.substr(6);
          }
          if (len > 7) {
            std::string sub = option.substr(0, 8);
            if (sub == "wheel_l:") {
              wheel_l_name = option.substr(8);
            } else if (sub == "wheel_r:") {
              wheel_r_name = option.substr(8);
            }
          }
        }
      }
    }

    std::cout << "WHEEL_L: " << wheel_l_name << std::endl;
    std::cout << "WHEEL_R: " << wheel_r_name << std::endl;
    std::cout << "BODY:  " << body_name << std::endl;
    std::cout << "GYRO:  " << gyro_name << std::endl;
    std::cout << "ACCEL: " << accel_name << std::endl;

    io->enableInput(io->body()->rootLink(), LINK_POSITION);
    body  = ioBody->link(body_name);
    if (!body) {
      std::cout << "BODY: " << body_name << " not found!" << std::endl;
      return false;
    }
    wheel_l = ioBody->link(wheel_l_name);
    if (!wheel_l) {
      std::cout << "WHEEL_L: " << wheel_l_name << " not found!" << std::endl;
      return false;
    }
    wheel_r = ioBody->link(wheel_r_name);
    if (!wheel_r) {
      std::cout << "WHEEL_R: " << wheel_r_name << " not found!" << std::endl;
      return false;
    }
    {
      wheel_l->setActuationMode(Link::JOINT_TORQUE);

      io->enableIO(wheel_l);
      io->enableInput(wheel_l, LINK_POSITION);
      io->enableInput(wheel_l, JOINT_VELOCITY);
    }
    {
      wheel_r->setActuationMode(Link::JOINT_TORQUE);

      io->enableIO(wheel_r);
      io->enableInput(wheel_r, LINK_POSITION);
      io->enableInput(wheel_r, JOINT_VELOCITY);
    }
    //// bush
    {
      Link* joint = ioBody->link("BUSH_Z");
      if (!!joint) {
        joint->setActuationMode(Link::JOINT_TORQUE);
        io->enableIO(joint);
        io->enableInput(joint, LINK_POSITION);
        io->enableInput(joint, JOINT_VELOCITY);
        bush_z = joint;
      } else {
        bush_z = NULL;
      }
    }
    {
      Link* joint = ioBody->link("BUSH_Y");
      if (!!joint) {
        joint->setActuationMode(Link::JOINT_TORQUE);
        io->enableIO(joint);
        io->enableInput(joint, LINK_POSITION);
        io->enableInput(joint, JOINT_VELOCITY);
        bush_y = joint;
      } else {
        bush_y = NULL;
      }
    }

    if (!!bush_z) {

      z_qold = z_qref = bush_z->q();
    }
    if (!!bush_y) {
      y_qold = y_qref = bush_y->q();
    }

    z_pgain = 200000;
    z_dgain = 1000;

    y_pgain = 5000;
    y_dgain = 200;

    //// control
    {
      Matrix3  m = body->rotation();
      Vector3d mz = Vector3d::UnitZ().cross(m.col(2));
      double theta_init = asin(mz.norm());
      if ( Vector3d::UnitY().dot(mz) < 0 ) {
        theta_init = -theta_init;
      }
      prev_theta = theta_init;
    }
    prev_wheel.setIdentity();

    impl = new InvPendulumControllerImpl(dt, prev_theta);
    pitch_comple      = prev_theta;
    prev_theta_sensor = prev_theta;

    /// sensor devices
    accel_dev = ioBody->findDevice<AccelerationSensor>(accel_name);
    gyro_dev  = ioBody->findDevice<RateGyroSensor>(gyro_name);

    if (!accel_dev) {
      std::cout << "ACCEL: " << accel_name << " not found!" << std::endl;
      return false;
    }
    if (!gyro_dev) {
      std::cout << "GYRO: " << gyro_name << " not found!" << std::endl;
      return false;
    }
    io->enableInput(accel_dev);
    io->enableInput(gyro_dev);

    accel_rotation = Quaterniond(accel_dev->localRotation());
    gyro_rotation  = Quaterniond(gyro_dev->localRotation());

    std::cout << "initialized[InvPendulumController]" << std::endl;

    /// service
    srv = nh->serviceClient<aizu::Control>("/chainerrl/control");
    ROS_WARN("waiting service /chainerrl/control");
    srv.waitForExistence();
    ROS_WARN("service /chainerrl/control found");
    ///

    counter = 0;
    return true;
  }
  void control_bush()
  {
    //// bush control
    if (!!bush_z)
    {
      double z_q  = bush_z->q();
      double z_dq = (z_q - z_qold) / dt;
      double z_u  = (z_qref - z_q) * z_pgain + (0.0 - z_dq) * z_dgain;
      bush_z->u() = z_u;

      z_qold = z_q;
    }
    if (!!bush_y)
    {
      double y_q  = bush_y->q();
      double y_dq = (y_q - y_qold) / dt;
      double y_u  = (y_qref - y_q) * y_pgain + (0.0 - y_dq) * y_dgain;
      bush_y->u() = y_u;

      y_qold = y_q;
    }
  }

  ///
  virtual bool control() override
  {
    control_bush();

    //// wait until accel sensor returns valid value
    if (counter++ < 2) {
      return true;
    }

    ///>>> storeing actual data
    /// actual theta
    double theta_act;
    {
      Vector3d p = body->translation();
      Matrix3  m = body->rotation();
      Vector3d mz = Vector3d::UnitZ().cross(m.col(2));
      theta_act = asin(mz.norm());
      if ( Vector3d::UnitY().dot(mz) < 0 ) {
        theta_act = -theta_act;
      }
    }

    /// actual d_phi
    double d_phi_act;
    {
      Vector3d wz = wheel_l->rotation().col(2);
      double wheel_act_v = asin(Vector3d::UnitZ().cross(wz).norm());
      Quaterniond wheel_act(wheel_l->rotation());

      Matrix3 dwheel = (prev_wheel.inverse() * wheel_act).toRotationMatrix();
      Vector3d dz = Vector3d::UnitZ().cross(dwheel.col(2));
      d_phi_act = asin(dz.norm()) / dt;
      if ( Vector3d::UnitY().dot(dz) < 0 ) {
        d_phi_act = -d_phi_act;
      }
      prev_wheel = wheel_act;
    }
    double d_theta_act = (theta_act - prev_theta) / dt;
    prev_theta = theta_act;
    ///<<< storeing actual data
#if DEBUG
    std::cout << "ac[ " << theta_act;
    std::cout << " " << d_theta_act;
    std::cout << " " << d_phi_act;
    std::cout << " ] " << std::endl;
#endif

#if 0
    double rl_torque = 0.0;
    {
      ///
      /// service-call for act theta_act, d_theta_act, phi_act = q, d_phi_act = dq
      ///
      aizu::Control msg;
      double  q = (wheel_l->q() + wheel_r->q())/2;
      double dq = (wheel_l->dq() + wheel_r->dq())/2;

      msg.request.state.resize(4);
      msg.request.state[0] = theta_act;
      msg.request.state[1] = d_theta_act;
      msg.request.state[2] = q;
      msg.request.state[3] = dq;

      if (srv.call(msg) ) {
        rl_torque = msg.response.action[0];
      } else {
        ROS_ERROR("Failed to call service ...");
        // exit
      }
    }
#endif

#if 0
    /// using actual value
    double u = (theta_act   * 443.9) +
                      (d_theta_act * 125.1) +
                      ((d_phi_act - 0)   * 6.325);
    /// apply caluclated torque to joint
    ioBody->joint(0)->u() = -u;
#endif

    /// sensor data
    Vector3d accel = accel_rotation * accel_dev->dv();
    Vector3d omega = gyro_rotation  * gyro_dev->w();

    double dq = (wheel_l->dq() + wheel_r->dq())/2;
    double u_torque = - impl->control(-dq + vx_ref, omega, accel, srv);

    double diff_tq = 5 * (omega.z() - wz_ref);

    double u_l = 0.5 * u_torque + 0.5 * diff_tq;
    double u_r = 0.5 * u_torque - 0.5 * diff_tq;
#if 0
    std::cout << "om_z: " << omega.z();
    std::cout << ", L: " << wheel_l->dq();
    std::cout << ", R: " << wheel_r->dq() << std::endl;
    std::cout << "u_l: " << u_l << ", u_r: " << u_r << std::endl;
#endif
    wheel_l->u() = u_l;
    wheel_r->u() = u_r;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(InvPendulumController)

/*
/usr/bin/c++ -DInvPendulumControllerSrv_EXPORTS -I/home/leus/choreonoid_ws/devel/include -I/home/leus/choreonoid_ws/devel/include/choreonoid-1.7 -I/usr/include/eigen3 -I/opt/ros/kinetic/include -O3 -DNDEBUG -fPIC -std=c++11 -c InvPendulumControllerSrv.cpp

/usr/bin/c++  -fPIC -O3 -DNDEBUG  -shared -Wl,-soname,InvPendulumControllerSrv.so InvPendulumControllerSrv.o -o InvPendulumControllerSrv.so -lroscpp -L/opt/ros/kinetic/lib
*/
