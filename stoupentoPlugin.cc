#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include "stoupento_mesurement.pb.h"
#include "controlCommand.pb.h"
#include <sdf/sdf.hh>
#include <functional>
#include <iostream>
#include <string>
#include <algorithm>

namespace gazebo
{
   typedef const boost::shared_ptr<const stoupentoPlugin_msgs::msgs::ControlCommand> ControlCommandPtr;
class EncoderPlugin : public ModelPlugin
{
  private: transport::NodePtr node;
  private: transport::PublisherPtr publisher;
  private: physics::ModelPtr model;
  private: physics::JointPtr joint1L;
  private: physics::JointPtr joint1R;
  private: physics::LinkPtr wheelL;
  private: physics::LinkPtr wheelR;
  private: event::ConnectionPtr updateConnection;
  private: transport::SubscriberPtr commandSubscriber;
  private: transport::SubscriberPtr imuSubscriber;
  private: double u0L,u0R;
  private: ignition::math::Vector3d bodyLinAcc,bodyAngVel;

    public: EncoderPlugin() : ModelPlugin()
            {

            }
  public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  { 
    this->model = _model;
    this->model = _model;
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());
    this->publisher  = node->Advertise<stoupentoPlugin_msgs::msgs::StoupentoMesurement>("~/stoupentoMesurement");
    this->commandSubscriber = node->Subscribe("~/controlCommand",&EncoderPlugin::callback,this);
    this->imuSubscriber = node->Subscribe("~/Stoupento/Stoupento/mid_link/body_imu/imu",&EncoderPlugin::IMUcallback,this);
    this->u0L =0;
    this->u0R = 0;
    this->bodyLinAcc = ignition::math::Vector3d(0,0,0);
    this->bodyAngVel = ignition::math::Vector3d(0,0,0);

     if (!_sdf->HasElement("wheelL") || !_sdf->HasElement("wheelR") || !_sdf->HasElement("joint1L") || !_sdf->HasElement("joint1R") ){
    gzerr << "Plugin missing wheel element\n";
     }
    this->joint1L = _model->GetJoint(_sdf->GetElement("joint1L")->Get<std::string>());
    this->joint1R = _model->GetJoint(_sdf->GetElement("joint1R")->Get<std::string>());
    this->wheelL = _model->GetLink(_sdf->GetElement("wheelL")->Get<std::string>());
    this->wheelR = _model->GetLink(_sdf->GetElement("wheelR")->Get<std::string>());

    if (!this->joint1L || !this->joint1R || !this->wheelL || !this->wheelR){
      gzerr << "Unable to find joint or wheel elenet\n";}

       this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&EncoderPlugin::OnUpdate, this));
  }

      public: void OnUpdate()
    {
        this->sendMes();
    }
    public: void Init(){
    }

      public: void callback(ControlCommandPtr &msg)
  {
    this->u0L =msg->u0l();
    this->u0R = msg->u0r();
  }

       public: void IMUcallback(ConstIMUPtr &msg)
  {
    this->bodyLinAcc =ConvertIgn(msg->linear_acceleration());
    this->bodyAngVel =ConvertIgn(msg->angular_velocity());

  }

      public: void Reset(){
    stoupentoPlugin_msgs::msgs::StoupentoMesurement msg;

    this->u0L =0;
    this->u0R = 0;
    this->bodyLinAcc = ignition::math::Vector3d(0,0,0);
    this->bodyAngVel = ignition::math::Vector3d(0,0,0);

    //
    msg.set_left_wheel_vel(0);
    msg.set_left_wheel_ang(0);
    //
    msg.set_right_wheel_vel(0);
    msg.set_right_wheel_ang(0);
    //
    msg.set_left_joint_vel(0);
    msg.set_left_joint_ang(0);
    //
    msg.set_right_joint_vel(0);
    msg.set_right_joint_ang(0);
    //
    msg.set_left_wheel_torque(this->u0L);
    msg.set_right_wheel_torque(this->u0R);
    //
   msgs::Set(msg.mutable_linear_acc(),this->bodyLinAcc);
   msgs::Set(msg.mutable_angular_vel(),this->bodyAngVel);
    
    this->publisher->Publish(msg);

    }

    private: void sendMes(){
    stoupentoPlugin_msgs::msgs::StoupentoMesurement msg;
    double v0L, a0L, v0R, a0R, v1L, a1L, v1R, a1R;

    v0L = -this->wheelL->RelativeAngularVel().Z();
    a0L = (this->wheelL->WorldPose().Rot()*this->joint1L->GetParent()->WorldPose().Rot().Inverse()).Inverse().Yaw();
	//
    v0R = -this->wheelR->RelativeAngularVel().Z();
    a0R = (this->wheelR->WorldPose().Rot()*this->joint1R->GetParent()->WorldPose().Rot().Inverse()).Inverse().Yaw();
	//
    v1L = -this->joint1L->GetParent()->WorldAngularVel().Y();
    a1L = this->joint1L->Position();
	//
    v1R = -this->joint1R->GetParent()->WorldAngularVel().Y();
    a1R = this->joint1R->Position();



    msg.set_left_wheel_vel(v0L);
    msg.set_left_wheel_ang(a0L);
	//
    msg.set_right_wheel_vel(v0R);
    msg.set_right_wheel_ang(a0R);
        //
    msg.set_left_joint_vel(v1L);
    msg.set_left_joint_ang(a1L);
        //
    msg.set_right_joint_vel(v1R);
    msg.set_right_joint_ang(a1R);
	//
   msg.set_left_wheel_torque(this->u0L);
   msg.set_right_wheel_torque(this->u0R);
	//

   msgs::Set(msg.mutable_linear_acc(),this->bodyLinAcc);
   msgs::Set(msg.mutable_angular_vel(),this->bodyAngVel);

      this->joint1L->SetForce(0,-this->u0L);
      this->joint1R->SetForce(0,-this->u0R);
    this->publisher->Publish(msg);

    }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(EncoderPlugin)
}
