#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class SphereDrive : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      std::cerr << "\nThe SphereDrive plugin is attached to model[" <<
        model->GetName() << "]\n";

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      //     std::bind(&SphereDrive::OnUpdate, this));

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }
      
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      
      // Create a named topic for velocity, and subscribe to it.
      ros::SubscribeOptions so_vel =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/vel_cmd",
            1,
            boost::bind(&SphereDrive::OnRosMsgVel, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSubVel = this->rosNode->subscribe(so_vel);
      
      // Spin up the queue helper thread.
      // this->rosQueueThread =
      //   std::thread(std::bind(&SphereDrive::QueueThread, this));

      // Create a named topic for yaw, and subscribe to it.
      ros::SubscribeOptions so_yaw =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/yaw_cmd",
            1,
            boost::bind(&SphereDrive::OnRosMsgYaw, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSubYaw = this->rosNode->subscribe(so_yaw);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&SphereDrive::QueueThread, this));
    }

    // Called by the world update start event
    // public: void OnUpdate()
    // {
      // Apply a small linear velocity to the model.
      // this->model->SetLinearVel(ignition::math::Vector3d(.1, 0, 0));
      // Apply a small angular velocity to the model.
      // this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
    // }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the angular 
    /// velocity of the Sphere.
    public: void OnRosMsgVel(const std_msgs::Float32ConstPtr &_msg)
    {
      std::cerr << "\n[" << model->GetName() << "] angular speed set to " << 
        _msg->data << "\n";
      double x_part, y_part;
      x_part = std::cos(this->yaw) * _msg->data;
      y_part = std::sin(this->yaw) * _msg->data;
      this->model->SetAngularVel(ignition::math::Vector3d(x_part, y_part, 0)); 
    }
    
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the yaw 
    /// of the Sphere.
    public: void OnRosMsgYaw(const std_msgs::Float32ConstPtr &_msg)
    {
      std::cerr << "\n[" << model->GetName() << "] yaw set to " << 
        _msg->data << "\n";
      double cx, cy, cz;
      gazebo::math::Pose pose;
      pose = this->model->GetWorldPose();
      math::Vector3 v(0, 0, 0);
      v = pose.pos;
      cx = v.x;
      cy = v.y;
      cz = v.z;
      this->model->SetWorldPose(ignition::math::Pose3d(cx, cy, cz, 
                                                  0, 0, _msg->data));
      this->yaw = _msg->data;
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // Current yaw
    private: double yaw;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber for velocity
    private: ros::Subscriber rosSubVel;
    
    /// \brief A ROS subscriber for yaw
    private: ros::Subscriber rosSubYaw;
    
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SphereDrive)
}

