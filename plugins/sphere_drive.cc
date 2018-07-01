#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

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

      // Initialize ros, if it has not already been initialized.
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
      
      // Create a named topic for twist, and subscribe to it.
      ros::SubscribeOptions so_twist =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/" + this->model->GetName() + "/twist_cmd",
            1,
            boost::bind(&SphereDrive::OnRosMsgTwist, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSubTwist = this->rosNode->subscribe(so_twist);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&SphereDrive::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A twist message that is used to set the angular 
    /// velocity of the Sphere.
    public: void OnRosMsgTwist(const geometry_msgs::Twist::ConstPtr& _msg)
    {
      std::cerr << "\n[" << model->GetName() << "] vel[x, y] set to " << 
        _msg->angular.x << ", " << _msg->angular.y << "\n";
      this->model->SetAngularVel(ignition::math::Vector3d(_msg->angular.x, 
        _msg->angular.y, 0)); 
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
    
    /// \brief A ROS subscriber for twist
    private: ros::Subscriber rosSubTwist;
    
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SphereDrive)
}

