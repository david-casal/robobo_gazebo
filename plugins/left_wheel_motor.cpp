#ifndef _LEFT_WHEEL_MOTOR_HH_
#define _LEFT_WHEEL_MOTOR_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"

namespace gazebo
{

    class LeftMotor : public ModelPlugin
    {

        public: LeftMotor() {}


        public: physics::JointControllerPtr jointController;

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Safety check
            if (_model->GetJointCount() == 0)
            {
            std::cerr << "Invalid joint count, model not loaded\n";
            return;
            }

            this->model = _model;

            // Initialize ros
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }

            //Create node handler
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create topic to subscribe
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64>("/" + this->model->GetName() + "/left_wheel_motor",1,boost::bind(&LeftMotor::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);

            //Declare the node as a subscriber
            this->rosSub = this->rosNode->subscribe(so);

            this->rosQueueThread = std::thread(std::bind (&LeftMotor::QueueThread, this));
        }

        public: void SetVelocity(const double &_vel)
        {
            // Set enough force to achieve the target velocity
            this->model->GetJoint("left_motor")->SetParam("fmax", 0, 0.5);

            // Transform the velocity value from Robobo service moveWheels to rad/s
            double rads;
            if (_vel < -100)
            {
                rads = -11.9716;
            }
            else if (_vel > 100)
            {
                rads = 11.9716;
            }
            else if (_vel == 0)
            {
                rads = 0;
            }
            else
                rads = 0.11136 * _vel + 0.8356;

            // Set the joint's target velocity
            this->model->GetJoint("left_motor")->SetParam("vel", 0, rads);
        }

        // Callback function for each msg
        public: void OnRosMsg(const std_msgs::Float64ConstPtr &_msg)
        {
            this->SetVelocity(_msg->data);
        }

        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joint.
        private: physics::JointPtr joint;

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;

        public: event::ConnectionPtr updateConnection;

        /// \brief A ROS callback queue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(LeftMotor)
}
#endif
