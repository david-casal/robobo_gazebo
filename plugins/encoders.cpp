#ifndef _ENCODERS_HH_
#define _ENCODERS_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <std_msgs/Int16.h>
#include <gazebo_msgs/LinkStates.h>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"
#include "robobo_msgs/Wheels.h"

namespace gazebo
{

    class Encoders : public ModelPlugin
    {

        public: Encoders() {}

        private: ros::Publisher pubWheels;
        private: ros::Publisher pubPan;
        private: ros::Publisher pubTilt;
        private: robobo_msgs::Wheels msgWheels;
        private: std_msgs::Int16 msgPan;
        private: std_msgs::Int16 msgTilt;
        private: int RWPos;
        private: int RWVel;
        private: int LWPos;
        private: int LWVel;
        private: int pan;
        private: int tilt;

        //public: physics::JointControllerPtr jointController;

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

            // Subscribe to topic
            ros::SubscribeOptions so = ros::SubscribeOptions::create<gazebo_msgs::LinkStates>("/gazebo/link_states",1,boost::bind(&Encoders::Callback, this, _1),ros::VoidPtr(), &this->rosQueue);

            // Create topics to publish
            this->pubWheels = this->rosNode->advertise<robobo_msgs::Wheels>("/" + this->model->GetName() + "/wheels", 1);
            this->pubPan = this->rosNode->advertise<std_msgs::Int16>("/" + this->model->GetName() + "/pan", 1);
            this->pubTilt = this->rosNode->advertise<std_msgs::Int16>("/" + this->model->GetName() + "/tilt", 1);

            // Declare the node as a subscriber
            this->rosSub = this->rosNode->subscribe(so);

            this->rosQueueThread = std::thread(std::bind (&Encoders::QueueThread, this));
        }

        public: void Callback(const gazebo_msgs::LinkStates::ConstPtr& msg)
        {
            // Read and transform values to degrees of right wheel joint
            RWPos = int (round(this->model->GetJoint("right_motor")->GetAngle(0).Radian() * 180 / M_PI));
            RWVel = int (round(this->model->GetJoint("right_motor")->GetVelocity(0) * 180 / M_PI));

            // Read and transform values to degrees of left wheel joint
            LWPos = int (round(this->model->GetJoint("right_motor")->GetAngle(0).Radian() * 180 / M_PI));
            LWVel = int (round(this->model->GetJoint("right_motor")->GetVelocity(0) * 180 / M_PI));

            // Save data in Wheels msg
            this->msgWheels.wheelPosR.data = RWPos;
            this->msgWheels.wheelSpeedR.data = RWVel;
            this->msgWheels.wheelPosL.data = LWPos;
            this->msgWheels.wheelSpeedL.data = LWVel;

            // Publish msg in topic
            this->pubWheels.publish(this->msgWheels);

            // Read an transform values to degrees of pan and tilt
            pan = int (round(this->model->GetJoint("pan_motor")->GetAngle(0).Radian() * 180 / M_PI));
            tilt = int (round(this->model->GetJoint("tilt_motor")->GetAngle(0).Radian() * 180 / M_PI));

            // Save data in msg
            this->msgPan.data = pan;
            this->msgTilt.data = tilt;

            // Publish msg in topic
            this->pubPan.publish(this->msgPan);
            this->pubTilt.publish(this->msgTilt);
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

        //public: event::ConnectionPtr updateConnection;

        /// \brief A ROS callback queue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(Encoders)
}
#endif
