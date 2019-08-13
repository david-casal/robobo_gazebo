/* This plugin is used to publish the encoders values of the Gazebo model of Robobo and to create the ResetWheels service to reset wheel position values
/** \author David Casal. */

#ifndef _ENCODERS_HH_
#define _ENCODERS_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int16.h"
#include "gazebo_msgs/LinkStates.h"
#include "robobo_msgs/Wheels.h"
#include "robobo_msgs/ResetWheels.h"

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
        private: int r_reset = 0;
        private: int l_reset = 0;
        private: int pan;
        private: int tilt;

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

            // Subscribe to Gazebo link states topic
            ros::SubscribeOptions so = ros::SubscribeOptions::create<gazebo_msgs::LinkStates>("/gazebo/link_states",1,boost::bind(&Encoders::Callback, this, _1),ros::VoidPtr(), &this->rosQueue);

            // Create topics to publish
            this->pubWheels = this->rosNode->advertise<robobo_msgs::Wheels>("/" + this->model->GetName() + "/wheels", 1);
            this->pubPan = this->rosNode->advertise<std_msgs::Int16>("/" + this->model->GetName() + "/pan", 1);
            this->pubTilt = this->rosNode->advertise<std_msgs::Int16>("/" + this->model->GetName() + "/tilt", 1);

            // Declare the node as a subscriber
            this->rosSub = this->rosNode->subscribe(so);

            this->rosQueueThread = std::thread(std::bind (&Encoders::QueueThread, this));

             // Create ResetWheels service
            this->resetService = this->rosNode->advertiseService<robobo_msgs::ResetWheels::Request,
                robobo_msgs::ResetWheels::Response>("/" + this->model->GetName() + "/resetWheels",
                    boost::bind(&Encoders::CallbackResetWheels, this, _1, _2));
        }

        public: void Callback(const gazebo_msgs::LinkStates::ConstPtr& msg)
        {
            // Read and transform values to degrees of right wheel joint
            RWPos = int (round(this->model->GetJoint("right_motor")->GetAngle(0).Degree()) - this->r_reset);
            RWVel = int (round(this->model->GetJoint("right_motor")->GetVelocity(0) * 180 / M_PI));

            // Read and transform values to degrees of left wheel joint
            LWPos = int (round(this->model->GetJoint("left_motor")->GetAngle(0).Degree()) - this->l_reset);
            LWVel = int (round(this->model->GetJoint("left_motor")->GetVelocity(0) * 180 / M_PI));

            // Save data in Wheels msg
            this->msgWheels.wheelPosR.data = RWPos;
            this->msgWheels.wheelSpeedR.data = RWVel;
            this->msgWheels.wheelPosL.data = LWPos;
            this->msgWheels.wheelSpeedL.data = LWVel;

            // Publish msg in topic
            this->pubWheels.publish(this->msgWheels);

            // Read position values of pan and tilt
            pan = int (round(this->model->GetJoint("pan_motor")->GetAngle(0).Degree()));
            tilt = int (round(this->model->GetJoint("tilt_motor")->GetAngle(0).Degree()));

            // Save data in msg
            this->msgPan.data = pan;
            this->msgTilt.data = tilt;

            // Publish msg in topic
            this->pubPan.publish(this->msgPan);
            this->pubTilt.publish(this->msgTilt);
        }

        public: bool CallbackResetWheels(robobo_msgs::ResetWheels::Request &req, robobo_msgs::ResetWheels::Response &res)
        {
            this->r_reset = round(this->model->GetJoint("right_motor")->GetAngle(0).Degree());
            this->l_reset = round(this->model->GetJoint("left_motor")->GetAngle(0).Degree());
            return true;
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

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;

        /// \brief A ROS callback queue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

        /// \brief A ROS service server
        private: ros::ServiceServer resetService;

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(Encoders)
}
#endif
