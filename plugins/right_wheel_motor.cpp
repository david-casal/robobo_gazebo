#ifndef _RIGHT_WHEEL_MOTOR_HH_
#define _RIGHT_WHEEL_MOTOR_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>

//Acordarse deste
#include <gazebo/common/common.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"

namespace gazebo
{

    class RightMotor : public ModelPlugin
    {

        public: RightMotor() {}

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
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64>("/" + this->model->GetName() + "/right_wheel_motor",1,boost::bind(&RightMotor::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);

            //Declare the node as a subscriber
            this->rosSub = this->rosNode->subscribe(so);

            this->rosQueueThread = std::thread(std::bind (&RightMotor::QueueThread, this));

        }

        public: void SetVelocity(const double &_vel)
        {
            // Use enough force to achieve the target velocity
            this->model->GetJoint("right_motor")->SetParam("fmax", 0, 0.5);

            // Transform the velocity value from Robobo service moveWheels to rad/s
//            double rads;
//            if (_vel < -100)
//            {
//                rads = -11.9716;
//            }
//            else if (_vel > 100)
//            {
//                rads = 11.9716;
//            }
//            else if (_vel == 0)
//            {
//                rads = 0;
//            }
//            else
//                rads = 0.11136 * _vel + 0.8356;
//
//            // Set the joint's target velocity
//            this->model->GetJoint("right_motor")->SetParam("vel", 0, rads);
            //proba de funcionamento
            this->model->GetJoint("left_motor")->SetParam("fmax", 0, 0.5);
            double lp = _vel;
            int t = 3;
            common::Time time = 3;
            common::Time end_time = this->model->GetWorld()->GetSimTime() + time;
            double speedGazebo = ((-4.625E-05 * lp *lp*lp + 5.219E-03 * lp *lp + 6.357 * lp + 5.137E+01) + (
                     -3.253E-04 * lp *lp*lp + 4.285E-02 * lp *lp + -2.064 * lp - 1.770E+01) / t) * 3.14159 / 180;
             std::cerr << "\nAqui funciona speed= " << end_time;
            while(this->model->GetWorld()->GetSimTime() < end_time)
            {
                this->model->GetJoint("right_motor")->SetParam("vel", 0, speedGazebo);

                this->model->GetJoint("left_motor")->SetParam("vel", 0, speedGazebo);
            }
            std::cerr << "\n Saleu do while \n";
//            this->model->GetJoint("right_motor")->SetParam("vel", 0, speedGazebo);
//            this->model->GetJoint("left_motor")->SetParam("vel", 0, speedGazebo);
//            usleep(t*1000000);
//            std::cerr << "\n Aqui tamen funciona Time =" << end_time;
            speedGazebo = 0;
            this->model->GetJoint("right_motor")->SetParam("vel", 0, speedGazebo);
            this->model->GetJoint("left_motor")->SetParam("vel", 0, speedGazebo);
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

        // This could be written in a header file and use it for the three plugin

        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joint.
        private: physics::JointPtr joint;

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;

        public: event::ConnectionPtr updateConnection;

        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(RightMotor)
}
#endif