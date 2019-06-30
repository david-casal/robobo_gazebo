#ifndef _TILT_MOTOR_HH_
#define _TILT_MOTOR_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/math/gzmath.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"

namespace gazebo
{

    class TiltMotor : public ModelPlugin
    {

        public: TiltMotor() {}

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

            // Create node handler
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create topic to subscribe
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64>("/" + this->model->GetName() + "/tilt_motor",1,boost::bind(&TiltMotor::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);

            // Declare the node as a subscriber
            this->rosSub = this->rosNode->subscribe(so);

            this->rosQueueThread = std::thread(std::bind (&TiltMotor::QueueThread, this));

            //Load sdf parameters TRABALLO FUTURO OLLO!!!!!!!!!
//            this->sdf = _sdf;
//            if (!this->sdf->HasElement("jointName"))
//            {
//            ROS_ERROR_NAMED("motor", "Motor plugin need <jointName>");
//            }
//            else
//            this->jointName = this->sdf->Get<std::string>("jointName");
//            std::cerr << "\nComprobacion de funcinamento" << this->jointName << "\n";

            //Set Tilt initial position
//            this->model->GetJoint("tilt_motor")->SetPosition(0, 1.22);
//            this->model->GetJoint("tilt_motor")->SetLowStop(0, 1.22);
//            this->model->GetJoint("tilt_motor")->SetHighStop(0, 1.22);

            //QUITAR ESTO
            this->model->GetJoint("tilt_motor")->SetPosition(0, 0.0872665);
            this->model->GetJoint("tilt_motor")->SetLowStop(0, 0.0872665);
            this->model->GetJoint("tilt_motor")->SetHighStop(0, 0.0872665);
        }

        public: void SetVelocity(const double &_vel)
        {
            // Use no more than 0.22 Nm to achieve the target velocity
            this->model->GetJoint("tilt_motor")->SetParam("fmax", 0, 0.22);

            // Maximum joint velocity (rad/s)
            // Tilt motor rated speed is 12 rpm
//            double rads;
//            rads=_vel;
//            if (_vel < -100)
//            {
//                rads = -0.681;
//            }
//            else if (_vel > 100)
//            {
//                rads = 0.681;
//            }
//            else if (_vel == 0)
//            {
//                rads = 0;
//            }
//            else
//                if (_vel > 0)
//                {
//                    rads = 0.0061 * _vel + 0.071;
//                }
//                else
//                    rads = 0.0061 * _vel - 0.071;

            //Comprobacion para a memoria solo
            double lp = _vel;
            float d = 90;
            common::Time time;
            double speedGazebo = (d*(1.409E-05 * lp *lp*lp - 2.598E-03 * lp *lp + 0.4809 * lp + 3.182) / (d-(
                     -8.84E-05 * lp *lp*lp + 1.233E-02 * lp *lp + -0.5495 * lp + 4.738)) ) * 3.14159 / 180;
             time = this->model->GetWorld()->GetSimTime();

            // Set joint position and set joint's target velocity
            this->model->GetJoint("tilt_motor")->SetLowStop(0, 0.0873);
            this->model->GetJoint("tilt_motor")->SetHighStop(0, 1.9199);
//            this->model->GetJoint("tilt_motor")->SetParam("vel", 0, rads);


            //Sacar esto
            float pos = this->model->GetJoint("tilt_motor")->GetAngle(0).Radian() + d * 3.14159/180;
            while (this->model->GetJoint("tilt_motor")->GetAngle(0).Radian() < pos)
            {
                this->model->GetJoint("tilt_motor")->SetParam("vel", 0, speedGazebo);
            }
            this->model->GetJoint("tilt_motor")->SetParam("vel", 0, double (0.0));

            //Quitar esto
            common::Time endtime = this->model->GetWorld()->GetSimTime() - time;
            std::cerr << "\nLevoulle este tempo =====>  " << endtime;
            this->model->GetJoint("tilt_motor")->SetPosition(0, 0.0872665);
            this->model->GetJoint("tilt_motor")->SetLowStop(0, 0.0872665);
            this->model->GetJoint("tilt_motor")->SetHighStop(0, 0.0872665);

            // Fix joint position when velocity is 0
//            gazebo::math::Angle stop_pos = this->model->GetJoint("tilt_motor")->GetAngle(0);
//            if (_vel==0)
//            {
//               this->model->GetJoint("tilt_motor")->SetLowStop(0, stop_pos);
//               this->model->GetJoint("tilt_motor")->SetHighStop(0, stop_pos);
//            }
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

        //Fase de pruebas TRABALLO FUTURO!!!!!
//        private: sdf::ElementPtr sdf;
//        private: std::string jointName;

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(TiltMotor)
}
#endif