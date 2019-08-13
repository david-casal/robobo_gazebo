/* This plugin is used to create the MoveWheels service to move wheels motors in the Gazebo model of Robobo
/** \author David Casal. */

#ifndef _MOVE_WHEELS_HH_
#define _MOVE_WHEELS_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "robobo_msgs/MoveWheels.h"


namespace gazebo
{

    class MoveWheels : public ModelPlugin
    {

        public: MoveWheels() {}

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

            // Create MoveWheels service
            this->controlService = this->rosNode->advertiseService<robobo_msgs::MoveWheels::Request,
                robobo_msgs::MoveWheels::Response>("/" + this->model->GetName() + "/moveWheels",
                    boost::bind(&MoveWheels::Callback, this, _1, _2));

        }

        public: bool Callback(robobo_msgs::MoveWheels::Request &req, robobo_msgs::MoveWheels::Response &res)
        {
            // Save service parameter values
            if (req.lspeed.data < -100){

                this->lwp = -100;
            }
            else if (req.lspeed.data > 100)
            {
                this->lwp = 100;
            }
            else
            {
                this->lwp = req.lspeed.data;
            }
            if (req.rspeed.data < -100)
            {
                this->rwp = -100;
            }
            else if (req.rspeed.data > 100)
            {
                this->rwp = 100;
            }
            else
            {
                this->rwp = req.rspeed.data;
            }
            this->time = req.time.data / 1000;
            this->MoveWheelsThread = std::thread(std::bind (&MoveWheels::Handle_MoveWheels, this));
            this->MoveWheelsThread.detach();
            return true;
        }

        private: void Handle_MoveWheels ()
        {
            double lspeedGazebo;
            double rspeedGazebo;

            // Use enough force to achieve the target velocity
            this->model->GetJoint("right_motor")->SetParam("fmax", 0, 0.5);
            this->model->GetJoint("left_motor")->SetParam("fmax", 0, 0.5);

            // Calculate joints target velocity
            lspeedGazebo = ((-4.625E-05 * pow(abs(this->lwp),3) + 5.219E-03 * pow(abs(this->lwp),2) + 6.357 * abs(this->lwp) + 5.137E+01) + (
                     -3.253E-04 * pow(abs(this->lwp),3) + 4.285E-02 * pow(abs(this->lwp),2) + -2.064 * abs(this->lwp) - 1.770E+01) / this->time) * M_PI / 180;
            if (this->lwp < 0)
            {
                lspeedGazebo = -lspeedGazebo;
            }
            else if (this->lwp == 0)
            {
                lspeedGazebo = 0;
            }
            rspeedGazebo = ((-4.625E-05 * pow(abs(this->rwp),3) + 5.219E-03 * pow(abs(this->rwp),2) +
                    6.357 * abs(this->rwp) + 5.137E+01) + (-3.253E-04 * pow(abs(this->rwp),3) + 4.285E-02 *
                        pow(abs(this->rwp),2) + -2.064 * abs(this->rwp) - 1.770E+01) / this->time) * M_PI / 180;
            if (this->rwp < 0)
            {
                rspeedGazebo = - rspeedGazebo;
            }
            else if (this->rwp == 0 )
            {
                rspeedGazebo = 0;
            }

            // Set joints velocity
            this->model->GetJoint("left_motor")->SetParam("vel", 0, lspeedGazebo);
            this->model->GetJoint("right_motor")->SetParam("vel", 0, rspeedGazebo);
            usleep(this->time * 1E6);
            this->model->GetJoint("left_motor")->SetParam("vel", 0, double(0));
            this->model->GetJoint("right_motor")->SetParam("vel", 0, double(0));
        }

        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A thread the keeps running velocity setting
        private: std::thread MoveWheelsThread;

        /// \brief A ROS service server
        private: ros::ServiceServer controlService;

        // MoveWheels parameters
        private: int lwp;
        private: int rwp;
        private: float time;

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(MoveWheels)
}
#endif
