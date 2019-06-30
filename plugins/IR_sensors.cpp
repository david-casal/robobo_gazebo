#ifndef _IR_SENSORS_HH_
#define _IR_SENSORS_HH_

#include <gazebo/gazebo.hh>
//#include <gazebo/physics/physics.hh>
//#include <gazebo/transport/transport.hh>
//#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
//#include <gazebo/physics/Joint.hh>
//#include <gazebo/physics/JointController.hh>
//#include <gazebo/physics/Model.hh>
//#include <gazebo/physics/PhysicsTypes.hh>
//#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/common/Plugin.hh"
//#include "gazebo/sensors/SensorTypes.hh"
//#include "gazebo/util/system.hh"

//#include <thread>
//#include "ros/ros.h"
#include "ros/callback_queue.h"
//#include "ros/subscribe_options.h"
//#include "std_msgs/Float64.h"
#include "robobo_msgs/IRs.h"

namespace gazebo
{

    class IRSensors : public SensorPlugin
    {

        public: IRSensors() {}

        public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
        {

            gazebo::sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();


            RaySensorPtr frontc = mgr->GetSensor("front-c");


            //Get the parent sensor
            this->parentSensor = _parent;
            //Make sure the parent sensor is valid
            if (!this->parentSensor)
            {
                gzerr << "Prueba de funcionamento -> ERROR \n";
                return;
            }

            this->world = physics::get_world(this->parentSensor->GetWorldName());


            this->pubWheels = this->rosNode->advertise<robobo_msgs::IRs>("/" + this->model->GetName() + "/wheels", 1);
            this->newLaserScansConnection = this->parentSensor->GetLaserShape()->ConnectNewLaserScans(boost::bind(&IRSensors::OnNewLaserScans, this));
        }

        private: virtual void OnNewLaserScans()
        {

        }

        private: physics::WorldPtr world;

        private: sensors::RaySensorPtr parentSensor;

        private: event::ConnectionPtr newLaserScansConnection;

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_SENSOR_PLUGIN(IRSensors)
}
#endif
