#ifndef _GAZEBO_UNDERWATER_DYNAMICS_HH_
#define _GAZEBO_UNDERWATER_DYNAMICS_HH_

#include <string>
#include <vector>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{

  class properties
  {
    public: properties() : volume(0) {}

    public: math::Vector3 cop;
    public: math::Vector3 cob;
    public: math::Vector3 cog;

    public: math::Vector3 tangential;
    public: math::Vector3 normal;
    public: math::Vector3 localAxis;

    public: math::Vector3 size;

    public: double volume;
    public: double area;
    public: double length;
    public: double breadth;
  };

  class UWDynamicsPlugin : public ModelPlugin
  {
    public: UWDynamicsPlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();
    protected: virtual void OnUpdate();

    public: void getProperties(physics::JointPtr joint, properties& ptr, math::Vector3 y_axis, math::Vector3 z_axis);
    public: math::Vector3 vectorize(math::Vector3 vector);

    protected: event::ConnectionPtr updateConnection;
    protected: physics::WorldPtr world;
    protected: physics::PhysicsEnginePtr physics;

    protected: physics::ModelPtr model;
    protected: std::string modelName;

    protected: double rho;
    protected: int *lid;

    protected: physics::LinkPtr link;
    protected: sdf::ElementPtr sdf;
    protected: std::map<int, properties> propsMap;

  };
}
#endif