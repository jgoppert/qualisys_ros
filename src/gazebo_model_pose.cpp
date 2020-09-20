#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace gazebo
{
  class ModelPose : public ModelPlugin
  {
  private:
    physics::ModelPtr model{NULL};
    ros::Subscriber sub;
    ros::NodeHandle n;

  public:

    ModelPose() {
      std::map<std::string, std::string> args;
      ros::init(args, "set_pose");
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
      // Store the pointer to the model
      this->model = _parent;
      this->model->SetGravityMode(false);
      this->model->SetStatic(true);
      if (_sdf->HasElement("topic")) {
        std::string topic = _sdf->GetElement("topic")->Get<std::string>();
        this->sub = this->n.subscribe(topic, 1000, &ModelPose::PoseCallback, this);
      } else {
        gzerr << "[gazebo_model_pose] Must set pose topic";
      }
    }

    void PoseCallback(const geometry_msgs::PoseStamped& msg) {
      ignition::math::Pose3d pose = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
        msg.pose.orientation.w, msg.pose.orientation.x,
        msg.pose.orientation.y, msg.pose.orientation.z
      };
      this->model->SetWorldPose(pose);
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPose)
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
