#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>
#include <thread>
#include <functional>

namespace gazebo
{

class FirePlugin : public ModelPlugin
{
public:
  FirePlugin() = default;

  ~FirePlugin() override
  {
    executor_.cancel();

    if (ros_thread_.joinable()) {
      ros_thread_.join();
    }
  }

  void Load(
    physics::ModelPtr model,
    sdf::ElementPtr /*sdf*/) override
  {
    model_ = model;

    // Gazebo transport
    gz_node_.reset(new transport::Node());
    gz_node_->Init(model_->GetWorld()->Name());

    visual_pub_ =
      gz_node_->Advertise<msgs::Visual>("~/visual");

    // ROS2
    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }

    const std::string model_name =
      model_->GetName();

    std::string building_name =
      model_name;

    const std::string suffix = "_fire";

    if (building_name.size() >= suffix.size() &&
        building_name.compare(
          building_name.size() - suffix.size(),
          suffix.size(),
          suffix) == 0)
    {
      building_name.erase(
        building_name.size() - suffix.size());
    }

    topic_name_ =
      "/" + building_name + "/fire";

    node_ = std::make_shared<rclcpp::Node>(
      "fire_plugin_" + model_name);

    subscription_ =
      node_->create_subscription<std_msgs::msg::Bool>(
        topic_name_,
        10,
        std::bind(
          &FirePlugin::FireCallback,
          this,
          std::placeholders::_1));

    executor_.add_node(node_);

    ros_thread_ = std::thread([this]() {
      executor_.spin();
    });

    RCLCPP_INFO(
      node_->get_logger(),
      "Fire model [%s] listening on [%s]",
      model_name.c_str(),
      topic_name_.c_str());
  }

private:

  void FireCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Building fire [%s] -> %s",
      model_->GetName().c_str(),
      msg->data ? "ON" : "OFF");

    SetFireVisual(msg->data);
  }

  void SetFireVisual(bool visible)
  {
    SetOneVisual(
      "fire_red",
      visible,
      1.0, 0.05, 0.0);

    SetOneVisual(
      "fire_orange",
      visible,
      1.0, 0.35, 0.0);

    SetOneVisual(
      "fire_yellow",
      visible,
      1.0, 1.0, 0.0);
  }

  void SetOneVisual(
    const std::string &visual_name,
    bool visible,
    double r,
    double g,
    double b)
  {
    msgs::Visual msg;

    const std::string link_name =
      model_->GetName() +
      "::fire_link";

    const std::string scoped_visual =
      link_name +
      "::" +
      visual_name;

    msg.set_name(scoped_visual);
    msg.set_parent_name(link_name);

    msgs::Material *material =
      msg.mutable_material();

    if (visible)
    {
      msgs::Set(
        material->mutable_ambient(),
        ignition::math::Color(
          r, g, b, 1.0));

      msgs::Set(
        material->mutable_diffuse(),
        ignition::math::Color(
          r, g, b, 1.0));

      msgs::Set(
        material->mutable_emissive(),
        ignition::math::Color(
          r, g, b, 1.0));
    }
    else
    {
      // alpha = 0：完全透明
      msgs::Set(
        material->mutable_ambient(),
        ignition::math::Color(
          0, 0, 0, 0));

      msgs::Set(
        material->mutable_diffuse(),
        ignition::math::Color(
          0, 0, 0, 0));

      msgs::Set(
        material->mutable_emissive(),
        ignition::math::Color(
          0, 0, 0, 0));

    }

    if (visible) {
    }

    visual_pub_->Publish(msg);
  }

  physics::ModelPtr model_;

  transport::NodePtr gz_node_;
  transport::PublisherPtr visual_pub_;

  std::string topic_name_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    subscription_;

  rclcpp::executors::
    SingleThreadedExecutor executor_;

  std::thread ros_thread_;
};

GZ_REGISTER_MODEL_PLUGIN(FirePlugin)

}
