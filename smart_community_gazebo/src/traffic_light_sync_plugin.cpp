#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/rendering.hh>

#include <functional>
#include <string>
#include <cmath>

namespace gazebo
{

class TrafficLightSyncPlugin : public VisualPlugin
{
public:
  void Load(rendering::VisualPtr visual, sdf::ElementPtr) override
  {
    visual_ = visual;

    update_connection_ =
      event::Events::ConnectPreRender(
        std::bind(&TrafficLightSyncPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    if (!visual_)
      return;

    gazebo::rendering::ScenePtr scene = visual_->GetScene();

    if (!scene)
      return;

    // 使用当前渲染场景的仿真时间
    const double sim_time = scene->SimTime().Double();
    const double t = std::fmod(sim_time, 28.0);

    std::string material;

    // 红灯：10秒
    if (t < 10.0)
      material = "TrafficLight/Red";

    // 绿灯：15秒
    else if (t < 25.0)
      material = "TrafficLight/Green";

    // 黄灯：3秒
    else
      material = "TrafficLight/Yellow";

    if (material != current_material_)
    {
      visual_->SetMaterial(material);
      current_material_ = material;
    }
  }

private:
  rendering::VisualPtr visual_;
  event::ConnectionPtr update_connection_;
  std::string current_material_;
};

GZ_REGISTER_VISUAL_PLUGIN(TrafficLightSyncPlugin)

}
