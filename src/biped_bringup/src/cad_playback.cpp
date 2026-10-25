// Kinematic visualization only: no forces, balance controller or hardware output.
#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/common/Console.hh>

namespace biped {
class CadPlayback : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate {
  struct Frame { double time; std::vector<gz::math::Pose3d> poses; };
  std::vector<gz::sim::Entity> links;
  std::vector<Frame> frames;
  bool announced = false;
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    std::ifstream input(sdf->Get<std::string>("trajectory"));
    if (!input) throw std::runtime_error("Cannot open CAD demo trajectory");
    std::string line, name;
    std::getline(input, line);
    std::istringstream header(line);
    gz::sim::Model model(entity);
    while (header >> name) {
      auto link = model.LinkByName(ecm, name);
      if (!link) throw std::runtime_error("Missing CAD link: " + name);
      links.push_back(link);
    }
    if (links.empty()) throw std::runtime_error("Empty CAD trajectory header");
    while (std::getline(input, line)) {
      std::istringstream row(line);
      Frame frame;
      if (!(row >> frame.time)) continue;
      for (size_t i = 0; i < links.size(); ++i) {
        double x, y, z, r, p, yaw;
        if (!(row >> x >> y >> z >> r >> p >> yaw))
          throw std::runtime_error("Incomplete CAD trajectory frame");
        frame.poses.emplace_back(x, y, z, r, p, yaw);
      }
      if (!frames.empty() && frame.time <= frames.back().time)
        throw std::runtime_error("CAD trajectory times must increase");
      frames.push_back(std::move(frame));
    }
    if (frames.empty()) throw std::runtime_error("Empty CAD trajectory");
    gzmsg << "CAD WALK + STAIRS: kinematic demonstration, "
          << frames.back().time << " seconds. Pause/reset with Gazebo controls."
          << std::endl;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || frames.empty()) return;
    double t = std::chrono::duration<double>(info.simTime).count();
    auto next = std::upper_bound(frames.begin(), frames.end(), t,
      [](double v, const Frame &f) { return v < f.time; });
    const Frame &b = next == frames.end() ? frames.back() : *next;
    const Frame &a = next == frames.begin() ? frames.front() : *(next - 1);
    double u = a.time == b.time ? 0.0 : std::clamp((t-a.time)/(b.time-a.time), 0.0, 1.0);
    for (size_t i = 0; i < links.size(); ++i) {
      gz::math::Pose3d pose(a.poses[i].Pos()*(1-u) + b.poses[i].Pos()*u,
        gz::math::Quaterniond::Slerp(u, a.poses[i].Rot(), b.poses[i].Rot()));
      ecm.SetComponentData<gz::sim::components::Pose>(links[i], pose);
      // SetComponentData alone does not flag changes for the GUI state stream.
      ecm.SetChanged(links[i], gz::sim::components::Pose::typeId,
                     gz::sim::ComponentState::PeriodicChange);
    }
    if (t >= frames.back().time && !announced) {
      gzmsg << "CAD stair demo complete: both feet on the upper landing." << std::endl;
      announced = true;
    }
    if (t < frames.back().time) announced = false;
  }
};
}
GZ_ADD_PLUGIN(biped::CadPlayback, gz::sim::System,
              biped::CadPlayback::ISystemConfigure,
              biped::CadPlayback::ISystemPreUpdate)
