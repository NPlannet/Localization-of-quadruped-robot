#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>

namespace xgo_description
{
namespace systems
{
class GlideTrajectorySystem
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  struct Data
  {
    gz::sim::Model model;
    gz::math::Vector3d start;
    gz::math::Vector3d end;
    double period = 10.0;
    double phaseOffset = 0.0;
    double yawTimeConstant = 0.35;
    double minDirectionSpeed = 1e-4;
    double currentYaw = 0.0;
    double fixedYaw = 0.0;
    bool alignYaw = true;
    bool hasFixedYaw = false;
    bool initialized = false;
  };

  public: GlideTrajectorySystem() : dataPtr(std::make_unique<Data>())
  {
  }

  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->dataPtr->model = gz::sim::Model(_entity);
    const gz::math::Pose3d modelPose = gz::sim::worldPose(_entity, _ecm);

    this->dataPtr->start = modelPose.Pos();
    if (_sdf->HasElement("start"))
    {
      this->dataPtr->start = _sdf->Get<gz::math::Vector3d>("start");
    }

    if (_sdf->HasElement("end"))
    {
      this->dataPtr->end = _sdf->Get<gz::math::Vector3d>("end");
    }
    else if (_sdf->HasElement("offset"))
    {
      this->dataPtr->end =
          this->dataPtr->start + _sdf->Get<gz::math::Vector3d>("offset");
    }
    else
    {
      this->dataPtr->end = this->dataPtr->start;
    }

    this->dataPtr->period = _sdf->Get<double>("period", 10.0).first;
    this->dataPtr->phaseOffset = _sdf->Get<double>("phase_offset", 0.0).first;
    this->dataPtr->yawTimeConstant = _sdf->Get<double>(
        "yaw_time_constant", 0.35).first;
    this->dataPtr->minDirectionSpeed = _sdf->Get<double>(
        "min_direction_speed", 1e-4).first;
    this->dataPtr->alignYaw = _sdf->Get<bool>("align_yaw", true).first;
    if (_sdf->HasElement("fixed_yaw"))
    {
      this->dataPtr->fixedYaw = _sdf->Get<double>("fixed_yaw");
      this->dataPtr->hasFixedYaw = true;
    }

    if (this->dataPtr->period <= 0.0)
      this->dataPtr->period = 10.0;

    const gz::math::Vector3d direction = this->dataPtr->end - this->dataPtr->start;
    if (this->dataPtr->hasFixedYaw)
    {
      this->dataPtr->currentYaw = this->dataPtr->fixedYaw;
    }
    else if (direction.Length() > this->dataPtr->minDirectionSpeed)
    {
      this->dataPtr->currentYaw = std::atan2(direction.Y(), direction.X());
    }
    else
    {
      this->dataPtr->currentYaw = modelPose.Rot().Yaw();
    }

    this->dataPtr->model.SetWorldPoseCmd(
        _ecm,
        gz::math::Pose3d(
            this->dataPtr->start,
            gz::math::Quaterniond(0.0, 0.0, this->dataPtr->currentYaw)));
    this->dataPtr->initialized = true;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->dataPtr->initialized)
      return;

    const double simTimeSec =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            _info.simTime).count();
    const double dtSec = std::max(
        std::chrono::duration_cast<std::chrono::duration<double>>(
            _info.dt).count(),
        1e-6);

    const double phase = (2.0 * M_PI *
        (simTimeSec + this->dataPtr->phaseOffset)) / this->dataPtr->period;
    const double progress = 0.5 - 0.5 * std::cos(phase);
    const gz::math::Vector3d delta = this->dataPtr->end - this->dataPtr->start;
    const gz::math::Vector3d position =
        this->dataPtr->start + delta * progress;
    const gz::math::Vector3d velocity =
        delta * ((M_PI / this->dataPtr->period) * std::sin(phase));

    if (this->dataPtr->hasFixedYaw)
    {
      this->dataPtr->currentYaw = this->dataPtr->fixedYaw;
    }
    else if (this->dataPtr->alignYaw &&
        velocity.Length() > this->dataPtr->minDirectionSpeed)
    {
      const double targetYaw = std::atan2(velocity.Y(), velocity.X());
      this->dataPtr->currentYaw = this->SmoothAngle(
          this->dataPtr->currentYaw,
          targetYaw,
          dtSec,
          this->dataPtr->yawTimeConstant);
    }

    this->dataPtr->model.SetWorldPoseCmd(
        _ecm,
        gz::math::Pose3d(
            position,
            gz::math::Quaterniond(0.0, 0.0, this->dataPtr->currentYaw)));
  }

  private: static double WrapAngle(const double _angle)
  {
    return std::atan2(std::sin(_angle), std::cos(_angle));
  }

  private: static double SmoothAngle(
      const double _current,
      const double _target,
      const double _dt,
      const double _timeConstant)
  {
    if (_timeConstant <= 1e-6)
      return _target;

    const double alpha = 1.0 - std::exp(-_dt / _timeConstant);
    const double error = WrapAngle(_target - _current);
    return WrapAngle(_current + alpha * error);
  }

  private: std::unique_ptr<Data> dataPtr;
};
}
}

GZ_ADD_PLUGIN(
    xgo_description::systems::GlideTrajectorySystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    xgo_description::systems::GlideTrajectorySystem,
    "xgo_description::systems::GlideTrajectorySystem")
