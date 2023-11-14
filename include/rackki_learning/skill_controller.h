#ifndef SKILL_CONTROLLER_H_INCLUDED
#define SKILL_CONTROLLER_H_INCLUDED

#include <controller_interface/controller_interface.hpp>
#include <tensorflow/cc/saved_model/loader.h>
#include <thread>

namespace rackki_learning {

class SkillController : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  LifecycleNodeInterface::CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

private:
  std::thread m_serving_thread;
  tensorflow::SavedModelBundleLite m_bundle;
  std::atomic<bool> m_active = false;
};

} // namespace rackki_learning

#endif
