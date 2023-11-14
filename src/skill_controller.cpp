#include "rackki_learning/skill_controller.h"
#include <tensorflow/cc/saved_model/loader.h>
#include <tensorflow/core/public/session.h>

namespace rackki_learning {

controller_interface::InterfaceConfiguration
SkillController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  return conf;
}

controller_interface::InterfaceConfiguration SkillController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  return conf;
}

SkillController::CallbackReturn SkillController::on_init()
{
  auto_declare<std::string>("model_path", "");
  return CallbackReturn::SUCCESS;
}

SkillController::CallbackReturn
SkillController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  auto model_path = get_node()->get_parameter("model_path").as_string();
  auto status     = tensorflow::LoadSavedModel(
    tensorflow::SessionOptions(), tensorflow::RunOptions(), model_path, {"serve"}, &m_bundle);
  if (!status.ok())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

SkillController::CallbackReturn
SkillController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  m_active         = true;
  m_serving_thread = std::thread([this]() {
    while (m_active)
    {
      tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, 7, 19}));
      std::vector<std::pair<std::string, tensorflow::Tensor> > inputs = {
        {"serving_default_lstm_input:0", input_tensor}};
      std::vector<tensorflow::Tensor> outputs;
      auto status = m_bundle.GetSession()->Run(inputs, {"StatefulPartitionedCall:0"}, {}, &outputs);
    }
  });

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
SkillController::update([[maybe_unused]] const rclcpp::Time& time,
                        [[maybe_unused]] const rclcpp::Duration& period)
{
  return controller_interface::return_type::ERROR;
}

SkillController::CallbackReturn
SkillController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  m_active = false;
  if (m_serving_thread.joinable())
  {
    m_serving_thread.join();
  }
  return CallbackReturn::SUCCESS;
}

SkillController::CallbackReturn
SkillController::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  m_active = false;
  if (m_serving_thread.joinable())
  {
    m_serving_thread.join();
  }
  auto status = m_bundle.GetSession()->Close();
  if (!status.ok())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

} // namespace rackki_learning

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rackki_learning::SkillController, controller_interface::ControllerInterface)
