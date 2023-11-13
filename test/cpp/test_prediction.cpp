#include <filesystem>
#include <gtest/gtest.h>
#include <tensorflow/cc/saved_model/loader.h>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>


TEST(rackki_learning, test_prediction)
{
  tensorflow::SessionOptions session_options;
  tensorflow::SavedModelBundleLite bundle;

  std::string model_path = std::filesystem::absolute("model_1");

  auto status = tensorflow::LoadSavedModel(
    session_options, tensorflow::RunOptions(), model_path, {"serve"}, &bundle);
  ASSERT_TRUE(status.ok());

  auto* session = bundle.GetSession();

  // Check the available input and output tensor names with:
  // saved_model_cli show --dir model_1 --all
  tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, 7, 19}));
  std::vector<std::pair<std::string, tensorflow::Tensor> > inputs = {
    {"serving_default_lstm_input:0", input_tensor}};
  std::vector<tensorflow::Tensor> outputs;

  status = session->Run(inputs, {"StatefulPartitionedCall:0"}, {}, &outputs);
  ASSERT_TRUE(status.ok());
  ASSERT_TRUE(outputs[0].shape() == tensorflow::TensorShape({1, 6}));

  status = session->Close();
  ASSERT_TRUE(status.ok());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
