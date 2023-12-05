#include <deque>
#include <filesystem>
#include <gtest/gtest.h>
#include <initializer_list>
#include <iostream>
#include <tensorflow/cc/framework/ops.h>
#include <tensorflow/cc/ops/standard_ops.h>
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
  auto scope    = tensorflow::Scope::NewRootScope();

  std::deque<std::array<float, 7> > input_sequence;
  input_sequence.push_front({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0});
  input_sequence.push_front({1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7});

  tensorflow::Tensor input(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, 2, 7}));
  auto data = input.tensor<float, 3>();

  for (int i = 0; i < input_sequence.size(); ++i)
  {
    for (int j = 0; j < 7; ++j)
    {
      data(0, i, j) = input_sequence[i][j];
    }
  }
  input.tensor<float, 3>() = data;

  std::cout << "########## data:\n" << data << std::endl;
  std::vector<tensorflow::Tensor> outputs;

  // Check the available input and output tensor names with:
  // saved_model_cli show --dir model_1 --all
  status = session->Run(
    {{"serving_default_lstm_input:0", input}}, {"StatefulPartitionedCall:0"}, {}, &outputs);
  std::cout << status.message() << std::endl;
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
