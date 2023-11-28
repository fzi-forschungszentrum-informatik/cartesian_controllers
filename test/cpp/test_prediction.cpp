#include "tensorflow/cc/ops/const_op.h"
#include <filesystem>
#include <gtest/gtest.h>
#include <initializer_list>
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

  // Check the available input and output tensor names with:
  // saved_model_cli show --dir model_1 --all
  auto input_shape = tensorflow::TensorShape({1, 1, 19});
  tensorflow::Input::Initializer input(std::initializer_list<float>({1.0,
                                                                     2.0,
                                                                     3.0,
                                                                     4.0,
                                                                     5.0,
                                                                     6.0,
                                                                     7.0,
                                                                     8.0,
                                                                     9.0,
                                                                     10.0,
                                                                     11.0,
                                                                     12.0,
                                                                     13.0,
                                                                     14.0,
                                                                     15.0,
                                                                     16.0,
                                                                     17.0,
                                                                     18.0,
                                                                     19.0}),
                                       input_shape);

  std::vector<std::pair<std::string, tensorflow::Tensor> > inputs = {
    {"serving_default_lstm_input:0", input.tensor}};
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
