#include <filesystem>
#include <gtest/gtest.h>
#include <tensorflow/cc/saved_model/loader.h>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>


TEST(rackki_learning, test_prediction)
{
  tensorflow::Session* session;
  tensorflow::SessionOptions session_options;
  tensorflow::SavedModelBundleLite bundle;

  std::string model_path = std::filesystem::absolute("model_1");

  tensorflow::Status status = tensorflow::NewSession(session_options, &session);
  ASSERT_TRUE(status.ok());

  status = tensorflow::LoadSavedModel(
    session_options, tensorflow::RunOptions(), model_path, {"serve"}, &bundle);
  ASSERT_TRUE(status.ok());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
