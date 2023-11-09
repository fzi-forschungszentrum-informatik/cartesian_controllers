from rackki_learning.dataset import Dataset
from rackki_learning.model import Model
import tensorflow as tf
import os


def test_training_and_exporting_models(request):
    # Training
    data_path = os.path.join(request.node.fspath.dirname, "rosbags")
    training_data = Dataset(data_path)
    evaluation_data = Dataset(data_path)
    model = Model(n_nodes=17, n_gaussians=5)
    success = model.train(training_data, evaluation_data, training_iterations=12)
    assert success

    # Exporting
    model_dir = os.path.join(request.node.fspath.dirname, "models/model_1")
    success = model.export(model_dir)
    assert success


def test_loading_and_serving_models(request):
    # Loading
    exported_model_dir = os.path.join(request.node.fspath.dirname, "models/model_1")
    model = tf.saved_model.load(exported_model_dir)

    # Serving
    data_path = os.path.join(request.node.fspath.dirname, "rosbags")
    test_data = Dataset(data_path)
    x_test, y_test = test_data.get_batch(10)
    y = model.serve(x_test)  # noqa: F841
    # TODO:
    # - Support batchsize = 1
    # - Add an additional endpoint for sampling from a gaussian mixture
    assert True
