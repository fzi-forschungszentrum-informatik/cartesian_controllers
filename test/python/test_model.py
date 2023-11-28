from rackki_learning.dataset import Dataset
from rackki_learning.model import Model
import tensorflow as tf
import os
import numpy as np


def test_training_and_saving_models(request):
    # Training
    data_path = os.path.join(request.node.fspath.dirname, "./../rosbags")
    training_data = Dataset(data_path)
    evaluation_data = Dataset(data_path)
    model = Model(n_nodes=17, n_gaussians=5)
    success = model.train(training_data, evaluation_data, training_iterations=12)
    assert success

    # Saving
    model_dir = os.path.join(request.node.fspath.dirname, "./../models/model_1")
    success = model.save(model_dir)
    assert success


def test_loading_and_serving_models(request):
    # Loading
    model_dir = os.path.join(request.node.fspath.dirname, "./../models/model_1")
    model = tf.keras.models.load_model(model_dir)

    # Serving
    data_path = os.path.join(request.node.fspath.dirname, "./../rosbags")
    test_data = Dataset(data_path)
    x_test, y_test = test_data.get_batch(1)
    y = model.predict(x_test)
    assert np.array(y).shape == (1, 6)  # [[fx, fy, fz, tx, ty, tz]]
