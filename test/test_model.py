from rackki_learning.dataset import Dataset
from rackki_learning.model import Model
import os


def test_training(request):
    data_path = os.path.join(request.node.fspath.dirname, "rosbags")
    training_data = Dataset(data_path)
    evaluation_data = Dataset(data_path)
    model = Model(n_nodes=17, n_gaussians=5)
    success = model.train(training_data, evaluation_data, training_iterations=12)
    assert success
