from rackki_learning.dataset import Dataset
from rackki_learning.model import Model
import os


def test_training(request):
    directory_path = os.path.join(request.node.fspath.dirname, "rosbags")
    training_data = Dataset(directory_path)
    evaluation_data = Dataset(directory_path)
    model = Model()
    success = model.train(training_data, evaluation_data)
    assert success
