from rackki_learning.dataset import Dataset
import os


def test_dataset(request):
    directory_path = os.path.join(request.node.fspath.dirname, "./../rosbags")
    dataset = Dataset(directory_path)  # noqa: F841
    assert True
