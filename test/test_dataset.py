from rackki_learning.dataset import Dataset
import pytest
import os

def test_dataset(request):
    directory_path = os.path.join(request.node.fspath.dirname, "rosbags")
    dataset = Dataset(directory_path)
    assert True
