#!/usr/bin/env python3
import os
from rackki_learning.dataset import Dataset
from rackki_learning.model import Model


def main():
    data_path = os.path.join(os.getcwd(), "../datasets/put")
    training_data = Dataset(data_path, sequence_length=25)
    evaluation_data = Dataset(data_path, sequence_length=25)
    model = Model(n_nodes=50, n_gaussians=5)
    model.train(
        training_data, evaluation_data, training_iterations=10000, batch_size=128
    )

    model_dir = os.path.join(os.getcwd(), "../models/put")
    model.save(model_dir)
    print(f"Saved model here: {model_dir}")


if __name__ == "__main__":
    main()
