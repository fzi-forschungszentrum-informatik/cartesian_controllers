#!/usr/bin/env python3
import os
from rackki_learning.dataset import Dataset
from rackki_learning.model import Model


def main():
    training_files = os.path.join(os.getcwd(), "../datasets/put/train")
    evaluation_files = os.path.join(os.getcwd(), "../datasets/put/eval")
    training_data = Dataset(training_files, sequence_length=25)
    evaluation_data = Dataset(evaluation_files, sequence_length=25)
    model = Model(n_nodes=100, n_gaussians=12)
    model.train(
        training_data, evaluation_data, training_iterations=10000, batch_size=256
    )

    model_dir = os.path.join(os.getcwd(), "../models/put")
    model.save(model_dir)
    print(f"Saved model here: {model_dir}")


if __name__ == "__main__":
    main()
