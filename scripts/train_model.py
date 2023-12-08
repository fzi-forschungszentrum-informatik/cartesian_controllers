#!/usr/bin/env python3
import os
from rackki_learning.dataset import Dataset
from rackki_learning.model import Model

TYPE = "put"  # {put, get}


def main():
    training_files = os.path.join(os.getcwd(), f"../datasets/{TYPE}/train")
    evaluation_files = os.path.join(os.getcwd(), f"../datasets/{TYPE}/eval")
    training_data = Dataset(training_files, sequence_length=25)
    evaluation_data = Dataset(evaluation_files, sequence_length=25)
    model = Model(n_nodes=150, key_dim=32, n_heads=8, n_gaussians=4)
    model.train(
        training_data,
        evaluation_data,
        training_iterations=30000,
        batch_size=128,
        learning_rate=0.0005,
    )

    model_dir = os.path.join(os.getcwd(), f"../models/{TYPE}")
    model.save(model_dir)
    print(f"Saved model here: {model_dir}")


if __name__ == "__main__":
    main()
