#!/usr/bin/env python3
import os
from rackki_learning.dataset import Dataset
from rackki_learning.model import Model

TYPE = "put"  # {put, get}
SEQ_LEN = 25


def main():
    training_files = os.path.join(os.getcwd(), f"../datasets/{TYPE}/train")
    evaluation_files = os.path.join(os.getcwd(), f"../datasets/{TYPE}/eval")
    training_data = Dataset(training_files, sequence_length=SEQ_LEN)
    evaluation_data = Dataset(evaluation_files, sequence_length=SEQ_LEN)
    model_dir = os.path.join(os.getcwd(), f"../models/{TYPE}")

    model = Model(
        n_nodes=100, key_dim=32, n_heads=8, n_gaussians=4, sequence_length=SEQ_LEN
    )
    model.train(
        training_data,
        evaluation_data,
        training_iterations=10000,
        batch_size=128,
        learning_rate=0.0003,
        log_dir=model_dir,
    )

    model.save(model_dir)
    print(f"Saved model here: {model_dir}")


if __name__ == "__main__":
    main()
