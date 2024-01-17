#!/usr/bin/env python3
import os
import sys
from rackki_learning.dataset import Dataset
from rackki_learning.model import Model

TYPE = "put"  # {put, get}
SEQ_LEN = 50


def main():
    training_files = os.path.join(os.getcwd(), f"../datasets/{TYPE}/train")
    evaluation_files = os.path.join(os.getcwd(), f"../datasets/{TYPE}/eval")
    training_data = Dataset(training_files, sequence_length=SEQ_LEN)
    evaluation_data = Dataset(evaluation_files, sequence_length=SEQ_LEN)
    model_dir = os.path.join(os.getcwd(), f"../models/{TYPE}")

    model = Model(
        n_nodes=128, key_dim=64, n_heads=4, n_gaussians=4, sequence_length=SEQ_LEN
    )

    try:
        model.train(
            training_data,
            evaluation_data,
            epochs=10,
            iterations=1,
            batch_size=128,
            learning_rate=0.0005,
            log_dir=model_dir,
        )
    except KeyboardInterrupt:
        model.save(model_dir)
        print(f"Training canceled. Saved intermediate model here: {model_dir}")
        sys.exit()

    model.save(model_dir)
    print(f"Training finished. Saved model here: {model_dir}")


if __name__ == "__main__":
    main()
