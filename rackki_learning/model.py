import tensorflow as tf
from rackki_learning.dataset import Dataset
from rackki_learning.components import (
    LSTMEncoderLayer,
    MultiHeadAttentionLayer,
    MixtureDensityLayer,
    PredictionLayer,
    NegLogLikelihood,
)
import os
from os.path import expanduser
from datetime import datetime
from subprocess import check_output, PIPE


class Model(object):
    def __init__(
        self,
        n_nodes: int = 20,
        key_dim: int = 10,
        n_heads: int = 8,
        n_gaussians: int = 4,
    ):
        self.n_nodes = n_nodes
        self.key_dim = key_dim
        self.n_heads = n_heads
        self.n_gaussians = n_gaussians
        self.model = tf.keras.models.Sequential()
        self.model.add(LSTMEncoderLayer(n_nodes))
        self.model.add(MultiHeadAttentionLayer(n_heads=n_heads, key_dim=key_dim))
        self.model.add(MixtureDensityLayer(self.n_gaussians))
        self.model.add(PredictionLayer(self.n_gaussians))
        self.input_scaling = {}

    def set_testing(self, flag: bool):
        self.model.get_layer("prediction").testing = flag

    def train(
        self,
        training_data: Dataset,
        evaluation_data: Dataset,
        training_iterations: int = 20,
        batch_size: int = 10,
        learning_rate: float = 0.0005,
        log_dir: str = os.path.join(
            expanduser("~"), "tensorboard", datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        ),
    ) -> bool:
        self.training_iterations = training_iterations
        self.batch_size = batch_size
        self.learning_rate = learning_rate
        self.model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate),
            loss=NegLogLikelihood(self.n_gaussians),
        )
        self.sequence_length = training_data.sequence_length
        self.input_scaling = training_data.input_scaling
        writer = tf.summary.create_file_writer(log_dir)
        with writer.as_default():
            for step in range(1, training_iterations):
                x_train, y_train = training_data.get_batch(batch_size)
                loss = self.model.train_on_batch(x=x_train, y=y_train)

                if step % 10 == 0:
                    x_eval, y_eval = evaluation_data.get_batch(batch_size)
                    self.set_testing(True)
                    loss_eval = self.model.test_on_batch(x=x_eval, y=y_eval)
                    self.set_testing(False)
                    y_pred = self.model.predict_on_batch(x=x_eval)
                    accuracy = tf.reduce_mean(
                        tf.keras.losses.MSE(y_true=y_eval, y_pred=y_pred)
                    )
                    tf.summary.scalar("training_loss", data=loss, step=step)
                    tf.summary.scalar("evaluation_loss", data=loss_eval, step=step)
                    tf.summary.scalar("evaluation_accuracy", data=accuracy, step=step)
                    writer.flush()

                print(
                    f" {str(step)} / {str(training_iterations)}", end="\r", flush=True
                )
        return True

    def save(self, model_dir: str) -> bool:
        self.model.save(model_dir, save_format="tf")
        with open(os.path.join(model_dir, "input_scaling.yaml"), "w") as f:
            f.write(f"mean: {self.input_scaling['mean']}\n")
            f.write(f"sigma: {self.input_scaling['sigma']}\n")
        with open(os.path.join(model_dir, "training_parameters.yaml"), "w") as f:
            model_version = check_output(
                "git rev-parse HEAD", stdin=PIPE, cwd=os.getcwd(), shell=True
            ).decode("utf-8")
            f.write(f"model_version: {model_version}\n")
            f.write(f"n_nodes: {self.n_nodes}\n")
            f.write(f"n_heads: {self.n_heads}\n")
            f.write(f"n_gaussians: {self.n_gaussians}\n")
            f.write(f"key_dim: {self.key_dim}\n")
            f.write(f"sequence_length: {self.sequence_length}\n")
            f.write(f"batch_size: {self.batch_size}\n")
            f.write(f"learning_rate: {self.learning_rate}\n")
            f.write(f"training_iterations: {self.training_iterations}\n")
        return True
