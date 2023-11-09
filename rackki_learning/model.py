import tensorflow as tf
from rackki_learning.dataset import Dataset
from rackki_learning.components import (
    LSTMEncoderLayer,
    MixtureDensityLayer,
    NegLogLikelihood,
    GaussianMixture,
)


class Model(object):
    def __init__(self, n_nodes: int = 10, n_gaussians: int = 4):
        self.n_nodes = n_nodes
        self.n_gaussians = n_gaussians
        self.model = tf.keras.models.Sequential()
        self.model.add(LSTMEncoderLayer(self.n_nodes))
        self.model.add(MixtureDensityLayer(self.n_gaussians))

    def train(
        self,
        training_data: Dataset,
        evaluation_data: Dataset,
        training_iterations: int = 20,
        batch_size: int = 10,
        learning_rate: float = 0.0005,
    ) -> bool:
        self.model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate),
            loss=NegLogLikelihood(self.n_gaussians),
        )

        for step in range(1, training_iterations):
            x_train, y_train = training_data.get_batch(batch_size)
            loss = self.model.train_on_batch(x=x_train, y=y_train)

            if step % 10 == 0:
                x_eval, y_eval = evaluation_data.get_batch(batch_size)

                mixture_params = self.model.predict_on_batch(x=x_eval)
                mean = GaussianMixture(mixture_params, self.n_gaussians).mean()
                accuracy = tf.reduce_mean(
                    tf.keras.losses.MSE(y_true=y_eval, y_pred=mean)
                )
                loss_eval = self.model.test_on_batch(x=x_eval, y=y_eval)

                tf.summary.scalar("training_loss", data=loss, step=step)
                tf.summary.scalar("evaluation_loss", data=loss_eval, step=step)
                tf.summary.scalar("evaluation_accuracy", data=accuracy, step=step)

            print(f" {str(step)} / {str(training_iterations)}", end="\r", flush=True)
        return True

    def save(self, model_dir: str) -> bool:
        self.model.save(model_dir)
        return True
