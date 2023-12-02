import tensorflow as tf
from tensorflow.keras.layers import Layer
from tensorflow.keras.losses import Loss
from tensorflow_probability import distributions as tfd
from tensorflow.keras import saving


class LSTMEncoderLayer(Layer):
    def __init__(self, n_nodes, **kwargs):
        with tf.name_scope("lstm"):
            self.n_nodes = n_nodes
            self.lstm = tf.keras.layers.LSTM(
                self.n_nodes,
                stateful=False,
                return_state=True,
                dropout=0.5,
                recurrent_dropout=0.5,
            )
        super().__init__(name="lstm", **kwargs)

    def call(self, x, training=None, dynamic_batch_size=True):
        with tf.name_scope("lstm"):
            pred, h_state, c_state = self.lstm(x, training=training)
            return tf.concat([pred, h_state, c_state], axis=-1)


class MixtureDensityLayer(Layer):
    def __init__(self, n_gaussians, **kwargs):
        out_dim = 6  # 3 forces and 3 torques

        def elu_plus_one(x):
            return tf.keras.backend.elu(x) + 1

        def make_dense(units: int, activation: str, name: str):
            return tf.keras.layers.Dense(units, activation=activation, name=name)

        with tf.name_scope("mdn"):
            self.mus = make_dense(n_gaussians * out_dim, activation=None, name="mus")
            self.sigmas = make_dense(
                n_gaussians * out_dim, activation="exponential", name="sigmas"
            )
            self.alphas = make_dense(n_gaussians, activation="softmax", name="alphas")
        super().__init__(name="mdn", **kwargs)

    def call(self, x, dynamic_batch_size=True):
        with tf.name_scope("mdn"):
            return tf.keras.layers.concatenate(
                [self.mus(x), self.sigmas(x), self.alphas(x)], name="mixture_params"
            )


class PredictionLayer(Layer):
    def __init__(self, n_gaussians, **kwargs):
        with tf.name_scope("prediction"):
            self.n_gaussians = n_gaussians
            self.testing = False
            super().__init__(name="prediction", **kwargs)

    def call(self, mixture_params, training=None, **kwargs):
        with tf.name_scope("prediction"):
            if training or self.testing:
                return mixture_params
            else:
                gaussian_mixture = GaussianMixture(mixture_params, self.n_gaussians)
                return gaussian_mixture.sample()


@saving.register_keras_serializable()
class NegLogLikelihood(Loss):
    def __init__(self, n_gaussians):
        self.n_gaussians = n_gaussians
        super().__init__(name="neg_log_likelihood")

    def call(self, y_true, y_pred, dynamic_batch_size=True):
        mixture_params = y_pred
        gaussian_mixture = GaussianMixture(mixture_params, self.n_gaussians)
        return tf.reduce_mean(tf.negative(gaussian_mixture.log_prob(y_true)))

    def get_config(self):
        return {"n_gaussians": self.n_gaussians}


class GaussianMixture(object):
    def __init__(self, mixture_params, n_gaussians):
        out_dim = 6  # 3 forces and 3 torques
        nk = n_gaussians
        mus, sigmas, alphas = tf.split(
            mixture_params, num_or_size_splits=[nk * out_dim, nk * out_dim, nk], axis=-1
        )
        mus = tf.split(mus, num_or_size_splits=[out_dim] * nk, axis=-1)
        sigmas = tf.split(sigmas, num_or_size_splits=[out_dim] * nk, axis=-1)
        components = [
            tfd.MultivariateNormalDiag(loc=a, scale_diag=b) for a, b in zip(mus, sigmas)
        ]
        self.gaussian_mixture = tfd.Mixture(
            cat=tfd.Categorical(probs=alphas), components=components, validate_args=True
        )

    def mean(self):
        return self.gaussian_mixture.mean()

    def log_prob(self, y_true):
        return self.gaussian_mixture.log_prob(y_true)

    def sample(self):
        return self.gaussian_mixture.sample()
