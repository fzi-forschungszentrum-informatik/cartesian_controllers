import tensorflow as tf
from tensorflow.keras.layers import Layer
from tensorflow.keras.losses import Loss
from tensorflow_probability import distributions as tfd
from tensorflow.keras import saving
from tensorflow.keras.layers import (
    MultiHeadAttention,
    GlobalAveragePooling1D,
    Dense,
    Dropout,
    LayerNormalization,
    Add,
)
import numpy as np


class EmbeddingLayer(Layer):
    def __init__(self, sequence_length):
        with tf.name_scope("embedding"):
            super().__init__(name="embedding")
            self.embedding_dim = 32
            # Idea for the simple embedding here: https://arxiv.org/pdf/2010.02803.pdf
            self.embedding = Dense(units=self.embedding_dim, activation="relu")
            self.position_encoding = self.compute_position_encoding(
                length=sequence_length, depth=self.embedding_dim
            )

    def compute_position_encoding(self, length: int, depth: int):
        depth = depth / 2
        positions = np.arange(length)[:, np.newaxis]
        depths = np.arange(depth)[np.newaxis, :] / depth
        angle_rates = 1 / (10000**depths)
        angle_rads = positions * angle_rates
        pos_encoding = np.concatenate([np.sin(angle_rads), np.cos(angle_rads)], axis=-1)

        return tf.cast(pos_encoding, dtype=tf.float32)

    def call(self, x):
        with tf.name_scope("embedding"):
            length = tf.shape(x)[1]
            x = self.embedding(x)
            x *= tf.math.sqrt(tf.cast(self.embedding_dim, tf.float32))
            x = x + self.position_encoding[tf.newaxis, :length, :]
            return x


class FeedForwardLayer(Layer):
    def __init__(self, n_nodes, dropout=0.1):
        with tf.name_scope("self_attention"):
            super().__init__(name="feed_forward")
            self.dense = Dense(n_nodes, activation="relu")
            self.dropout = Dropout(dropout)
            self.layernorm = tf.keras.layers.LayerNormalization()

    def call(self, x):
        with tf.name_scope("self_attention"):
            x = self.dense(x)
            x = self.dropout(x)
            x = self.layernorm(x)
            return x


class SelfAttentionLayer(Layer):
    def __init__(self, n_heads, key_dim, **kwargs):
        with tf.name_scope("self_attention"):
            super().__init__(name="self_attention")
            self.attention = MultiHeadAttention(num_heads=n_heads, key_dim=key_dim)
            self.layernorm = LayerNormalization()
            self.average_pooling = GlobalAveragePooling1D()
            self.add = Add()

    def call(self, x):
        with tf.name_scope("self_attention"):
            output = self.attention(query=x, value=x, key=x)
            x = self.add([x, output])
            x = self.layernorm(x)
            x = self.average_pooling(x)
            return x


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
                return gaussian_mixture.mean()


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
