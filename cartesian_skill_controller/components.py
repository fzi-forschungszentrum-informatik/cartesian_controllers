################################################################################
# Copyright 2024 FZI Research Center for Information Technology
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
################################################################################

# -----------------------------------------------------------------------------
# \file    components.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2024/01/23
#
# -----------------------------------------------------------------------------

import tensorflow as tf
from tensorflow.keras.layers import Layer
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


class PredictionLayer(Layer):
    def __init__(self, **kwargs):
        with tf.name_scope("prediction"):
            super().__init__(name="prediction", **kwargs)
            out_dim = 6  # 3 forces and 3 torques
            self.dense = Dense(out_dim, activation="linear")

    def call(self, x):
        with tf.name_scope("prediction"):
            return self.dense(x)
