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
# \file    model.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2024/01/23
#
# -----------------------------------------------------------------------------

import tensorflow as tf
from cartesian_skill_controller.dataset import Dataset
from cartesian_skill_controller.components import (
    EmbeddingLayer,
    SelfAttentionLayer,
    FeedForwardLayer,
    PredictionLayer,
)
import os
from os.path import expanduser
from datetime import datetime
import time
from subprocess import check_output, PIPE


class Model(object):
    def __init__(
        self,
        n_nodes: int = 20,
        key_dim: int = 10,
        n_heads: int = 8,
        sequence_length: int = 25,
    ):
        self.n_nodes = n_nodes
        self.key_dim = key_dim
        self.n_heads = n_heads
        self.sequence_length = sequence_length
        self.model = tf.keras.models.Sequential()
        self.model.add(EmbeddingLayer(sequence_length=sequence_length))
        self.model.add(SelfAttentionLayer(n_heads=n_heads, key_dim=key_dim))
        self.model.add(FeedForwardLayer(n_nodes=n_nodes))
        self.model.add(PredictionLayer())
        self.input_scaling = {}

    def train(
        self,
        training_data: Dataset,
        evaluation_data: Dataset,
        epochs: int = 3,
        iterations: int = 20,
        batch_size: int = 10,
        learning_rate: float = 0.0005,
        log_dir: str = os.path.join(
            expanduser("~"), "tensorboard", datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        ),
    ) -> bool:
        self.epochs = epochs
        self.iterations = iterations
        self.training_files = training_data.get_file_count()
        self.batch_size = batch_size
        self.learning_rate = learning_rate
        lr_schedule = tf.keras.optimizers.schedules.InverseTimeDecay(
            learning_rate, decay_steps=2000, decay_rate=1, staircase=False
        )
        self.model.compile(
            optimizer=tf.keras.optimizers.Adam(lr_schedule),
            loss=tf.keras.losses.MeanSquaredError(),
        )
        self.sequence_length = training_data.sequence_length
        self.input_scaling = training_data.input_scaling
        writer = tf.summary.create_file_writer(log_dir)
        start = time.time()
        estimated_steps = self.epochs * self.training_files * iterations
        estimated_finish = "xx:xx:xx"

        def estimate_finish(step):
            elapsed_time = time.time() - start
            estimated_time = (elapsed_time / step) * estimated_steps
            finish = start + estimated_time
            finish = datetime.fromtimestamp(finish).strftime("%H:%M:%S")
            return finish

        def done(step):
            return f"{(step / estimated_steps) * 100:.1f}%"

        step = 0
        with writer.as_default():
            for e in range(1, self.epochs + 1):
                for t in range(0, self.training_files):
                    for i in range(1, iterations + 1):
                        x_train, y_train = training_data.get_batch(batch_size, t)
                        loss = self.model.train_on_batch(x=x_train, y=y_train)
                        print(
                            (
                                f"Epochs: {e:>5} / {self.epochs:<5}  "
                                f"Files: {t:>5} / {self.training_files:<5}  "
                                f"Iterations: {i:>5} / {iterations:<5}  "
                                f"Done: {done(step):<5}  "
                                f"Estimated finish: {estimated_finish:<9}"
                            ),
                            end="\r",
                            flush=True,
                        )
                        step += 1

                        if step % 10 == 0:
                            x_eval, y_eval = evaluation_data.get_batch(batch_size)
                            loss_eval = self.model.test_on_batch(x=x_eval, y=y_eval)
                            y_pred = self.model.predict_on_batch(x=x_eval)
                            accuracy = tf.reduce_mean(
                                tf.keras.losses.MSE(y_true=y_eval, y_pred=y_pred)
                            )
                            tf.summary.scalar("training_loss", data=loss, step=step)
                            tf.summary.scalar(
                                "evaluation_loss", data=loss_eval, step=step
                            )
                            tf.summary.scalar(
                                "evaluation_accuracy", data=accuracy, step=step
                            )
                            writer.flush()

                        if step % 100 == 0:
                            estimated_finish = estimate_finish(step)
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
            f.write(f"key_dim: {self.key_dim}\n")
            f.write(f"sequence_length: {self.sequence_length}\n")
            f.write(f"training_files: {self.training_files}\n")
            f.write(f"batch_size: {self.batch_size}\n")
            f.write(f"learning_rate: {self.learning_rate}\n")
            f.write(f"epochs: {self.epochs}\n")
            f.write(f"iterations: {self.iterations}\n")
        return True
