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
# \file    dataset.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2024/01/23
#
# -----------------------------------------------------------------------------

import os
from random import shuffle
import rclpy
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sklearn.preprocessing import StandardScaler


class Dataset(object):
    def __init__(self, directory_path: str, sequence_length: int = 25) -> None:
        self.inputs = []
        self.labels = []
        self.input_scaling = {}
        self.sequence_length = sequence_length

        file_paths = []
        for root, subdirectories, files in os.walk(directory_path):
            for filename in files:
                if filename.endswith(".db3"):
                    file_paths.append(os.path.join(root, filename))
        shuffle(file_paths)

        rclpy.init()
        node = rclpy.create_node("dataset_creation")
        rclpy.logging.get_logger("rosbag2_storage").set_level(
            30
        )  # FATAL = 50 ERROR = 40 WARN = 30 INFO = 20 DEBUG = 10 UNSET = 0  # noqa
        reader = SequentialReader()
        for file_count, file_path in enumerate(file_paths, 1):
            print(
                f"Reading dataset: {str(file_count)} / {str(len(file_paths))}",
                end="\r",
                flush=True,
            )
            tmp_poses = []
            tmp_wrenches = []
            storage_options = StorageOptions(uri=file_path, storage_id="sqlite3")
            converter_options = ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            )
            reader.open(storage_options, converter_options)
            topic_types = reader.get_all_topics_and_types()
            type_map = {
                topic_types[i].name: topic_types[i].type
                for i in range(len(topic_types))
            }

            while reader.has_next():
                topic, msg_data, t = reader.read_next()
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(msg_data, msg_type)
                if topic == "/current_pose":
                    tmp_poses.append(
                        [
                            msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z,
                            msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w,
                        ]
                    )
                if topic == "/target_wrench":
                    tmp_wrenches.append(
                        [
                            msg.wrench.force.x,
                            msg.wrench.force.y,
                            msg.wrench.force.z,
                            msg.wrench.torque.x,
                            msg.wrench.torque.y,
                            msg.wrench.torque.z,
                        ]
                    )

            # Only allow sufficiently long recordings
            common_length = min(len(tmp_poses), len(tmp_wrenches))
            if common_length > self.sequence_length:
                del tmp_poses[common_length:]
                del tmp_wrenches[common_length:]
                self.inputs.append(tmp_poses)
                self.labels.append(tmp_wrenches)
            reader.reset_filter()

        print("")
        node.destroy_node()
        rclpy.shutdown()

        # Build a single big vector for input feature scaling
        # and keep track of the subsequences to revert this concatenation later.
        split_idx = []
        idx = 0
        for i in self.inputs:
            idx += len(i)
            split_idx.append(idx)

        # Standardization
        self.inputs = np.concatenate(self.inputs, axis=0)
        std_scaler = StandardScaler()
        self.inputs = std_scaler.fit_transform(self.inputs)
        self.input_scaling = {
            "mean": std_scaler.mean_.tolist(),
            "sigma": std_scaler.scale_.tolist(),
        }

        # Revert shape
        self.inputs = np.split(self.inputs, split_idx)
        del self.inputs[-1]  # empty element through split at the end

    def get_file_count(self):
        return len(self.inputs)

    def get_batch(self, minibatch_size: int, seq_nr: int = None):
        """Compose a mini batch of data

        shape of inputs : [minibatch_size, sequence_length, in_dim]
        shape of labels : [minibatch_size, out_dim]

        time       : ------------->
        inputs (m) : ||||||||||
        labels (1) :           |

        """
        inputs = []
        labels = []

        def random_seq_nr():
            while True:
                seq_nr = np.random.randint(0, len(self.inputs))
                if len(self.inputs[seq_nr]) - self.sequence_length > 0:
                    return seq_nr

        def biased_seq_nr():
            draw = np.random.randint(0, 2)
            if draw == 0:
                return seq_nr
            return random_seq_nr()

        def standard_case():
            draw = np.random.randint(0, 10)
            if draw == 0:
                return False
            return True

        for _ in range(minibatch_size):
            if seq_nr is None:
                seq_nr = random_seq_nr()
            else:
                seq_nr = biased_seq_nr()

            limit = len(self.inputs[seq_nr]) - self.sequence_length
            begin = np.random.randint(0, limit)
            end = begin + self.sequence_length

            if standard_case():
                inputs.append([self.inputs[seq_nr][i] for i in range(begin, end)])
                labels.append(self.labels[seq_nr][end])
            else:
                inputs.append(
                    [self.inputs[seq_nr][begin] for i in range(self.sequence_length)]
                )
                labels.append(self.labels[seq_nr][begin + 1])

        return np.array(inputs), np.array(labels)
