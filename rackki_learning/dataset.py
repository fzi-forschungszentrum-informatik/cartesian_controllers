import os
from random import shuffle
import rclpy
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sklearn.preprocessing import StandardScaler
from copy import copy


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
        reader = SequentialReader()
        for file_count, file_path in enumerate(file_paths, 1):
            tmp_poses = []
            tmp_twists = []
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
                if topic == "/current_twist":
                    tmp_twists.append(
                        [
                            msg.twist.linear.x,
                            msg.twist.linear.y,
                            msg.twist.linear.z,
                            msg.twist.angular.x,
                            msg.twist.angular.y,
                            msg.twist.angular.z,
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

            common_length = min(len(tmp_poses), len(tmp_twists), len(tmp_wrenches))
            del tmp_poses[common_length:]
            del tmp_twists[common_length:]
            del tmp_wrenches[common_length:]
            print(f" {str(file_count)} / {str(len(file_paths))}", end="\r", flush=True)
            self.labels.append(
                np.concatenate((tmp_poses, tmp_twists, tmp_wrenches), axis=1)
            )
            reader.reset_filter()

        node.destroy_node()
        rclpy.shutdown()

        # Build a single big vector for input feature scaling
        # and keep track of the subsequences to revert this concatenation later.
        self.inputs = copy(self.labels)
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

    def get_batch(self, minibatch_size):
        """Compose a mini batch of data

        shape of inputs : [minibatch_size, sequence_length, in_dim]
        shape of labels : [minibatch_size, out_dim]

        time       : ------------->
        inputs (m) : ||||||||||
        labels (1) :           |

        """
        inputs = []
        labels = []

        for _ in range(minibatch_size):
            while True:  # Search for a sufficiently long sequence
                random_sequence = np.random.randint(0, len(self.inputs))
                limit = len(self.inputs[random_sequence]) - self.sequence_length
                if limit > 0:
                    break
            begin = np.random.randint(0, limit)
            end = begin + self.sequence_length

            # current pose:  [0:7]
            # current twist: [7:13]
            # target wrench: [13:19]
            inputs.append([self.inputs[random_sequence][i] for i in range(begin, end)])
            labels.append(self.labels[random_sequence][end][13:19])
        return np.array(inputs), np.array(labels)
