#!/usr/bin/env python3
import rclpy
import tensorflow as tf
from rclpy.node import Node
from rackki_learning.model import Model  # noqa: F401 Needed for load_model
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped
from collections import deque


class Server(Node):
    def __init__(self):
        super().__init__("server")
        self.prediction_rate = self.declare_parameter("prediction_rate", 5).value
        self.prediction_memory = self.declare_parameter("prediction_memory", 30).value
        self.model_dir = self.declare_parameter("model_dir", "").value
        self.model = tf.keras.models.load_model(self.model_dir)
        self.current_pose = PoseStamped()
        self.current_twist = TwistStamped()
        self.target_wrench = WrenchStamped()
        self.target_wrench_pub = self.create_publisher(
            WrenchStamped, "/target_wrench", 1
        )
        self.current_pose_sub = Subscriber(self, PoseStamped, "/current_pose")
        self.current_twist_sub = Subscriber(self, TwistStamped, "/current_twist")
        self.input_sequence = deque()
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.current_pose_sub, self.current_twist_sub],
            queue_size=1,
            slop=0.01,  # tolerated delay in sec
        )
        self.synchronizer.registerCallback(self.msg_callback)
        self.timer = self.create_timer(1.0 / self.prediction_rate, self.predict)

    def predict(self):
        self.update_input_sequence()
        input_data = tf.constant([list(self.input_sequence)], dtype=tf.float32)
        output = self.model.signatures["serving_default"](lstm_input=input_data)
        result = output["prediction"].numpy().tolist()[0]
        self.target_wrench.header.stamp = self.get_clock().now().to_msg()
        self.target_wrench.wrench.force.x = result[0]
        self.target_wrench.wrench.force.y = result[1]
        self.target_wrench.wrench.force.z = result[2]
        self.target_wrench.wrench.torque.x = result[3]
        self.target_wrench.wrench.torque.y = result[4]
        self.target_wrench.wrench.torque.z = result[5]
        self.target_wrench_pub.publish(self.target_wrench)

    def msg_callback(self, pose_msg, twist_msg):
        self.current_pose = pose_msg
        self.current_twist = twist_msg

    def update_input_sequence(self):
        point = [0.0 for i in range(19)]
        point[0] = self.current_pose.pose.position.x
        point[1] = self.current_pose.pose.position.y
        point[2] = self.current_pose.pose.position.z
        point[3] = self.current_pose.pose.orientation.x
        point[4] = self.current_pose.pose.orientation.y
        point[5] = self.current_pose.pose.orientation.z
        point[6] = self.current_pose.pose.orientation.w
        point[7] = self.current_twist.twist.linear.x
        point[8] = self.current_twist.twist.linear.y
        point[9] = self.current_twist.twist.linear.z
        point[10] = self.current_twist.twist.angular.x
        point[11] = self.current_twist.twist.angular.y
        point[12] = self.current_twist.twist.angular.z
        point[13] = self.target_wrench.wrench.force.x
        point[14] = self.target_wrench.wrench.force.y
        point[15] = self.target_wrench.wrench.force.z
        point[16] = self.target_wrench.wrench.torque.x
        point[17] = self.target_wrench.wrench.torque.y
        point[18] = self.target_wrench.wrench.torque.z
        self.input_sequence.appendleft(point)
        if len(self.input_sequence) > self.prediction_memory:
            self.input_sequence.pop()


def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
