#!/usr/bin/env python3
import rclpy
import tensorflow as tf
from rclpy.node import Node
from rackki_learning.model import Model  # noqa: F401 Needed for load_model
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped


class Server(Node):
    def __init__(self):
        super().__init__("server")
        prediction_rate = 5  # Hz
        model_dir = self.declare_parameter("model_dir", "").value
        self.model = tf.keras.models.load_model(model_dir)
        self.target_wrench_pub = self.create_publisher(
            WrenchStamped, "/target_wrench", 1
        )
        self.current_pose_sub = Subscriber(self, PoseStamped, "/current_pose")
        self.current_twist_sub = Subscriber(self, TwistStamped, "/current_twist")
        self.input_buffer = [0.0 for i in range(19)]
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.current_pose_sub, self.current_twist_sub],
            queue_size=1,
            slop=0.01,  # tolerated delay in sec
        )
        self.synchronizer.registerCallback(self.msg_callback)
        self.timer = self.create_timer(1.0 / prediction_rate, self.predict)

    def predict(self):
        input_data = tf.constant([[self.input_buffer]], dtype=tf.float32)
        output = self.model.signatures["serving_default"](lstm_input=input_data)
        target_wrench = output["prediction"].numpy().tolist()[0]
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wrench.force.x = target_wrench[0]
        msg.wrench.force.y = target_wrench[1]
        msg.wrench.force.z = target_wrench[2]
        msg.wrench.torque.x = target_wrench[3]
        msg.wrench.torque.y = target_wrench[4]
        msg.wrench.torque.z = target_wrench[5]
        self.target_wrench_pub.publish(msg)
        self.input_buffer[13:] = target_wrench

    def msg_callback(self, pose_msg, twist_msg):
        self.input_buffer[0] = pose_msg.pose.position.x
        self.input_buffer[1] = pose_msg.pose.position.y
        self.input_buffer[2] = pose_msg.pose.position.z
        self.input_buffer[3] = pose_msg.pose.orientation.x
        self.input_buffer[4] = pose_msg.pose.orientation.y
        self.input_buffer[5] = pose_msg.pose.orientation.z
        self.input_buffer[6] = pose_msg.pose.orientation.w
        self.input_buffer[7] = twist_msg.twist.linear.x
        self.input_buffer[8] = twist_msg.twist.linear.y
        self.input_buffer[9] = twist_msg.twist.linear.z
        self.input_buffer[10] = twist_msg.twist.angular.x
        self.input_buffer[11] = twist_msg.twist.angular.y
        self.input_buffer[12] = twist_msg.twist.angular.z


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
