#!/usr/bin/env python3
import rclpy
import tensorflow as tf
from rclpy.node import Node


class Server(Node):
    def __init__(self):
        super().__init__("server")
        prediction_rate = 5  # Hz
        model_dir = self.declare_parameter("model_dir", "").value
        self.model = tf.keras.models.load_model(model_dir)
        self.timer = self.create_timer(1.0 / prediction_rate, self.predict)

    def predict(self):
        input_data = tf.constant(
            [
                [
                    [
                        1.0,
                        2.0,
                        3.0,
                        4.0,
                        5.0,
                        6.0,
                        7.0,
                        8.0,
                        9.0,
                        10.0,
                        11.0,
                        12.0,
                        13.0,
                        14.0,
                        15.0,
                        16.0,
                        17.0,
                        18.0,
                        19.0,
                    ]
                ]
            ],
            dtype=tf.float32,
        )
        output = self.model.signatures["serving_default"](lstm_input=input_data)
        output_tensor = output["prediction"].numpy()[0]
        print(output_tensor)


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
