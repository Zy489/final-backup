#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

class SimpleTTS(Node):
    def __init__(self):
        super().__init__('simple_tts')
        self.sub = self.create_subscription(String, 'say_text', self.on_text, 10)
        self.engine = pyttsx3.init()
        rate = self.engine.getProperty('rate')
        self.engine.setProperty('rate', int(rate * 1.1))

    def on_text(self, msg: String):
        try:
            self.get_logger().info(f"TTS: {msg.data}")
            self.engine.say(msg.data)
            self.engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTTS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
