#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def deg2rad(d):
    return d * math.pi / 180.0

# A=前, B=右, C=后, D=左
DIR2ANGLE_RAD = {
    'A': deg2rad(0),
    'B': deg2rad(-90),   # 如果发现方向反了可以改成 +90
    'C': deg2rad(180),
    'D': deg2rad(90),
}

class QuizDemoHardcode(Node):
    """
    硬编码顺序：B -> C -> B
    在每一题开始前等待：
      Q1: 30 秒
      Q2: 30 秒
      Q3: 60 秒
    然后：转向 -> 直行到答案 -> 停 5s 说一句话 -> 掉头 -> 回中心 -> 回正。
    """

    def __init__(self):
        super().__init__('quiz_demo_hardcode')

        # ===== 参数 =====
        self.declare_parameter('sequence', ['B', 'C', 'B'])
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('distance_m', 1.5)
        self.declare_parameter('stop_wait_s', 5.0)
        self.declare_parameter(
            'tts_text',
            "Congratulations! You answered correctly!"
        )

        # 三道题的等待时间（秒）
        self.answer_times = [30.0, 30.0, 60.0]

        # ===== 通信 =====
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.say_pub = self.create_publisher(String, 'say_text', 10)

        # ===== 参数读取 =====
        self.seq = [s for s in self.get_parameter('sequence').value]
        self.idx = 0  # 当前题号 0,1,2

        self.lin_spd = float(self.get_parameter('linear_speed').value)
        self.ang_spd = float(self.get_parameter('angular_speed').value)
        self.dist = float(self.get_parameter('distance_m').value)
        self.stop_wait = float(self.get_parameter('stop_wait_s').value)
        self.tts_text = self.get_parameter('tts_text').value

        # 控制时长
        self.turn_time = 0.0
        self.forward_time = self.dist / max(self.lin_spd, 1e-3)
        self.turn_sign = 1.0

        # 初始状态：等待第 1 题答题
        now = self.get_clock().now()
        self.state = 'WAIT_ANSWER'
        self.phase_end = now + Duration(seconds=self.get_answer_time_for_index(0))

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("QuizDemoHardcode started, waiting for Question 1...")

    def get_answer_time_for_index(self, idx: int) -> float:
        if idx < len(self.answer_times):
            return self.answer_times[idx]
        return 30.0  # 兜底

    def plan_for_current_answer(self):
        if self.idx >= len(self.seq):
            return
        ans = self.seq[self.idx]
        ang = DIR2ANGLE_RAD[ans]

        self.turn_sign = 1.0 if ang >= 0 else -1.0
        self.turn_time = abs(ang) / max(self.ang_spd, 1e-3)
        self.forward_time = self.dist / max(self.lin_spd, 1e-3)

        self.get_logger().info(
            f"[Q{self.idx+1}/{len(self.seq)}] Answer={ans}, "
            f"turn={self.turn_time:.2f}s, forward={self.forward_time:.2f}s"
        )

    def loop(self):
        now = self.get_clock().now()
        cmd = Twist()

        # 所有题都做完且完全空闲
        if self.idx >= len(self.seq) and self.state == 'IDLE':
            self.cmd_pub.publish(cmd)
            return

        # ========== 状态机 ==========

        # 1) 等学生答题
        if self.state == 'WAIT_ANSWER':
            if now > self.phase_end:
                self.plan_for_current_answer()
                self.phase_end = now + Duration(seconds=self.turn_time)
                self.state = 'TURNING'

        # 2) 朝答案方向转向
        elif self.state == 'TURNING':
            cmd.angular.z = self.ang_spd * self.turn_sign
            if now > self.phase_end:
                self.phase_end = now + Duration(seconds=self.forward_time)
                self.state = 'ADVANCE'

        # 3) 朝答案方向前进
        elif self.state == 'ADVANCE':
            cmd.linear.x = self.lin_spd
            if now > self.phase_end:
                # 到达答案位置：说一句祝贺语，并停留
                self.say_pub.publish(String(data=self.tts_text))
                self.phase_end = now + Duration(seconds=self.stop_wait)
                self.state = 'STOP_AT_GOAL'

        # 4) 在答案位置停留
        elif self.state == 'STOP_AT_GOAL':
            if now > self.phase_end:
                # 掉头 180°
                self.phase_end = now + Duration(seconds=math.pi / self.ang_spd)
                self.state = 'TURN_BACK'

        # 5) 掉头回中心
        elif self.state == 'TURN_BACK':
            cmd.angular.z = self.ang_spd
            if now > self.phase_end:
                self.phase_end = now + Duration(seconds=self.forward_time)
                self.state = 'RETURN'

        # 6) 回中心
        elif self.state == 'RETURN':
            cmd.linear.x = self.lin_spd
            if now > self.phase_end:
                # 回到中心后再转回正前方
                self.phase_end = now + Duration(seconds=math.pi / self.ang_spd)
                self.state = 'FACE_FORWARD'

        # 7) 转回面向前
        elif self.state == 'FACE_FORWARD':
            cmd.angular.z = self.ang_spd
            if now > self.phase_end:
                # 准备下一题
                self.idx += 1
                if self.idx >= len(self.seq):
                    self.state = 'IDLE'
                    self.get_logger().info("All questions finished.")
                else:
                    wait_time = self.get_answer_time_for_index(self.idx)
                    self.phase_end = now + Duration(seconds=wait_time)
                    self.state = 'WAIT_ANSWER'
                    self.get_logger().info(
                        f"Waiting {wait_time:.0f}s for Question {self.idx+1}..."
                    )

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = QuizDemoHardcode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
