#!/usr/bin/env python3
import rospy
import random
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class RandomWalker:
    def __init__(self):
        rospy.init_node("random_walker")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/walker_active", Bool, self.active_callback)
        self.active = True

        self.state = "FORWARD"  # or "TURN"
        self.last_turn_trigger_time = rospy.Time.now()
        self.turn_start_time = None
        self.turn_duration = rospy.Duration(0)
        self.turn_direction = 1

        self.turn_speed = 0.5  # rad/s
        self.forward_speed = 0.2  # m/s
        self.rate = rospy.Rate(10)  # 10 Hz

    def active_callback(self, msg):
        self.active = msg.data

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()

            if not self.active:
                self.pub.publish(Twist())  # 停止
                self.rate.sleep()
                continue

            now = rospy.Time.now()

            if self.state == "FORWARD":
                # 每隔 3 秒觸發轉向
                if (now - self.last_turn_trigger_time).to_sec() >= 3.0:
                    angle_deg = random.uniform(-180, 180)
                    angle_rad = math.radians(abs(angle_deg))
                    self.turn_direction = 1 if angle_deg > 0 else -1
                    duration = angle_rad / self.turn_speed

                    self.turn_duration = rospy.Duration(duration)
                    self.turn_start_time = now
                    self.state = "TURN"
                    rospy.loginfo(f"[Walker] Turning {angle_deg:.1f} degrees for {duration:.2f} sec")
                else:
                    twist.linear.x = self.forward_speed

            elif self.state == "TURN":
                if now - self.turn_start_time < self.turn_duration:
                    twist.angular.z = self.turn_direction * self.turn_speed
                else:
                    self.state = "FORWARD"
                    self.last_turn_trigger_time = now

            self.pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        RandomWalker().run()
    except rospy.ROSInterruptException:
        pass

