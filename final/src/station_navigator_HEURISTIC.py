#!/usr/bin/env python3
import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt
import math

# 목적지 위치 미리 정의
station_locs = [
    (0.42, -0.23, 0.0),
    (0.62, -1.56, - math.pi / 2),
    (1.00, -2.91, 0.0),
    (2.78, -2.68, 0.0),
    (1.32, -0.14, 0.0),
    (3.34, 0.16, - math.pi / 2),
    (3.34, -1.05, - math.pi / 2)
]

class Station():
    def __init__(self, num, loc):
        self.num = num
        self.state = [0, 0, 0]
        self.loc = loc
        self.time = float('inf') 
        self.finish_time = float('inf')
        self.producing = 0
        self.ct_part = float("inf")
    
class Robot:
    def __init__(self):
        self.inv = [0, 0, 0, 0, 0, 0, 0, 0]
        self.station = 0
        self.capacity = 3

class Env:
    def __init__(self, locs):
        self.station = [Station(num, locs[num]) for num in range(7)]
        self.robot = Robot()
        self.start_time = time.time()

        self.station[0].state = [12, 0, 0]
        self.station[1].state = [[0, 0, 0], [0, 0, 0]]
        self.station[2].state = [2, 0, 0]
        self.station[3].state = [2, 0, 0]
        self.station[4].state = [2, 0, 0]
        self.station[5].state = [2, 0, 0]
        self.station[6].state = [[0, 0, 0], [0, 0, 0]]

        self.process_time = [10, 20, 20, 10, 30, 10, 10]

        self.wip_a = 4
        self.wip_b = 4
        self.goal_a = 8
        self.goal_b = 7
        self.goal = self.goal_a + self.goal_b

        self.wipa = []
        self.wipb = []

    def next_station(self):
        idx = 0

        if sum(self.robot.inv) == 0:
            if self.station[1].state[0][0] <= 1 and self.wip_a < self.goal_a:   #raw materials A
                idx = 0

            elif self.station[1].state[1][0] <= 1 and self.wip_b < self.goal_b:     # raw materials B
                idx = 0

            elif self.station[2].state[0] <= 1 and self.station[1].state[0][2] > 0:    # process A_1
                idx = 1

            elif self.station[4].state[0] <= 1 and self.station[1].state[1][2] > 0:    # process B_1
                idx = 1

            elif self.station[3].state[0] <= 1 and self.station[2].state[2] > 0:    # process A_2
                idx = 2

            elif self.station[5].state[0] <= 1 and self.station[4].state[2] > 0:    # process B_2
                idx = 4

            elif self.station[3].state[2] > 0:  # part A
                idx = 3

            elif self.station[5].state[2] > 0:  # part B
                idx = 5

            else:
                idx = 0

        elif self.robot.inv[0] > 0:
            idx = 1

        elif self.robot.inv[1] > 0:
            idx = 1

        elif self.robot.inv[2] > 0:
            idx = 2

        elif self.robot.inv[3] > 0:
            idx = 3

        elif self.robot.inv[4] > 0:
            idx = 4

        elif self.robot.inv[5] > 0:
            idx = 5

        elif self.robot.inv[6] > 0:
            idx = 6

        elif self.robot.inv[7] > 0:
            idx = 6

        else:
            idx = 0

        self.robot.station = idx

        return self.station[idx]

    def update_station(self):
        if self.station[1].state[0][1] + self.station[1].state[1][1] == 0 and \
           self.station[1].state[0][0] + self.station[1].state[1][0] > 0:
            self.station[1].time = time.time()

            cnt = min(self.station[1].state[0][0], 2)
            self.station[1].state[0][0] -= cnt
            self.station[1].state[0][1] += cnt
            
            cnt = min(self.station[1].state[1][0], 2 - cnt)
            self.station[1].state[1][0] -= cnt
            self.station[1].state[1][1] += cnt

        if self.station[1].time <= time.time() - self.process_time[1]:
            self.station[1].state[0][2] += self.station[1].state[0][1]
            self.station[1].state[0][1] = 0

            self.station[1].state[1][2] += self.station[1].state[1][1]
            self.station[1].state[1][1] = 0

            self.station[1].time = float('inf')
            self.station[1].finish_time = time.time()
        
        for i in range(2, 6):
            if self.station[i].state[1] == 0 and self.station[i].state[0] > 0:
                cnt = min(self.station[i].state[0], 2)

                if cnt > 0:
                    self.station[i].time = time.time()

                self.station[i].state[0] -= cnt
                self.station[i].state[1] += cnt

            if self.station[i].time <= time.time() - self.process_time[i]:
                self.station[i].state[2] += self.station[i].state[1]
                self.station[i].state[1] = 0

                self.station[i].time = float('inf')
                self.station[i].finish_time = time.time()

        self.wipa.append(self.wip_a - self.station[6].state[0][0])
        self.wipb.append(self.wip_b - self.station[6].state[1][0])

        print("[INFO] 스테이션 상태 업데이트:")
        for s in self.station:
            print(f"스테이션 {s.num} 상태: {s.state}, 시간: {s.time}")
        return

    def update_robot(self):
        idx = self.robot.station

        if idx == 0:
            rospy.sleep(self.process_time[6])
            capacity = self.robot.capacity - sum(self.robot.inv)            
            cnt = min(self.goal_a - self.wip_a, capacity)

            self.station[0].state[0] -= cnt
            self.robot.inv[0] += cnt
            self.wip_a += cnt

            capacity = self.robot.capacity - sum(self.robot.inv)
            cnt = min(self.goal_b - self.wip_b, capacity)

            self.station[0].state[0] -= cnt
            self.robot.inv[1] += cnt
            self.wip_b += cnt

        elif idx == 1:
            self.station[1].state[0][0] += self.robot.inv[0]
            self.robot.inv[0] = 0

            self.station[1].state[1][0] += self.robot.inv[1]
            self.robot.inv[1] = 0

            capacity = self.robot.capacity - sum(self.robot.inv)
            cnt = min(self.station[1].state[0][2], capacity)

            self.station[1].state[0][2] -= cnt
            self.robot.inv[2] += cnt

            capacity = self.robot.capacity - sum(self.robot.inv)
            cnt = min(self.station[1].state[1][2], capacity)

            self.station[1].state[1][2] -= cnt
            self.robot.inv[4] += cnt       

        elif idx == 6:
            rospy.sleep(self.process_time[6])
            self.station[6].state[0][0] += self.robot.inv[6]
            self.robot.inv[6] = 0

            if self.station[6].state[0][0] >= self.goal_a and self.station[6].state[0][1] == float('inf'):    
                self.station[6].state[0][1] = time.time()   #finish_time a

            self.station[6].state[1][0] += self.robot.inv[7]
            self.robot.inv[7] = 0

            if self.station[6].state[1][0] >= self.goal_b and self.station[6].state[1][1] == float('inf'):
                self.station[6].state[1][1] = time.time()   #finish_time b

        else:
            self.station[idx].state[0] += self.robot.inv[idx]
            self.station[idx].producing += self.robot.inv[idx]

            if idx == 2 and self.station[idx].producing >= self.goal_a - 4 and self.station[idx].ct_part == float('inf'):
                self.station[idx].ct_part = (time.time() - self.start_time) / (self.goal_a - 4)

            elif idx == 3 and self.station[idx].producing >= self.goal_a - 2 and self.station[idx].ct_part == float('inf'):
                self.station[idx].ct_part = (time.time() - self.start_time + self.station[2].ct_part * 2) / (self.goal_a - 2)

            elif idx == 4 and self.station[idx].producing >= self.goal_b - 4 and self.station[idx].ct_part == float('inf'):
                self.station[idx].ct_part = (time.time() - self.start_time) / (self.goal_b - 4)

            elif idx == 5 and self.station[idx].producing >= self.goal_b - 2 and self.station[idx].ct_part == float('inf'):
                self.station[idx].ct_part = (time.time() - self.start_time + self.station[4].ct_part * 2) / (self.goal_b - 2)

            self.robot.inv[idx] = 0

            capacity = self.robot.capacity - sum(self.robot.inv)
            cnt = min(self.station[idx].state[2], capacity)

            self.station[idx].state[2] -= cnt
            
            if idx == 2 or idx == 4:
                self.robot.inv[idx + 1] += cnt

            elif idx == 3:
                self.robot.inv[idx + 3] += cnt

            else:   # idx == 5
                self.robot.inv[idx + 2] += cnt

        print(f"[INFO] 로봇 상태 업데이트: {self.robot.inv}, 스테이션 {idx} 상태: {self.station[idx].state}")
        return

def move_loc(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    quat = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    rospy.loginfo(f"[INFO] 이동 중 : x={x}, y={y}, yaw={yaw}")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("[INFO] 도착 완료!")

def main():
    rospy.init_node('station_navigator_client')
    env = Env(station_locs)

    timer = rospy.Timer(rospy.Duration(1), lambda event: env.update_station())

    while env.goal > env.station[6].state[0][0] + env.station[6].state[1][0]:
        move_loc(*env.next_station().loc)
        env.update_robot()

    timer.shutdown()

    ct_part = [env.station[i].ct_part for i in range(2, 6)]
    ct = [((ct_part[2 * i] + ct_part[2 * i + 1]) * 2 + env.station[6].state[i][1]) / env.station[6].state[i][0] for i in range(2)]

    th_1 = (env.goal - 8) / (env.station[1].finish_time - env.start_time)
    th_2 = (env.goal_a) / (env.station[2].finish_time - env.start_time + ct_part[0] * 2)
    th_3 = (env.goal_a) / (env.station[4].finish_time - env.start_time + ct_part[1] * 2)
    th_4 = (env.goal_b) / (env.station[3].finish_time - env.start_time + ct_part[2] * 2)
    th_5 = (env.goal_b) / (env.station[5].finish_time - env.start_time + ct_part[3] * 2)
    th_a = (env.goal_a) / (ct[0])
    th_b = (env.goal_b) / (ct[1])
    th = [th_1, th_2, th_3, th_4, th_5, th_a, th_b]

    total_completion_time = time.time() - env.start_time

    capacity = [2 / env.process_time[i] for i in range(1, 6)]
    utilization = [th[i] / capacity[i] for i in range(5)]

    bottleneck = utilization.index(max(utilization)) + 1

    print(ct, th, utilization, bottleneck, total_completion_time)

    time_list = list(range(len(env.wipa)))
    plt.figure(figsize=(10, 5))
    plt.plot(time_list, env.wipa, marker='o', linestyle='-', color='blue', label='WIPA over time')
    plt.plot(time_list, env.wipb, marker='o', linestyle='-', color='red', label='WIPB over time')

    plt.title('WIP vs Time')
    plt.xlabel('Time (sec)')
    plt.ylabel('WIP')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
