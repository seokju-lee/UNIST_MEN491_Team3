import sys
import math
import numpy as np
from numpy.lib.function_base import average
from rospy.names import scoped_name
import rospy
import csv
import time

# Parameters
k = 0.3  # look forward gain
Lfc = 0.5  # [m] look-ahead distance
Kp = 2.5  # speed proportional gain
WB = 0.5  # [m] wheel base of vehicle
vel = 9

#drives straight ahead at a speed of 5
def distance(x1, y1, x2, y2):
    result = math.sqrt( math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))
    return result

def ave(x1,x2):
    result = (x1 + x2) / 2
    return result

def ai_tuning(ai, poses):
    x = poses[0][0]
    y = poses[1][0]
    if x < 0 and y < 0 and x > -13 and y > -20:
        ai = (20 - vel) * (1 - distance(x,y,0,0) / distance(0,0,-13,-20)) + vel
    elif x < -25.5 and y < -32.6 and x > -59 and y > -43:
        d0 = distance(-25.5, -32.6,-61, -43)
        d = distance(x, y, -25.5, -32.6)
        n = 4
        if d < d0 / n:
            ai = (17 - vel) * (n * d / d0) + vel
        else:
            # ai = (20 - vel) * (n / (1-n) * (d-d0/n) / d0)
            ai = 17 - (17 - vel) * ((d-d0/n) / (d0-d0/n))
    elif x < -110 and y > -84 and y < -11:
        ai = 10
    elif x > -105 and y < -9 and x < -60 and y > -24:
        d0 = distance(-105, -9,-60, -24)
        d = distance(x, y, -105, -9)
        n = 4
        if d < d0 / n:
            ai = (16 - vel) * (n * d / d0) + vel
        else:
            # ai = (20 - vel) * (n / (1-n) * (d-d0/n) / d0)
            ai = 16 - (16 - vel) * ((d-d0/n) / (d0-d0/n))
    elif x > -22 and y > -10:
        ai = 10
    else:
        ai = vel
    

    if ai > 20:
        ai = 20
    elif ai < vel:
        ai = vel
    return ai

def Look_Ahead_Distance(Lfc, poses):
    x = poses[0][0]
    y = poses[1][0]
    if x < 0 and y < 0 and x > -13 and y > -20:
        Lfc = 15
    print(Lfc)
    return Lfc

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, dt, poses):
        # print('poses2: ', poses)
        self.x = poses[0][0]
        self.y = poses[1][0]
        self.yaw = poses[2][0]
        self.v +=  a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)

def proportional_control(target, current):
    a = Kp * (target - current)

    return a

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        # if self.old_nearest_point_index is None:
        #     # search nearest point index
        
        # if self.old_nearest_point_index is None:
        dx = [state.rear_x - icx for icx in self.cx]
        dy = [state.rear_y - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        ind = np.argmin(d)
        self.old_nearest_point_index = ind    
            
        Lf = k * state.v + Lfc  # update look ahead distance

        # print('Lf: ', Lf)
        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind, poses):
    ind, Lf = trajectory.search_target_index(state)

    # if pind >= ind:
    #     ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    Lf = Look_Ahead_Distance(Lf, poses)
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw # theta
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0) # delta

    return delta, ind

class PureDriver:
    def __init__(self):
        self.speed = 0
        self.state = State(x=0, y=0, yaw=0, v=0)
        self.states = States()
        cx = []
        cy = []
        f = open('/home/seok-ju/seok_ws/src/MEN491_2021/f1tenth-riders-quickstart/pkg/src/pkg/Team_3/maps/track.csv','r', encoding='utf-8-sig')
        rdr = csv.reader(f)
        count = 0
        for line in rdr:
            if count <= 249:
                for j in range(0, 4):
                    cx.append(float(line[j]))
            else:
                for j in range(0, 4):
                    cy.append(float(line[j]))
            count += 1
        
        f.close()
    
        self.present_time = time.time()
        self.past_time = self.present_time
        self.target_course = TargetCourse(cx, cy)
        self.target_ind, _ = self.target_course.search_target_index(self.state)
        self.states.append(self.present_time, self.state)
        
    def process_lidar(self, ranges, poses):
        #  target course
        
        
        target_speed = vel  # [m/s]
        target_speed = ai_tuning(target_speed, poses)
        
        di, self.target_ind = pure_pursuit_steer_control(
            self.state, self.target_course, self.target_ind, poses)
        
        x = poses[0][0]
        y = poses[1][0]
        if np.abs(di) > 0.05:
            if x > -109.4 and y > -68.8 and x < -57 and y < -29:
                target_speed = vel * 2 / 3
            else:
                target_speed = vel * 1 / 2

        ai = proportional_control(target_speed, self.state.v)
        
        self.present_time = time.time()
        dt = self.present_time - self.past_time
        self.past_time = self.present_time
        self.state.update(ai, dt, poses)  # Control vehicle
        self.states.append(time, self.state)
        self.speed = self.state.v
        print(self.speed, di)
        return self.speed, di