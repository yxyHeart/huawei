#!/bin/bash
import sys
from typing import List
import math
import json
import numpy as np


def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()







def update(cur_angle,cur,workbench): # 返回下一帧的线速度和角速度 cur是（-pi，pi）的极角坐标 ，tar是目标的（x，y）  ,cur是当前的（x，y）
    def get_nearest(cur,workbench):
        min_dist = math.inf
        ans = -1
        
        for i,w in enumerate(workbench):
            if pow((w[1]-cur[0]),2)+pow((w[2]-cur[1]),2)<min_dist:
                ans = i
        return [workbench[ans][1],workbench[ans][2]]

    tar  = get_nearest(cur,workbench)
    angular_velocity=line_velocity=0
    def cartesian_to_polar(x, y): #转成极角坐标 (-pi,pi)范围之内
        angle = math.atan2(y, x)
        return angle
    
    tar_angle = cartesian_to_polar(tar[0]-cur[0],tar[1]-cur[1])
    def convert_angle(angle):
        """
        将角度从 (-pi, pi) 转换到 (0, 2pi) 范围内
        """
        angle = angle + 2 * np.pi
        angle = angle % (2 * np.pi)
        return angle
    cur_angle,tar_angle = convert_angle(cur_angle),convert_angle(tar_angle)


    diff_angle =tar_angle-cur_angle
    if 0 < abs(diff_angle) <= np.pi/2:
        line_velocity=(6*(1-diff_angle/(np.pi/2)))
        angular_velocity=(1 if diff_angle>0 else -1)*abs(diff_angle)
    elif np.pi/2 < abs(diff_angle) <= np.pi:
        line_velocity = 0
        angular_velocity=(1 if diff_angle>0 else -1)*abs(diff_angle)
    elif np.pi < abs(diff_angle) <= 2*np.pi:
        if 1.5*np.pi < abs(diff_angle) <= 2*np.pi:
            line_velocity =(6*(1-diff_angle/(np.pi/2)))
            angular_velocity=(-1 if diff_angle>0 else 1)*abs(diff_angle)
        elif 1*np.pi < abs(diff_angle) <= 1.5*np.pi:
            line_velocity =0
            angular_velocity=(-1 if diff_angle>0 else 1)*abs(diff_angle)

    return line_velocity,angular_velocity



line_speed = [0,0,0,0]
angle_speed = [0,0,0,0]

if __name__ == '__main__':
    read_util_ok()
    finish()
    while True:
        # 帧序号、当前金钱数
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])

        # 读取工作台数量
        line = sys.stdin.readline()
        workbenchs_cnt = int(line)

        #读取每个工作台的信息
        workbenchs_info = []
        for i in range(workbenchs_cnt):
            line = sys.stdin.readline()
            workbenchs_info.append(list(map(float,line.split(' '))))

        workbenchs = [[] for _ in range(10)] #1-9个工作台 0没有
        for w in workbenchs_info:
            w_id = int(w[0])
            workbenchs[w_id].append(w)


        #读取每个机器人的信息
        robot_info = []

        for i in range(4):
            line = sys.stdin.readline()
            robot_info.append(list(map(float,line.split(' '))))


        read_util_ok()

        sys.stdout.write('%d\n' % frame_id)

        # if frame_id<500:
        #     for i in range(4):
        #         line_speed[i],angle_speed[i] = update(robot_info[i][-3],[robot_info[i][-2],robot_info[i][-1]],workbenchs[5])
        # elif frame_id>=500:
        #     for i in range(4):
        #         line_speed[i],angle_speed[i] = update(robot_info[i][-3],[robot_info[i][-2],robot_info[i][-1]],workbenchs[5])
        # sys.stderr.write(json.dumps(workbenchs[1]))
        if robot_info[0][1]==float(0):
            target = np.random.randint(0,1,1)[0]+1
            line_speed[0],angle_speed[0] = update(robot_info[0][-3],[robot_info[0][-2],robot_info[0][-1]],workbenchs[target])
        else:
            target = 4
            line_speed[0],angle_speed[0] = update(robot_info[0][-3],[robot_info[0][-2],robot_info[0][-1]],workbenchs[target])



        line_speed[1],angle_speed[1] = update(robot_info[1][-3],[robot_info[1][-2],robot_info[1][-1]],workbenchs[5])
        line_speed[2],angle_speed[2] = update(robot_info[2][-3],[robot_info[2][-2],robot_info[2][-1]],workbenchs[5])
        line_speed[3],angle_speed[3] = update(robot_info[3][-3],[robot_info[3][-2],robot_info[3][-1]],workbenchs[5])

        
                

        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, line_speed[robot_id]))
            sys.stdout.write('buy %d\n' %(robot_id))
            sys.stdout.write('sell %d\n' %(robot_id))
            sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed[robot_id]))
        finish()
