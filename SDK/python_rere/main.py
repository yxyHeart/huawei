#!/bin/bash
import sys
from typing import List
import math
import json
import numpy as np
from collections import defaultdict,deque




def log(x):
    sys.stderr.write(str(x)+'\n')

def read_util_ok():
    while input()!="OK":
        pass

def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()



class Robot():
    def __init__(self,robot_id,robot_info=[0,0,0,0,0,0,0,0,0,0],workbenchs_type_to_id={}):
        self.robot_id = robot_id

        self.cur_workbench = int(robot_info[0]) #-1 0~工作台总数-1 为读地图时的工作台顺序
        self.type_of_carry = int(robot_info[1]) #0未携带 1-7为携带的物品
        self.time_value = robot_info[2]
        self.crash_value = robot_info[3]
        self.angle_speed = robot_info[-6]
        self.line_speed = [robot_info[-5],robot_info[-4]]
        self.cur_angle = robot_info[-3]
        self.cur_pos =[robot_info[-2],robot_info[-1]]

        # 线速度和前进速度不太一样？先不管
        self.forward_speed = 0

        self.workbenchs_type_to_id = workbenchs_type_to_id
        self.task = -1
        
        self.target = -1

        self.seller_dict = {
            1:[4,5,9],
            2:[4,6,9],
            3:[5,6,9],
            4:[7,9],
            5:[7,9],
            6:[7,9],
            7:[8,9],
        }
        self.needs_dict = {
            1:[],
            2:[],
            3:[],
            4:[1,2],
            5:[1,3],
            6:[2,3],
            7:[4,5,6],
            8:[7],
            9:[1,2,3,4,5,6,7]
        }

    def _distance(self,a,b):
        return math.sqrt(pow(a[1]-b[1],2)+pow(a[0]-b[0],2))
    
    def _cartesian_to_polar(self,x, y): #转成极角坐标 (-pi,pi)范围之内
        angle = math.atan2(y, x)
        return angle
    
    def _convert_angle(self,angle):
        # 将角度从 (-pi, pi) 转换到 (0, 2pi) 范围内
        angle = angle + 2 * np.pi
        angle = angle % (2 * np.pi)
        return angle
    
    def _update_speed(self):
        line_speed=angular_speed=0
        tar = self.workbenchs[self.target].pos
        tar_angle = self._cartesian_to_polar(tar[0]-self.cur_pos[0],tar[1]-self.cur_pos[1])
        cur_angle,tar_angle = self._convert_angle(self.cur_angle),self._convert_angle(tar_angle)
        diff_angle =tar_angle-cur_angle
        # log(diff_angle)
        if 0 < abs(diff_angle) <= np.pi/2:
            line_speed=(6*(1-diff_angle/(np.pi/2)))
            angular_speed=(1 if diff_angle>0 else -1)*abs(diff_angle)*3
        elif np.pi/2 < abs(diff_angle) <= np.pi:
            line_speed = 0
            angular_speed=(1 if diff_angle>0 else -1)*abs(diff_angle)*3

        elif 1.5*np.pi < abs(diff_angle) <= 2*np.pi:
            line_speed = (6*(1-(2*np.pi-abs(diff_angle))/(np.pi/2)))
            angular_speed=(-1 if diff_angle>0 else 1)*abs(2*np.pi-abs(diff_angle))*3
        elif 1*np.pi < abs(diff_angle) <= 1.5*np.pi:
            line_speed = 0
            angular_speed=(-1 if diff_angle>0 else 1)*abs(2*np.pi-abs(diff_angle))*3

        if angular_speed>np.pi:
            angular_speed=np.pi
        if angular_speed<-1*np.pi:
            angular_speed=-1*np.pi

        self.forward_speed,self.angle_speed = line_speed,angular_speed


    

    def _someone_need_this(self,need_type):
        #统计机器人将要运送的+已经在机器人手上的 是否小于 场上需要的
        onhand=will_buy=need_cnt=0
        for robot in self.robots:
            if robot_id!=self.robot_id:
                if robot.type_of_carry==need_type:
                    onhand+=1
                    
                elif robot.target!=-1 and robot.task == 1 and self.workbenchs[robot.target].type==need_type:
                    will_buy+=1
        for seller_type in self.seller_dict[need_type]:
            for wb_id in self.workbenchs_type_to_id[seller_type]:
                wb = self.workbenchs[wb_id]
                if need_type not in wb.need_status:
                    need_cnt+=1
        # log(need_cnt>onhand+will_buy)
        return need_cnt>onhand+will_buy

    def _find_the_wanted(self):

        q= [7,4,5,6]
        while len(q):
            next_q = []
            for node in q:
                cur_type = node
                for wb_id in self.workbenchs_type_to_id[cur_type]:
                    wb = self.workbenchs[wb_id]
                    if wb.product_status==1 and self._someone_need_this(wb.type) and wb.who_come_to_buy==-1:
                        return wb_id
                    else:
                        next_q.extend(list(set(self.needs_dict[cur_type])-set(wb.need_status)))
            q = next_q

        return -1
    def _find_the_seller(self):

        for seller_type in self.seller_dict[self.type_of_carry]:
            for wb_id in self.workbenchs_type_to_id[seller_type]:
                wb = self.workbenchs[wb_id]
                if self.type_of_carry not in wb.need_status and wb.who_deliver_n[self.type_of_carry]==-1:
                    return wb_id

        return -1 
                
    def update(self): # 返回下一帧的线速度和角速度 cur_angle是（-pi，pi）的极角坐标 ，tar是目标的（x，y） ,cur是当前的（x，y）

        
        # log(self.target)
        if self.target == -1 and self.type_of_carry==0:
            self.target = self._find_the_wanted()
            self.workbenchs[self.target].who_come_to_buy=self.robot_id
            self.task = 1

        if self.target == -1 and self.type_of_carry!=0:
            self.target = self._find_the_seller()
            self.workbenchs[self.target].who_deliver_n[self.type_of_carry]=self.robot_id
            self.task = 0

        if self._distance(self.workbenchs[self.target].pos,self.cur_pos)<0.4 and self.type_of_carry==0:
            sys.stdout.write('buy %d\n'%(self.robot_id))
            self.workbenchs[self.target].who_come_to_buy=-1
            self.target = -1
            self.task=-1
        if self._distance(self.workbenchs[self.target].pos,self.cur_pos)<0.4 and self.type_of_carry!=0:
            self.workbenchs[self.target].who_deliver_n[self.type_of_carry]=-1
            sys.stdout.write('sell %d\n'%(self.robot_id))
            self.target = -1
            self.task=-1
        if self.target!=-1:
            self._update_speed()



    def update_info(self,robot_info,workbenchs,robots):
        self.cur_workbench = int(robot_info[0]) #-1没有处于工作台 （0，8）为对应的工作台-1
        self.type_of_carry = int(robot_info[1]) #0未携带 1-7为携带的物品
        self.time_value = robot_info[2]
        self.crash_value = robot_info[2]
        self.angle_speed = robot_info[-6]
        self.line_speed = [robot_info[-5],robot_info[-4]]
        self.cur_angle = robot_info[-3]
        self.cur_pos = [robot_info[-2],robot_info[-1]]

        self.workbenchs = workbenchs
        self.robots = robots

class Workbench():
    def __init__(self,workbench_id=-1,workbench_info=[0,0,0,0,0,0]):
        self.workbench_id = workbench_id
        self.type = int(workbench_info[0])
        self.pos = [workbench_info[1],workbench_info[2]]
        self.rest_product_time = int(workbench_info[3])

        self.need_status = []
        for i in range(1,8):
            if int(workbench_info[4]>>i)&1:
                self.need_status.append(i)
        self.product_status = int(workbench_info[5])

        self.needs_dict = {
            1:[],
            2:[],
            3:[],
            4:[1,2],
            5:[1,3],
            6:[2,3],
            7:[4,5,6],
            8:[7],
            9:[1,2,3,4,5,6,7]
        }
        self.who_deliver_n = [-1]*8
        self.who_come_to_buy = -1


    def update(self,workbench_info=[0,0,0,0,0,0]):
        self.rest_product_time = int(workbench_info[3])
        self.need_status = []
        for i in range(1,8):
            if (int(workbench_info[4])>>i)&1:
                self.need_status.append(i)
        self.product_status = int(workbench_info[5])


        




def init():

    workbenchs_type_to_id = defaultdict(list)
    robots_pos = []
    workbenchs_init_info = []
    row=0
    workbenchs_id=0
    while True:
        line = input()
        if line =="OK":
            break

        line = list(line)
        for i,c in enumerate(line):
            if c=='A':
                robots_pos.append([0.25+0.5*i,49.75-0.5*row])
            elif c.isdigit():
                workbenchs_type_to_id[int(c)].append(workbenchs_id)
                workbenchs_init_info.append([workbenchs_id,int(c),0.25+0.5*i,49.75-0.5*row])
                workbenchs_id+=1
        row+=1
    robots = []
    for i in range(4):
        robots.append(Robot(robot_id=i,robot_info=[0,0,0,0,0,0,0,0,robots_pos[i][0],robots_pos[i][1]],workbenchs_type_to_id=workbenchs_type_to_id))

    workbenchs=[]
    for w in workbenchs_init_info:
        workbenchs.append(Workbench(workbench_id=w[0],workbench_info=[w[1],w[2],w[3],0,0,0]))

    return robots,workbenchs


if __name__ == '__main__':

    robots,workbenchs = init()
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

        #读取并更新每个工作台的信息
        workbenchs_info = []
        for i in range(workbenchs_cnt):
            line = sys.stdin.readline()
            workbenchs_info.append(list(map(float,line.split(' '))))

        for i,w in enumerate(workbenchs_info):
            workbenchs[i].update(w)

        
        #读取并更新每个机器人的信息
        robots_info = []

        for i in range(4):
            line = sys.stdin.readline()
            robots_info.append(list(map(float,line.split(' '))))

        for i,robot_info in enumerate(robots_info):
            robots[i].update_info(robot_info,workbenchs,robots)

        read_util_ok()
        sys.stdout.write('%d\n' % frame_id)



        robots[0].update()
        robots[1].update()
        robots[2].update()
        robots[3].update()

        if 1258<=frame_id<=1259:
            log(robots[1].target)
            log(workbenchs[11].who_come_to_buy)


                

        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, robots[robot_id].forward_speed))
            # sys.stdout.write('buy %d\n' %(robot_id))
            # sys.stdout.write('sell %d\n' %(robot_id))
            sys.stdout.write('rotate %d %f\n' % (robot_id, robots[robot_id].angle_speed))
        finish()
