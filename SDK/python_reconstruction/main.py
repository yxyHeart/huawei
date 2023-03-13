#!/bin/bash
import sys
from typing import List
import math
import json
import numpy as np



def log(x):
    sys.stderr.write(str(x)+'\n')

def read_util_ok():
    while input()!="OK":
        pass

def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


class Robot():
    def __init__(self,robot_id,robot_info=[0,0,0,0,0,0,0,0,0,0]):
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

        #目标工作台的id
        self.target_id = -1 

        #当前是否有任务
        self.has_task = False
        
        # 当前是买任务还是卖任务 卖：0 买：1
        self.task_type = -1

        #买任务需要记住卖家
        self.buyer = -1

        #是去买的路上还是去卖的路上
        self.has_buy = False

        #记录谁收卖家的货
        self.comsumer = -1

        #卖货是去拿货的路上还是去卖货的路上
        self.has_carry = False

        #买东西要记住卖家
        self.provider = -1
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
    
    def _update_speed(self,workbenchs):
        line_speed=angular_speed=0
        tar = workbenchs[self.target_id].pos
        tar_angle = self._cartesian_to_polar(tar[0]-self.cur_pos[0],tar[1]-self.cur_pos[1])
        cur_angle,tar_angle = self._convert_angle(self.cur_angle),self._convert_angle(tar_angle)
        diff_angle =tar_angle-cur_angle
        # log(diff_angle)
        if 0 < abs(diff_angle) <= np.pi/2:
            line_speed=(6*(1-diff_angle/(np.pi/2)))
            angular_speed=(1 if diff_angle>0 else -1)*abs(diff_angle)*4
        elif np.pi/2 < abs(diff_angle) <= np.pi:
            line_speed = 0
            angular_speed=(1 if diff_angle>0 else -1)*abs(diff_angle)*3
        elif np.pi < abs(diff_angle) <= 2*np.pi:
            if 1.5*np.pi < abs(diff_angle) <= 2*np.pi:
                line_speed =(6*(1-diff_angle/(np.pi/2)))
                angular_speed=(-1 if diff_angle>0 else 1)*abs(diff_angle)*3
            elif 1*np.pi < abs(diff_angle) <= 1.5*np.pi:
                line_speed =0
                angular_speed=(-1 if diff_angle>0 else 1)*abs(diff_angle)*4
        self.forward_speed,self.angle_speed = line_speed,angular_speed

    def _find_nearest_target_sell(self,task,workbenchs,buyer_type):
        dist = math.inf
        target = -1
        for w in workbenchs:
            if w.type==buyer_type:
                cur_dist = self._distance(self.cur_pos,w.pos)
                if cur_dist<dist:
                    dist = cur_dist
                    target = w.workbench_id
        
        return target
    
    def _find_nearest_target_buy(self,task,workbenchs,to_buy_type):
        dist = math.inf
        target = -1
        for w in workbenchs:
            if w.type==to_buy_type:
                cur_dist = self._distance(self.cur_pos,w.pos)
                if cur_dist<dist:
                    dist = cur_dist
                    target = w.workbench_id
        
        return target
    
    def _someone_need_this(self,task,type_)->bool:
        cnt = 0
        for t in task.task_info:
            for k,v in task.needs_dict.items():
                if type_ in v:
                    if t[0]==k and t[1][v.index(type_)]==False and t[3][v.index(type_)]==-1:
                        cnt+=1
                    
        return cnt>=1

    
    def _get_task(self,task,workbenchs):
        for i,t in enumerate(task.task_info):
            if t[0]==7 and t[2]==1 and t[4]==-1:
                t[4]=self.robot_id
                self.comsumer = self._find_nearest_target_sell(task,workbenchs,8)
                task.task_info[self.comsumer][3][task.needs_dict[workbenchs[self.comsumer].type].index(t[0])]=self.robot_id
                self.has_task =True
                self.task_type=0
                self.target_id = i
                self.provider = i
                return 
        for i,t in enumerate(task.task_info):
            #优先卖货 卖货要判断有没有人收
            if t[0] in (4,5,6) and t[2]==1 and t[4]==-1 and self._someone_need_this(task,t[0]):
                # log(t[0])
                t[4]=self.robot_id
                to_sell_type = t[0]
                for k,v in task.needs_dict.items():
                    if to_sell_type in v:
                        # log(to_sell_type)
                        self.comsumer = self._find_nearest_target_sell(task,workbenchs,k)
                        # log(self.comsumer)
                        task.task_info[self.comsumer][3][task.needs_dict[workbenchs[self.comsumer].type].index(t[0])]=self.robot_id
                        self.has_task =True
                        self.task_type=0
                        self.target_id = i
                        self.provider = i
                        break
            else:
                for j,st in enumerate(t[1]):
                    if st==False and t[3][j]==-1:
                        # log(t)
                        t[3][j] = self.robot_id
                        to_buy_type = task.needs_dict[t[0]][j]
                        self.target_id = self._find_nearest_target_buy(task,workbenchs,to_buy_type)
                        self.has_task=True
                        self.task_type=1
                        self.buyer = i
                        # log(self.buyer)
                        break

            # log(self.has_task)
            if self.has_task:
                break
        if 0.8<self.time_value<0.93:
            sys.stdout.write("destroy %d\n" %(self.robot_id))
        if 0.8<self.crash_value<0.97:
            sys.stdout.write("destroy %d\n" %(self.robot_id))
                
    def update(self,task,workbenchs): # 返回下一帧的线速度和角速度 cur_angle是（-pi，pi）的极角坐标 ，tar是目标的（x，y） ,cur是当前的（x，y）
        

        #没有任务，就去领
        if not self.has_task:
            self._get_task(task,workbenchs)
            # log(self.has_task)
        # 有任务就更新速度
        elif self.has_task:
            self._update_speed(workbenchs)
        
        # 卖任务要先去拿货
        if self.task_type==0 and self.has_carry==False \
                and self._distance(self.cur_pos,workbenchs[self.provider].pos)<0.4:
            task.task_info[self.provider][4]=-1
            sys.stdout.write("buy %d\n" %(self.robot_id))
            self.has_carry = True
            self.target_id = self.comsumer
        #卖任务要去送货
        elif self.task_type==0 and self.has_carry==True \
                and self._distance(workbenchs[self.comsumer].pos,self.cur_pos)<0.4:
            try:
                task.task_info[self.comsumer][3][task.needs_dict[workbenchs[self.comsumer].type].index(self.type_of_carry)]=-1
            except Exception as e:
                log(e)

            sys.stdout.write("sell %d\n" %(self.robot_id))
            self.has_carry = False
            self.target_id = -1
            self.has_task=False
            self.comsumer = -1
            self.task_type=-1
            self.provider = -1
            
        # 买任务完成后需要先返回需要这个物品的点
        elif self.task_type==1 and self.has_buy==False \
                and self._distance(workbenchs[self.target_id].pos,self.cur_pos)<0.4:
            sys.stdout.write("buy %d\n" %(self.robot_id))
            #self.buyer要交货之后再清除
            self.has_buy = True
            self.target_id = self.buyer
        # 买回来再卖的任务
        elif self.task_type==1 and self.has_buy==True \
                and self._distance(workbenchs[self.buyer].pos,self.cur_pos)<0.4:
            try:
                task.task_info[self.buyer][3][task.needs_dict[workbenchs[self.buyer].type].index(self.type_of_carry)]=-1
            except Exception as e:
                log(e)
            sys.stdout.write("sell %d\n" %(self.robot_id))
            self.has_buy = False
            self.target_id = -1
            self.has_task=False
            self.buyer = -1
            self.task_type=-1




    def update_info(self,robot_info):
        self.cur_workbench = int(robot_info[0]) #-1没有处于工作台 （0，8）为对应的工作台-1
        self.type_of_carry = int(robot_info[1]) #0未携带 1-7为携带的物品
        self.time_value = robot_info[2]
        self.crash_value = robot_info[2]
        self.angle_speed = robot_info[-6]
        self.line_speed = [robot_info[-5],robot_info[-4]]
        self.cur_angle = robot_info[-3]
        self.cur_pos =[robot_info[-2],robot_info[-1]]

class Workbench():
    def __init__(self,workbench_id=-1,workbench_info=[0,0,0,0,0,0]):
        self.workbench_id = workbench_id
        
        self.type = int(workbench_info[0])
        self.pos = [workbench_info[1],workbench_info[2]]
        self.rest_product_time = int(workbench_info[3])
        self.need_status = int(workbench_info[4])
        self.product_status = int(workbench_info[5])


    def update_info(self,workbench_info=[0,0,0,0,0,0]):
        self.rest_product_time = int(workbench_info[3])
        self.need_status = int(workbench_info[4])
        self.product_status = int(workbench_info[5])

class Task():
    def __init__(self,workbenchs_init_info):
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
        #task的长度要和workbenchs的长度一致
        #type, [bool]*len(needs_dict[type]), bool是否有货, int 谁接了原料任务，int 谁接了卖货任务
        self.task_info= [] 
        for w in workbenchs_init_info:
            self.task_info.append([ w[1],
                                    [False]*len(self.needs_dict[w[1]]), 
                                    0,
                                    [-1]*len(self.needs_dict[w[1]]),
                                    -1
                                ])

    # 更新当前货物的状态
    def update_info(self,workbenchs_info):
        for i,w in enumerate(workbenchs_info):
            self.task_info[i][1] = [ True if (int(w[-2])>>need)&1 else False for need in self.needs_dict[self.task_info[i][0]] ]
            self.task_info[i][2] = int(w[-1])



def init():
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
                workbenchs_init_info.append([workbenchs_id,int(c),0.25+0.5*i,49.75-0.5*row])
                workbenchs_id+=1
        row+=1
    robots = []
    for i in range(4):
        robots.append(Robot(robot_id=i,robot_info=[0,0,0,0,0,0,0,0,robots_pos[i][0],robots_pos[i][1]]))

    workbenchs=[]
    for w in workbenchs_init_info:
        workbenchs.append(Workbench(workbench_id=w[0],workbench_info=[w[1],w[2],w[3],0,0,0]))

    task = Task(workbenchs_init_info)
    return robots,workbenchs,task


if __name__ == '__main__':

    robots,workbenchs,task = init()
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
            workbenchs[i].update_info(w)

        task.update_info(workbenchs_info)
        
        #读取并更新每个机器人的信息
        robots_info = []

        for i in range(4):
            line = sys.stdin.readline()
            robots_info.append(list(map(float,line.split(' '))))

        for i,r in enumerate(robots_info):
            robots[i].update_info(r)

        read_util_ok()
        sys.stdout.write('%d\n' % frame_id)



        robots[0].update(task,workbenchs)
        robots[1].update(task,workbenchs)
        robots[2].update(task,workbenchs)
        robots[3].update(task,workbenchs)

        if frame_id==3018:
            log(task.task_info)

                

        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, robots[robot_id].forward_speed))
            # sys.stdout.write('buy %d\n' %(robot_id))
            # sys.stdout.write('sell %d\n' %(robot_id))
            sys.stdout.write('rotate %d %f\n' % (robot_id, robots[robot_id].angle_speed))
        finish()
