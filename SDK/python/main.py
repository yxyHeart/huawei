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
        self.angle_speed = robot_info[-6]
        self.line_speed = [robot_info[-5],robot_info[-4]]
        self.cur_angle = robot_info[-3]
        self.cur_pos =[robot_info[-2],robot_info[-1]]

        # 线速度和前进速度不太一样？先不管
        self.forward_speed = 0

        #表示是否有目标
        self.has_target = False
        
        #当前目标的工作台
        self.target_workbench = -1

    def _make_decision_whether_to_buy(self,task,want_to_buy):
        #遍历当前想要买的物品，如果当前场上需要此物品的数量为0，就不买
        cnt=0
        for need in task.need_list:
            for i,n in enumerate(need[2]):
                if n==want_to_buy and need[3][i]==False:
                    cnt+=1
        return cnt>0

    #只有当前没有携带物品并且没有目标的时候，才要考虑去哪 

    def _make_decision_buy2(self,task,workbenchs)->int:
        ans=-1
        #先遍历45678需要的物品，然后找距离自己最近的那个
        arr = [4,5,6,7,8,9]

    '''
    买东西的逻辑要重写
    '''
    def _make_decision_buy(self,task,workbenchs)->int:
        ans = -1
        #应该先遍历 4567，看他们缺什么 ，缺什么搜什么 而不是嗯搜123
        #每个need应该有独立的编号，不然搜出来的错乱
        for need in task.need_list:
            find_flag = False
            arr = [4,5,6,7,8,9]
            np.random.shuffle(arr)
            if need[0] in arr:
                for i,n in enumerate(need[2]):
                    #当前需要这件物品并且没人选择
                    if not need[3][i] and need[4][i]==-1:
                        dist = math.inf
                        for k in range(len(workbenchs)):
                            if workbenchs[k].type==n:
                                cur_dist = math.sqrt(pow(self.cur_pos[0]-workbenchs[k].pos[0],2)+pow(self.cur_pos[1]-workbenchs[k].pos[1],2))
                                if cur_dist<dist:
                                    dist = cur_dist
                                    ans = workbenchs[k].workbench_id
                        if ans!=-1:
                            find_flag = True
                            break
                if find_flag:
                    break
        # log("buy")
        # log(ans)
        return ans

    #每一帧都要检查当前目标是否需要自己
    def _make_decision_sell_without_carry(self,task,workbenchs)->int:
        #先不判断9号
        ans = -1
        for can_sell in task.sell_dict[self.type_of_carry]:

            # 如果当前是可以卖的 并且当前workbench需要this
            #先遍历所有workbenchs，找到type是can_sell的，找到其中最短的一个

            dist = math.inf
            for k in range(len(workbenchs)):
                #当前是可以出售的目标并且当前目标需要这个并且没有机器人选择当前目标

                if workbenchs[k].type==can_sell \
                    and task.need_list[k][3][task.need_list[k][2].index(self.type_of_carry)]==False\
                    and task.need_list[k][4][task.need_list[k][2].index(self.type_of_carry)]==-1: 

                    cur_dist = math.sqrt(pow(self.cur_pos[0]-workbenchs[k].pos[0],2)+pow(self.cur_pos[1]-workbenchs[k].pos[1],2))
                    if cur_dist<dist:
                        dist = cur_dist
                        ans = workbenchs[k].workbench_id
                if ans!=-1:
                    break

        # log("sell")
        # log(ans)
        return ans
    


    
    def _make_decision_sell_with_carry(self,task,workbenchs):
        #先不判断9号
        ans = -1
        
        type_ = workbenchs[self.cur_workbench].type
        for can_sell in task.sell_dict[type_]:
            # 如果当前是可以卖的 并且当前workbench需要this
            #先遍历所有workbenchs，找到type是can_sell的，找到其中最短的一个

            dist = math.inf
            for k in range(len(workbenchs)):
                #当前是可以出售的目标并且当前目标需要这个并且没有机器人选择当前目标

                if workbenchs[k].type==can_sell \
                    and task.need_list[k][3][task.need_list[k][2].index(type_)]==False\
                    and task.need_list[k][4][task.need_list[k][2].index(type_)]==-1: 

                    cur_dist = math.sqrt(pow(self.cur_pos[0]-workbenchs[k].pos[0],2)+pow(self.cur_pos[1]-workbenchs[k].pos[1],2))
                    if cur_dist<dist:
                        dist = cur_dist
                        ans = workbenchs[k].workbench_id
                if ans!=-1:
                    break

        # log("sell")
        # log(ans)
        return ans


    def _distance(self,a,b):
        return math.sqrt(pow(a[1]-b[1],2)+pow(a[0]-b[0],2))
                
    def update(self,task,workbenchs): # 返回下一帧的线速度和角速度 cur_angle是（-pi，pi）的极角坐标 ，tar是目标的（x，y） ,cur是当前的（x，y）
        angular_velocity=line_velocity=0    
        # log("\n")
        # log(task.need_list)
        
        if self.type_of_carry==0 and not self.has_target:
            self.target_workbench = self._make_decision_buy(task,workbenchs)
            self.has_target = True

        #没有携带物品或者携带的物品已经被满足了
        if self.type_of_carry!=0 and not self.has_target:
            self.target_workbench = self._make_decision_sell_without_carry(task,workbenchs)
            if self.target_workbench == -1:
                self.forward_speed,self.angle_speed = 0,0
                return
            task.need_list[self.target_workbench][4][task.need_list[self.target_workbench][2].index(self.type_of_carry)] = self.robot_id
            self.has_target = True


        workbench = workbenchs[self.target_workbench]
        tar  = workbench.pos

        
        if self._distance(tar,self.cur_pos)<0.4 and self.type_of_carry==0:
            # 如果物品没有准备就绪就买其他的
            if workbenchs[self.cur_workbench].product_status==0:
                self.target_workbench = self._make_decision_buy(task,workbenchs)
                self.has_target = True
                workbench = workbenchs[self.target_workbench]
                tar = workbench.pos
            else:
                sys.stdout.write('buy %d\n' %(self.robot_id))
                self.has_target = False
                return 
        if self._distance(tar,self.cur_pos)<0.4 and self.type_of_carry!=0:
            #卖完要更新task.need_list 的状态
            try:
                task.need_list[self.cur_workbench][4][task.need_list[self.cur_workbench][2].index(self.type_of_carry)]=-1
            except Exception as e:
                log(e)
                log(self.type_of_carry)
                log(workbenchs[self.cur_workbench].type)
            sys.stdout.write('sell %d\n' %(self.robot_id))
            if workbenchs[self.target_workbench].product_status==1:
                sys.stdout.write('buy %d\n' %(self.robot_id))
                self.target_workbench = self._make_decision_sell_with_carry(task,workbenchs)
                self.has_target=True
                # log(self.target_workbench)
                return 

            self.has_target=False
            return 

        def cartesian_to_polar(x, y): #转成极角坐标 (-pi,pi)范围之内
            angle = math.atan2(y, x)
            return angle
        tar_angle = cartesian_to_polar(tar[0]-self.cur_pos[0],tar[1]-self.cur_pos[1])

        def convert_angle(angle):
            """
            将角度从 (-pi, pi) 转换到 (0, 2pi) 范围内
            """
            angle = angle + 2 * np.pi
            angle = angle % (2 * np.pi)
            return angle
        self.cur_angle,tar_angle = convert_angle(self.cur_angle),convert_angle(tar_angle)

        diff_angle =tar_angle-self.cur_angle
        if 0 < abs(diff_angle) <= np.pi/2:
            line_velocity=(6*(1-diff_angle/(np.pi/2)))
            angular_velocity=(1 if diff_angle>0 else -1)*abs(diff_angle)*4
        elif np.pi/2 < abs(diff_angle) <= np.pi:
            line_velocity = 0
            angular_velocity=(1 if diff_angle>0 else -1)*abs(diff_angle)*3
        elif np.pi < abs(diff_angle) <= 2*np.pi:
            if 1.5*np.pi < abs(diff_angle) <= 2*np.pi:
                line_velocity =(6*(1-diff_angle/(np.pi/2)))
                angular_velocity=(-1 if diff_angle>0 else 1)*abs(diff_angle)*3
            elif 1*np.pi < abs(diff_angle) <= 1.5*np.pi:
                line_velocity =0
                angular_velocity=(-1 if diff_angle>0 else 1)*abs(diff_angle)*4
        self.forward_speed,self.angle_speed = line_velocity,angular_velocity

    def update_info(self,robot_info):
        self.cur_workbench = int(robot_info[0]) #-1没有处于工作台 （0，8）为对应的工作台-1
        self.type_of_carry = int(robot_info[1]) #0未携带 1-7为携带的物品
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

    def update(self):
        pass

    def update_info(self,workbench_info=[0,0,0,0,0,0]):
        self.rest_product_time = int(workbench_info[3])
        self.need_status = int(workbench_info[4])
        self.product_status = int(workbench_info[5])

class Task():
    def __init__(self,workbenchs_pos):
        # sys.stderr.write(str(workbenchs_pos))
        self.nine_flag = False
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

        self.need_list = []

        for w in workbenchs_pos:
            if w[0]==9 and self.nine_flag==False:
                self.nine_flag=True
            # （工作台类型，工作台id，工作台需要什么，需要被满足的状态 ,当前是否有机器人选择了这个工作台的某件物品（选择了其他机器人就暂时不选了））
            # 应该是一个签名，true or false没法判断 签名为 0，1，2，3 -1表示没有签名
            self.need_list.append([ w[0],
                                    w[-1], 
                                    [*self.needs_dict[w[0]]], 
                                    [False for need in range(len(self.needs_dict[w[0]]))],
                                    [-1 for need in range(len(self.needs_dict[w[0]]))] 
                                ]) 
        self.sell_dict = {
            1:[4,5,9],
            2:[4,6,9],
            3:[5,6,9],
            4:[7,9],
            5:[7,9],
            6:[7,9],
            7:[8,9],
        } if self.nine_flag else {
            1:[4,5],
            2:[4,6],
            3:[5,6],
            4:[7],
            5:[7],
            6:[7],
            7:[8],
        }

    def update_info(self,workbenchs_info):
        for i,w in enumerate(workbenchs_info):
            self.need_list[i][3] = [ True if (int(w[-2])>>need)&1 else False for need in self.need_list[i][2] ]


        # log(self.need_list)




    def update(self,):
        pass


def init():
    robots_pos = []
    workbenchs_pos = []
    row=0
    workbenchs_cnt=0
    while True:
        line = input()
        if line =="OK":
            break
        # sys.stderr.write(line+'\n')
        line = list(line)
        
        for i,c in enumerate(line):
            
            if c=='A':
                robots_pos.append([0.25+0.5*i,49.75-0.5*row])
            elif c.isdigit():
                workbenchs_pos.append([int(c),0.25+0.5*i,49.75-0.5*row,workbenchs_cnt])
                workbenchs_cnt+=1
        row+=1
    robots = []
    for i in range(4):
        robots.append(Robot(robot_id=i,robot_info=[0,0,0,0,0,0,0,0,robots_pos[i][0],robots_pos[i][1]]))

    workbenchs=[]
    for w in workbenchs_pos:
        workbenchs.append(Workbench(workbench_id=w[-1],workbench_info=[w[0],w[1],w[2],0,0,0]))

    # sys.stderr.write(str(len(workbenchs_pos)))
    task = Task(workbenchs_pos)
    
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
        

        
                

        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, robots[robot_id].forward_speed))
            # sys.stdout.write('buy %d\n' %(robot_id))
            # sys.stdout.write('sell %d\n' %(robot_id))
            sys.stdout.write('rotate %d %f\n' % (robot_id, robots[robot_id].angle_speed))
        finish()
