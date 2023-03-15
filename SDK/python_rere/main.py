#!/bin/bash
import sys
import math
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
class Info():
    def __init__(self):
        # define robot move speed ,accelerate,radius ...and so on
        # 定义机器人移动极限速度、加速度等信息
        self.v_min = -0.5
        self.v_max = 3.0
        self.w_max = 50.0 * math.pi / 180.0
        self.w_min = -50.0 * math.pi / 180.0
        self.vacc_max = 0.5
        self.wacc_max = 30.0 * math.pi / 180.0
        self.v_reso = 0.5
        self.w_reso = 2 * math.pi / 180.0
        self.radius = 1.0
        self.dt = 0.8#wxw
        self.predict_time = 0
        self.goal_factor = 1.0
        self.vel_factor = 1.0
        self.traj_factor = 1.0


# 定义机器人运动模型
# 返回坐标(x,y),偏移角theta,速度v,角速度w
def motion_model(x, u, dt):
    # robot motion model: x,y,theta,v,w
    x[0] += u[0] * dt * math.cos(x[2])
    x[1] += u[0] * dt * math.sin(x[2])
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]
    return x

# 产生速度空间
def vw_generate(x, info):
    # generate v,w window for traj prediction
    Vinfo = [info.v_min, info.v_max,
             info.w_min, info.w_max]

    Vmove = [x[3] - info.vacc_max * info.dt,
             x[3] + info.vacc_max * info.dt,
             x[4] - info.wacc_max * info.dt,
             x[4] + info.wacc_max * info.dt]

    # 保证速度变化不超过info限制的范围
    vw = [max(Vinfo[0], Vmove[0]), min(Vinfo[1], Vmove[1]),
          max(Vinfo[2], Vmove[2]), min(Vinfo[3], Vmove[3])]
    return vw

# 依据当前位置及速度，预测轨迹
def traj_cauculate(x, u, info):
    ctraj = np.array(x)
    xnew = np.array(x)  # Caution!!! Don't use like this: xnew = x, it will change x value when run motion_modle below
    time = 0

    while time <= info.predict_time:  # preditc_time作用？循环40次
        xnew = motion_model(xnew, u, info.dt)
        ctraj = np.vstack((ctraj, xnew))
        time += info.dt#0.1

    return ctraj


def dwa_core(x, u, goal, info, obstacles):
    # the kernel of dwa
    vw = vw_generate(x, info)
    min_score = 10000.0

    # 速度v,w都被限制在速度空间里
    for v in np.arange(vw[0], vw[1], info.v_reso):
        for w in np.arange(vw[2], vw[3], info.w_reso):
            # cauculate traj for each given (v,w)
            ctraj = traj_cauculate(x, [v, w], info)
            # 计算评价函数
            goal_score = info.goal_factor * goal_evaluate(ctraj, goal)
            vel_score = info.vel_factor * velocity_evaluate(ctraj, info)
            traj_score = info.traj_factor * traj_evaluate(ctraj, obstacles, info)
            # 可行路径不止一条，通过评价函数确定最佳路径
            # 路径总分数 = 距离目标点 + 速度 + 障碍物
            # 分数越低，路径越优
            ctraj_score = goal_score + vel_score + traj_score
            # evaluate current traj (the score smaller,the traj better)
            if min_score >= ctraj_score:
                min_score = ctraj_score
                u = np.array([v, w])
    return u


# 距离目标点评价函数
def goal_evaluate(traj, goal):
    # cauculate current pose to goal with euclidean distance
    goal_score = math.sqrt((traj[-1, 0] - goal[0]) ** 2 + (traj[-1, 1] - goal[1]) ** 2)
    return goal_score


# 速度评价函数
def velocity_evaluate(traj, info):
    # cauculate current velocty score
    vel_score = info.v_max - traj[-1, 3]
    return vel_score


# 轨迹距离障碍物的评价函数
def traj_evaluate(traj, obstacles, info):
    # evaluate current traj with the min distance to obstacles
    min_dis = float("Inf")
    for i in range(len(traj)):
        for ii in range(len(obstacles)):
            current_dist = math.sqrt((traj[i, 0] - obstacles[ii, 0]) ** 2 + (traj[i, 1] - obstacles[ii, 1]) ** 2)

            if current_dist <= info.radius:
                return float("Inf")

            if min_dis >= current_dist:
                min_dis = current_dist

    return 1 / min_dis


# 生成包含障碍物的地图
def obstacles_generate():
    obstacles = np.array([[25.25, 45.25],
                          [25.25, 42.75],
                          [25.25, 40.25],])
    return obstacles


class Robot():
    def __init__(self,robot_id,robot_info=[0,0,0,0,0,0,0,0,0,0],workbenchs_type_to_id={},frame_id=0,info=Info()):
        self.robot_id = robot_id
        self.frame_id = frame_id
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

        self.info = info

    # def _avoid_crash(self):

    #     for robot in self.robots:
    #         if robot.robot_id !=self.robot_id:
    #             other_angle = self._convert_angle(robot.cur_angle)
    #             if self._distance(self.cur_pos,robot.cur_pos)<5:
    #                 if (abs(self.cur_angle)+abs(robot.cur_angle)+np.pi)%np.pi < 0.10:
    #                     self.angle_speed += 0.4*np.pi
    #                     robot.angle_speed -=0.4*np.pi
    #                     self.forward_speed-=0.5
    
    # def _avoid_crash(self):
    #     pass
    def _avoid_crash(self):
        x = np.array([self.cur_pos[0], self.cur_pos[1], self.cur_angle, 0, 0])
        u = np.array([0, 0])
        tar_pos = self.workbenchs[self.target].pos
        goal = np.array([tar_pos[0], tar_pos[1]])
        obstacles = []
        for robot in self.robots:
            if robot.robot_id != self.robot_id:
                obstacles.append(robot.cur_pos)
        obstacles = np.array(obstacles)
        log(obstacles)
        log(goal)
        u = dwa_core(x, u, goal, self.info, obstacles)
        x = motion_model(x, u, self.info.dt)
        log(x)
        self.forward_speed = 20*x[-2]
        self.angle_speed = 0.5*x[-1]

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
            x = abs(diff_angle)
            # line_speed=(6*(1-diff_angle/(np.pi/2)))
            # line_speed=6*(1/(x+0.15)-0.58)
            line_speed=6*(-(2*x/np.pi)**2+1)
            angular_speed=(1 if diff_angle>0 else -1)*x*3
        elif np.pi/2 < abs(diff_angle) <= np.pi:
            line_speed = 0
            angular_speed=(1 if diff_angle>0 else -1)*abs(diff_angle)*1.5

        elif 1.5*np.pi < abs(diff_angle) <= 2*np.pi:
            x = 2*np.pi-abs(diff_angle)
            line_speed = (6*(1-(2*np.pi-abs(diff_angle))/(np.pi/2)))
            # line_speed=6*(1/(x+0.15)-0.58)
            # line_speed=6*(-(2*x/np.pi)**2+1)
            angular_speed=(-1 if diff_angle>0 else 1)*x*3
        elif 1*np.pi < abs(diff_angle) <= 1.5*np.pi:
            line_speed = 0
            angular_speed=(-1 if diff_angle>0 else 1)*abs(2*np.pi-abs(diff_angle))*1.5

        if angular_speed>np.pi:
            angular_speed=np.pi
        if angular_speed<-1*np.pi:
            angular_speed=-1*np.pi

        line_speed *=self._distance(self.workbenchs[self.target].pos,self.cur_pos)*0.95
        self.forward_speed,self.angle_speed = line_speed,angular_speed


    

    def _someone_need_this(self,need_type):
        #统计机器人将要运送的+已经在机器人手上的 是否小于 场上需要的
        onhand=will_buy=need_cnt=0
        for robot in self.robots:
            # if self.frame_id==1866 and robot.robot_id==1:
            #     log('--')
            #     log(robot.task)
            #     log(robot.target)
            #     log(robot.workbenchs[robot.target].type)
            #     log('--')
            if robot.robot_id!=self.robot_id:
                
                if robot.type_of_carry==need_type:
                    onhand+=1
                    
                elif robot.target!=-1 and robot.task == 1 and self.workbenchs[robot.target].type==need_type:
                    will_buy+=1
        for seller_type in self.seller_dict[need_type]:
            for wb_id in self.workbenchs_type_to_id[seller_type]:
                wb = self.workbenchs[wb_id]
                if need_type not in wb.need_status and wb.who_deliver_n[need_type]==-1:
                    need_cnt+=1
        # log(need_cnt>onhand+will_buy)
        # if self.frame_id==1866:
        #     log("onhand")
        #     log(onhand)
        #     log("will_buy")
        #     log(will_buy)
        #     log('need_cnt')
        #     log(need_cnt)
        return need_cnt>onhand+will_buy

    def _find_the_wanted(self):
        q= [7,4,5,6]
        # weights = np.array([0.7, 0.1, 0.1, 0.1])
        # q = np.random.choice(q,size=len(q),replace=False,p=weights)
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
            #买的时候优先把当前 已经有配件的排在前面 已经有的配件越多 排在越前面 
            q = list(set(next_q))

        return -1
    

    def _find_the_seller(self):
        cur_can_sell = self.seller_dict[self.type_of_carry]

        #根据7需要的类型 进行特判 加速7的进度
        the_need_of_n = {4:0,5:0,6:0}
        if self.type_of_carry in (1,2,3):
            for wb_id_7 in self.workbenchs_type_to_id[7]:
                wb_7 = self.workbenchs[wb_id_7]
                for has_get in wb_7.need_status:
                    the_need_of_n[has_get]+=1
            cur_can_sell.sort(key=lambda x:the_need_of_n.get(x,0))

        #优先卖给位置上已经有东西的工作台
        for seller_type in cur_can_sell:
            for wb_id in self.workbenchs_type_to_id[seller_type]:
                wb = self.workbenchs[wb_id]
                if self.type_of_carry not in wb.need_status and len(wb.need_status)>=1\
                        and wb.who_deliver_n[self.type_of_carry]==-1:
                    return wb_id
        for seller_type in cur_can_sell:
            for wb_id in self.workbenchs_type_to_id[seller_type]:
                wb = self.workbenchs[wb_id]
                if self.type_of_carry not in wb.need_status \
                        and wb.who_deliver_n[self.type_of_carry]==-1:
                    return wb_id
        return -1 
                
    def update(self): # 返回下一帧的线速度和角速度 cur_angle是（-pi，pi）的极角坐标 ，tar是目标的（x，y） ,cur是当前的（x，y）

        
        li = [-1]*4
        if self.target == -1 and self.type_of_carry==0:
            li[0]=1
            self.target = self._find_the_wanted()
            if self.target!=-1:
                self.workbenchs[self.target].who_come_to_buy=self.robot_id
                self.task = 1

        elif self.target == -1 and self.type_of_carry!=0:
            li[1]=1
            self.target = self._find_the_seller()
            if self.target!=-1:
                self.workbenchs[self.target].who_deliver_n[self.type_of_carry]=self.robot_id
                self.task = 0

        # if self._distance(self.workbenchs[self.target].pos,self.cur_pos)<0.4 and self.type_of_carry==0 and self.task==1:
        elif self.cur_workbench==self.target and self.type_of_carry==0 and self.task==1:
            li[2]=1
            sys.stdout.write('buy %d\n'%(self.robot_id))
            self.workbenchs[self.target].who_come_to_buy=-1
            self.target = -1
            self.task=-1
        # if self._distance(self.workbenchs[self.target].pos,self.cur_pos)<0.4 and self.type_of_carry!=0 and self.task==0:
        elif self.cur_workbench==self.target and self.type_of_carry!=0 and self.task==0:
            li[3]=1
            self.workbenchs[self.target].who_deliver_n[self.type_of_carry]=-1
            sys.stdout.write('sell %d\n'%(self.robot_id))
            self.target = -1
            self.task=-1
        if self.target!=-1:
            self._update_speed()


        self._avoid_crash()

        # if self.frame_id==1866 and self.robot_id==3:
        #     # log(self._distance(self.workbenchs[self.target].pos,self.cur_pos))
        #     # log(self.type_of_carry)
        #     # log(self.task)
        #     log(self.target)
        #     log(li)
        #     log(self.workbenchs[14].who_deliver_n)
        #     log(self.workbenchs[16].who_deliver_n)
        # if self.frame_id==1866 and self.robot_id==1:
        #     # log(self._distance(self.workbenchs[self.target].pos,self.cur_pos))
        #     # log(self.type_of_carry)
        #     log(self.task)
        #     log(self.target)
        #     log(self.workbenchs[self.target].type)


    def update_info(self,robot_info,workbenchs,robots,frame_id):
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
        self.frame_id=frame_id

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
        robots.append(Robot(robot_id=i,
                            robot_info=[0,0,0,0,0,0,0,0,robots_pos[i][0],robots_pos[i][1]],
                            workbenchs_type_to_id=workbenchs_type_to_id,
                            frame_id=0
                            ))

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
            robots[i].update_info(robot_info,workbenchs,robots,frame_id)

        read_util_ok()
        sys.stdout.write('%d\n' % frame_id)



        robots[0].update()
        robots[1].update()
        robots[2].update()
        robots[3].update()

        # if 1513<=frame_id<=1515:
        #     # log(robots[3].type_of_carry)
        #     log(robots[3].target)
        #     # log(workbenchs[-2].who_deliver_n)
        #     # log(workbenchs[-1].who_come_to_buy)
        #     # log(workbenchs[-6].who_come_to_buy)
        #     # log(robots[3]._distance(robots[3].workbenchs[robots[3].target].pos,robots[3].cur_pos))
        #     log(robots[3]._find_the_wanted())


                

        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, robots[robot_id].forward_speed))
            # sys.stdout.write('buy %d\n' %(robot_id))
            # sys.stdout.write('sell %d\n' %(robot_id))
            sys.stdout.write('rotate %d %f\n' % (robot_id, robots[robot_id].angle_speed))
        finish()
