# -*- coding: utf-8 -*-
"""
Created on Thu Jan 18 13:42:48 2024

@author: Jichen Zhu
"""

from src import cav_controller
from src import chv_prediction
from src import link_cav_cav
from src import link_cav_chv
from src import signal_controller

import numpy as np
import random
import traci
import optparse
from sumolib import checkBinary
import sys
import gurobipy
import bs4
from random import sample
import matplotlib.pyplot as plt

from time import time
import pandas as pd

delta_x = 1
sigma = 0.000000000000000001
vf = 50/3.6


# 进口道路段编号
in_edge = ['1','-2','3','-4']

directions = ['WE/L','WE/T','NS/L','NS/T'] #分别对应sumo中的绿灯相位编号0，2，4，6（因为sumo中有黄灯）

sumo_dir = 'env/test.sumocfg'
demand_dir = 'env/Demand_075.rou.xml'

#随机选取渗透率的车辆为CAV
demand_soup = bs4.BeautifulSoup(open(demand_dir),'html.parser')
od_soup = demand_soup.find_all('route')
vehicle_soup = demand_soup.find_all('vehicle')
total_vehicle_number = len(vehicle_soup)
p = 0.4 #渗透率
id_list = []
random.seed(1000)
for i in range(len(vehicle_soup)):
    id_list.append(vehicle_soup[i]['id'])
cav_total_list = sample(id_list, int(total_vehicle_number*p))
# print(cv_list)
print('渗透率为：'+str(p))

cav_list_pd = pd.DataFrame(cav_total_list)
cav_list_pd.to_csv(r'cav_total_list.csv')

#轨迹
for number in range(total_vehicle_number):
    #exec("global cav_" + str(number))
    exec("veh_" + str(number) + "= []")

def get_vehicle_position():
    run_id = traci.vehicle.getIDList()
    for v_id in run_id:
        position = traci.vehicle.getPosition(v_id)
        exec(v_id + ".append([position[0],position[1]])")


def judge_veh_in_signal_index(lane_id):
    # 返回车道对应的信号绿灯序号，这里的序号是不考虑黄灯的，到时候约束的时候会把黄灯加进去
    if lane_id == '1_2' or lane_id == '-2_2':
        direction = 0
    elif lane_id == '1_1' or lane_id == '-2_1':
        direction = 1
    elif lane_id == '3_2' or lane_id == '-4_2':
        direction = 2
    elif lane_id == '3_1' or lane_id == '-4_1':
        direction = 3
        
    return direction


def chv_leader_judege(adjacent_vehicles,chv_id,cav_list,chv_list):
    leader_flag = 1
    for link in adjacent_vehicles:
        if link[0] == chv_id and (link[1] in cav_list or link[1] in chv_list):
            leader_flag = 0
    return leader_flag

# 获取车辆（CAV or CHV）的参考轨迹
def get_v_t_r(td,lane_len,v_range,step):
    v_t_r = []
    v_v_r = []
    total_range = int(lane_len/delta_x)
    for x in range(int(total_range)):
        if x == 0:
            v_t_r.append(0) #第一个元素是通过每个断面点的时刻
            v_v_r.append(vf) #第二个元素是通过每个断面点的速度
        else:
            v_t_r.append(v_t_r[x-1]+delta_x/(v_v_r[x-1]+sigma)) #第一个元素是通过每个断面点的时刻
            v_v_r.append(vf) #第二个元素是通过每个断面点的速度
    
    current_time = step/100
    v_t_r = [t-current_time for t in v_t_r]
    
    v_t_r_new = v_t_r[total_range-v_range:]
    v_v_r_new = v_v_r[total_range-v_range:]
    
    
    return v_v_r_new,v_t_r_new

# 获取车辆（CAV or CHV）的初始轨迹，以vf速度行驶
def get_v_initial(v_range,v_v0,t0):
    v_t_initial = []
    v_v_initial = []
    for x in range(int(v_range)):
        if x == 0:
            v_t_initial.append(t0) #第一个元素是通过每个断面点的时刻
            v_v_initial.append(v_v0) #第二个元素是通过每个断面点的速度
        else:
            v_t_initial.append(v_t_initial[x-1]+delta_x/(v_v_initial[x-1]+sigma)) #第一个元素是通过每个断面点的时刻
            v_v_initial.append(vf) #第二个元素是通过每个断面点的速度
    return v_v_initial,v_t_initial


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

options = get_options()
# this script has been called from the command line. It will start sumo as a
# server, then connect and run
if options.nogui:
    sumoBinary = checkBinary('sumo')
else:
    sumoBinary = checkBinary('sumo-gui')

traci.start([sumoBinary, "-c", sumo_dir,
              "--device.emissions.probability", "1",
              "--tripinfo-output", 
              r"tripinfo_admm.xml",])
              # "--full-output",
              # "fulloutput_admm.xml"])

simulation_step = 1000*100 #以0.01s为步长，需要仿真时间为t，则共需要t*100时间步
step = 0


s_s = [0,25,50,75]
s_e = [22,47,72,97]


total_cav_solve_time = []
total_link_solve_time = []
total_signal_solve_time = []
total_a_list = []




while step < simulation_step:
    traci.simulationStep() #以0.01s的步长仿真，已在sumocfg文件中规定
    print(step)
    # print(traci.trafficlight.getNextSwitch('0'))
    
    run_id = traci.vehicle.getIDList()
    for v in run_id:
        traci.vehicle.setLaneChangeMode(v,0)
    
    last_s = s_s
    last_e = s_e
    
    
    if step%200 == 0:
        print('此时运行时间为'+str(step/100)+'s，进行优化')
        #每1s优化一次
        
        get_vehicle_position()
        
        t0 = 0
        current_phase = traci.trafficlight.getPhase('0')
        
        # optimize_cycle是以此时刻绿灯相位为第一相位，重新组成的未来周期。
        if current_phase == 0:
            optimize_cycle = [0,1,2,3]
            # next_switch_step = traci.trafficlight.getNextSwitch('0')
            # phase_duration = round(last_e[0]-last_s[0])
            # start_abs_step = next_switch_step-phase_duration
            # first_phase_start_time = round(t0 - (step/100-start_abs_step))
            first_phase_start_time = last_s[0]
        elif current_phase == 2:
            optimize_cycle = [1,2,3,0]
            # next_switch_step = traci.trafficlight.getNextSwitch('0')
            # phase_duration = round(last_e[0]-last_s[0])
            # start_abs_step = next_switch_step-phase_duration
            # first_phase_start_time = round(t0 - (step/100-start_abs_step))
            first_phase_start_time = last_s[0]
        elif current_phase == 4:
            optimize_cycle = [2,3,0,1]
            # next_switch_step = traci.trafficlight.getNextSwitch('0')
            # phase_duration = round(last_e[0]-last_s[0])
            # start_abs_step = next_switch_step-phase_duration
            # first_phase_start_time = round(t0 - (step/100-start_abs_step))
            first_phase_start_time = last_s[0]
        elif current_phase == 6:
            optimize_cycle = [3,0,1,2]
            # next_switch_step = traci.trafficlight.getNextSwitch('0')
            # phase_duration = round(last_e[0]-last_s[0])
            # start_abs_step = next_switch_step-phase_duration
            # first_phase_start_time = round(t0 - (step/100-start_abs_step))
            first_phase_start_time = last_s[0]
        
        #如果此时为黄灯，则以下一个绿灯相位作为绿灯相位
        elif current_phase == 1:
            optimize_cycle = [1,2,3,0]
            next_switch_step = traci.trafficlight.getNextSwitch('0')
            first_phase_start_time = t0 + (next_switch_step-step/100)
        elif current_phase == 3:
            optimize_cycle = [2,3,0,1]
            next_switch_step = traci.trafficlight.getNextSwitch('0')
            first_phase_start_time = t0 + (next_switch_step-step/100)
        elif current_phase == 5:
            optimize_cycle = [3,0,1,2]
            next_switch_step = traci.trafficlight.getNextSwitch('0')
            first_phase_start_time = t0 + (next_switch_step-step/100)
        elif current_phase == 7:
            optimize_cycle = [0,1,2,3]
            next_switch_step = traci.trafficlight.getNextSwitch('0')
            first_phase_start_time = t0 + (next_switch_step-step/100)
        
        
        
        
        run_id = traci.vehicle.getIDList()
        
        
        
        #对run_id进行调整，因为预测chv的时候，必须按照chv先后顺序进行预测，所以直接把run_id排序好
        number_list = []
        for i in range(len(run_id)):
            v_id = run_id[i]
            number = float(v_id[v_id.index('_')+1:])
            number_list.append(number)
        sort_list = sorted(number_list.copy())
        
        run_id_new = []
        for j in sort_list:
            run_id_new.append(run_id[number_list.index(j)])
        
        run_id = run_id_new
        
        # print(run_id)
        
        # 初始化ADMM需要的车辆初始信息
        # link
        adjacent_vehicles = []
        # CAV
        cav_list = []
        cav_x0_list = []
        cav_v0_list = []
        cav_phase_list = []
        cav_td_list = []
        cav_lane_len_list = []
        # CHV
        chv_list = []
        chv_x0_list = []
        chv_v0_list = []
        chv_phase_list = []
        chv_leader_flag_list = []
        chv_td_list = []
        chv_lane_len_list = []
        
        if len(run_id) > 0:
            for v in run_id:
                lane = traci.vehicle.getLaneIndex(v)
                edge = traci.vehicle.getRoadID(v)
                lane_id = traci.vehicle.getLaneID(v)
                if lane != 0 and edge in in_edge: #右转车不受控、出口道车辆不受控，故右转道和出口道车辆不考虑
                    if v in cav_total_list:
                        if traci.lane.getLength(traci.vehicle.getLaneID(v))-traci.vehicle.getLanePosition(v) >= 1:
                            #接近交叉口的车不再受控制
                            cav_list.append(v)
                            x0 = traci.lane.getLength(traci.vehicle.getLaneID(v))-traci.vehicle.getLanePosition(v)
                            cav_x0_list.append(x0)
                            v0 = traci.vehicle.getSpeed(v)
                            cav_v0_list.append(v0)
                            abs_phase_index = judge_veh_in_signal_index(lane_id)#得到的是按照信号灯运行的相位编号
                            rel_phase_index = optimize_cycle.index(abs_phase_index)
                            cav_phase_list.append(rel_phase_index)
                        
                            cav_td_list.append(traci.vehicle.getDeparture(v))
                            cav_lane_len_list.append(traci.lane.getLength(traci.vehicle.getLaneID(v)))
                            
                    else:
                        chv_list.append(v)
                        x0 = traci.lane.getLength(traci.vehicle.getLaneID(v))-traci.vehicle.getLanePosition(v)
                        chv_x0_list.append(x0)
                        v0 = traci.vehicle.getSpeed(v)
                        chv_v0_list.append(v0)
                        abs_phase_index = judge_veh_in_signal_index(lane_id)#得到的是按照信号灯运行的相位编号
                        rel_phase_index = optimize_cycle.index(abs_phase_index)
                        chv_phase_list.append(rel_phase_index)
                        
                        chv_td_list.append(traci.vehicle.getDeparture(v))
                        chv_lane_len_list.append(traci.lane.getLength(traci.vehicle.getLaneID(v)))
                        
                    # 构建所有两两相邻车的link的list，其中每一个元素表示一个link[后车，前车]
                    follower = traci.vehicle.getFollower(v, dist=300)[0]
                    if follower != '':
                        link = [follower,v]
                        adjacent_vehicles.append(link)
                    
            

                
                
            
            print('link: ' + str(adjacent_vehicles))
            print('在控制范围内的cav list: ' + str(cav_list))
            print('在控制范围内的chv_list: ' + str(chv_list))
        
            for chv_id in chv_list:
                leader_flag = chv_leader_judege(adjacent_vehicles,chv_id,cav_list,chv_list)
                chv_leader_flag_list.append(leader_flag)
            
            # CAV信息，reference轨迹
            cav_range_list = []
            cav_number = len(cav_list)
            for i in range(cav_number):
                cav_range_list.append(int(cav_x0_list[i]/delta_x)+1)
            cav_v_r_list = []
            cav_t_r_list = []
            cav_v_initial_list = []
            cav_t_initial_list = []
            for i in range(cav_number):
                cav_v_r_list.append(get_v_t_r(cav_td_list[i],cav_lane_len_list[i],cav_range_list[i],step)[0])
                cav_t_r_list.append(get_v_t_r(cav_td_list[i],cav_lane_len_list[i],cav_range_list[i],step)[1])
                
                cav_v_initial_list.append(get_v_initial(cav_range_list[i],cav_v0_list[i],t0)[0])
                cav_t_initial_list.append(get_v_initial(cav_range_list[i],cav_v0_list[i],t0)[1])
                
            # 存储所有cav的轨迹（初始存储cav_t_r和cav_v_r）
            cav_v_list = []
            cav_t_list = []
            cav_a_list = []
            for i in range(cav_number):
                cav_v_list.append(cav_v_initial_list[i])
                cav_t_list.append(cav_t_initial_list[i])
                cav_a_list.append(list(np.zeros(cav_range_list[i])))
            
            
            # CHV信息，reference轨迹
            chv_number = len(chv_list)
            chv_range_list = []
            for i in range(chv_number):
                chv_range_list.append(int(chv_x0_list[i]/delta_x)+1)
            chv_v_r_list = []
            chv_t_r_list = []
            chv_v_initial_list = []
            chv_t_initial_list = []
            for i in range(chv_number):
                chv_v_r_list.append(get_v_t_r(chv_td_list[i],chv_lane_len_list[i],chv_range_list[i],step)[0])
                chv_t_r_list.append(get_v_t_r(chv_td_list[i],chv_lane_len_list[i],chv_range_list[i],step)[1])
                
                chv_v_initial_list.append(get_v_initial(chv_range_list[i],chv_v0_list[i],t0)[0])
                chv_t_initial_list.append(get_v_initial(chv_range_list[i],chv_v0_list[i],t0)[1])
            # 存储所有chv的轨迹（初始存储chv_t_r和chv_v_r）
            chv_v_list = []
            chv_t_list = []
            for i in range(chv_number):
                chv_v_list.append(chv_v_initial_list[i])
                chv_t_list.append(chv_t_initial_list[i])
            
            
            # link信息
            # link_list每一项内的元素是指link中包含的CAV
            # 如果link有两辆CAV，则元素中包含两辆车，分别表示后车和前车在cav_list的位置
            # 如果link中后车为CAV，前车为CHV，则只有一个元素，即后车CAV在cav_list的位置。还需要外部对应到他前车的CHV编号以及IDM模型结果
            # 如果是CHV-CAV、CHV-CHV，则不存在link，直接由IDM模型预测
            link_list = []
            for i in range(len(adjacent_vehicles)):
                link = []
                if adjacent_vehicles[i][0] in cav_list:
                    if adjacent_vehicles[i][1] in cav_list or adjacent_vehicles[i][1] in chv_list:
                        link.append(cav_list.index(adjacent_vehicles[i][0]))
                        if adjacent_vehicles[i][1] in cav_list:
                            link.append(cav_list.index(adjacent_vehicles[i][1]))
                if link != []:
                    link_list.append(link)
            link_number = len(link_list)
            
            
            
            
            # signal初始信息
            s_s0 = [first_phase_start_time,first_phase_start_time+25,first_phase_start_time+50,first_phase_start_time+75]
            s_e0 = [first_phase_start_time+22,first_phase_start_time+47,first_phase_start_time+72,first_phase_start_time+97]
            
            
            # 更新信号配时方案优化结果
            s_s = s_s0
            s_e = s_e0
            
            # 存储每个controller每一步的计算时间
            cav_solve_time_list = []
            for i in range(cav_number):
                cav_solve_time_list.append([])
            
            link_solve_time_list = []
            for i in range(link_number):
                link_solve_time_list.append([])
            
            signal_solve_time_list = []
            
            
            # ADMM迭代
            max_iter = 100
            epslon = 0.5
            
            
            # 一致性变量（有n辆CAV，z_t就有n个，z_s_s和z_s_e各一个）
            z_t_total_cav_list = []
            for i in range(cav_number):
                z_t_total_cav_list.append(list(np.zeros(cav_range_list[i])))
            z_s_s = s_s0#list(np.zeros(4))
            z_s_e = s_e0#list(np.zeros(4))
            
            
            z_s_s_list = []
            z_s_e_list = []
            
            for i in range(cav_number):
                exec("z_t_%s_list = []"%(cav_list[i]))
                exec("z_t_%s_list.append(z_t_total_cav_list[%s])"%(cav_list[i],i))
            
            z_s_s_list.append(z_s_s)
            z_s_e_list.append(z_s_e)
            
            
            
            # 对偶变量
            # CAV controller
            # 每一辆CAV都要有
            lambda_t_total_cav_list = []
            lambda_s_s_total_cav_list = []
            lambda_s_e_total_cav_list = []
            
            for i in range(cav_number):
                lambda_t_total_cav_list.append(list(np.zeros(cav_range_list[i])))
                lambda_s_s_total_cav_list.append(list(np.zeros(4)))
                lambda_s_e_total_cav_list.append(list(np.zeros(4)))
            
            for i in range(cav_number):
                exec("lambda_t_%s_list = []"%(cav_list[i]))
                exec("lambda_t_%s_list.append(lambda_t_total_cav_list[%s])"%(cav_list[i],i))
                
                exec("lambda_s_s_%s_list = []"%(cav_list[i]))
                exec("lambda_s_s_%s_list.append(lambda_s_s_total_cav_list[%s])"%(cav_list[i],i))
                
                exec("lambda_s_e_%s_list = []"%(cav_list[i]))
                exec("lambda_s_e_%s_list.append(lambda_s_e_total_cav_list[%s])"%(cav_list[i],i))
            
            
            # link controller
            lambda_t_total_cav_link_list = []
            
            for i in range(link_number):
                lambda_t_total_cav_link_list.append([])
            
            for i in range(link_number):
                for j in link_list[i]:
                    # j是cav在cav_list中的编号
                    lambda_t_total_cav_link_list[i].append([list(np.zeros(cav_range_list[j]))])
            
            # lambda_t_total_cav_link_list = [lambda_t_total_cav_link_list]
            
            # for i in range(link_number):
            #     for j in link_list[i]:
            #         exec("lambda_t_%s_link_list = []"%(cav_list[j]))
            #         exec("lambda_t_%s_link_list.append(lambda_t_total_cav_link_list[i][link_list[i].index(j)])"%(cav_list[j]))
            
            
            # signal controller
            lambda_t_cav_signal_list = []
            lambda_s_s_signal_list = []
            lambda_s_e_signal_list = []
            
            for i in range(cav_number):
                lambda_t_cav_signal_list.append(list(np.zeros(cav_range_list[i])))
                
            lambda_s_s_signal_list.append(list(np.zeros(4)))
            lambda_s_e_signal_list.append(list(np.zeros(4)))
            
            
            lambda_t_total_cav_signal_list = [] #每个CAV的每次迭代的lambda都以一个list列表的形式存储
            for i in range(cav_number):
                lambda_t_total_cav_signal_list.append([])
                
            for i in range(cav_number):
                exec("lambda_t_total_cav_signal_list[i].append(lambda_t_cav_signal_list[i])")
            
            
            
            
            # 进入ADMM
            t_0 = time()
            a = 0
            
            for i in range(cav_number):
                exec("primal_error_t_%s_list = []" % (cav_list[i]))
            

            while a <= max_iter:
                a += 1
                print(a)
                
                
                # 每次迭代前，都根据上一次得到的一致性变量去预测CHV轨迹
                # 因此，cav的轨迹（cav_v_list和cav_t_list）和信号方案（s_s和s_e）在每次迭代后都要更新
                for i in range(len(chv_list)):
                    leader_flag = chv_leader_flag_list[i]
                    chv_v0 = chv_v0_list[i]
                    chv_x0 = chv_x0_list[i]
                    chv_range = chv_range_list[i]
                    phase_index = chv_phase_list[i]
                    if leader_flag == 1:
                        #此时CHV为头车，没有front_t和front_v，chv_prediction也不会用到这两个参数，因此赋予空表
                        front_v = []
                        front_t = []
                        chv_t_idm, chv_v_idm = chv_prediction.chv_prediction(phase_index,front_v,front_t,s_s,s_e,
                                                                                        chv_v0,chv_x0,t0,chv_range,
                                                                                        leader_flag)
                        
                    elif leader_flag == 0:
                        #此时CHV不是头车，要找到CHV为后车的那个初始link，并找到他的前车
                        for x in range(len(adjacent_vehicles)):
                            if adjacent_vehicles[x][0] == chv_list[i]:
                                front_vehicle = adjacent_vehicles[x][1]
                                if front_vehicle in cav_list:
                                    #前车为CAV
                                    front_index = cav_list.index(front_vehicle)
                                    front_v = cav_v_list[front_index].copy()
                                    front_t = cav_t_list[front_index].copy()
                                    #需要再把front_v和front_t转化成和后车chv一样大小的list
                                    for xx in range(chv_range_list[i]-cav_range_list[front_index]):
                                        front_v.insert(xx,vf) #假设前面这一段前车都按照vf行驶
                                        front_t.insert(xx,t0-(chv_range_list[i]-cav_range_list[front_index]-xx)/vf)
                                    chv_t_idm, chv_v_idm = chv_prediction.chv_prediction(phase_index,front_v,front_t,s_s,s_e,
                                                                                                    chv_v0,chv_x0,t0,chv_range,
                                                                                                    leader_flag)
                                    
                                else:
                                    #前车为CHV
                                    front_index = chv_list.index(front_vehicle)
                                    front_v = chv_v_list[front_index].copy()
                                    front_t = chv_t_list[front_index].copy()
                                    #需要再把front_v和front_t转化成和后车chv一样大小的list
                                    for xx in range(chv_range_list[i]-chv_range_list[front_index]):
                                        front_v.insert(xx,vf) #假设前面这一段前车都按照vf行驶
                                        front_t.insert(xx,t0-(chv_range_list[i]-chv_range_list[front_index]-xx)/vf)
                                    chv_t_idm, chv_v_idm = chv_prediction.chv_prediction(phase_index,front_v,front_t,s_s,s_e,
                                                                                                    chv_v0,chv_x0,t0,chv_range,
                                                                                                    leader_flag)
                    
                    # 更新chv的轨迹（chv_v_list和chv_t_list）
                    chv_v_list[i] = chv_v_idm
                    chv_t_list[i] = chv_t_idm
                    print('完成对车辆%s的IDM轨迹更新'%chv_list[i])
                
                # CAV模块
                for i in range(cav_number):
                    cav_id = cav_list[i]
                    
                    cav_x0 = cav_x0_list[i]
                    cav_v0 = cav_v0_list[i]
                    
                    exec("z_t_cav_list = z_t_%s_list"%cav_id)
                    exec("lambda_t_cav_list = lambda_t_%s_list"%cav_id)
                    exec("lambda_s_s_cav_list = lambda_s_s_%s_list"%cav_id)
                    exec("lambda_s_e_cav_list = lambda_s_e_%s_list"%cav_id)
                    exec("res_%s_t, res_%s_v, res_%s_a, res_%s_s_s, res_%s_s_e, solve_time = cav_controller.cav_controller(cav_id,cav_x0,cav_v0,t0,cav_phase_list[i],z_t_cav_list,lambda_t_cav_list,lambda_s_s_cav_list,z_s_s_list,lambda_s_e_cav_list,z_s_e_list,s_s0,s_e0,cav_v_list[i],cav_t_list[i],cav_a_list[i],round(s_s0[0]),round(s_e0[-1]))"
                         %(cav_id,cav_id,cav_id,cav_id,cav_id))
                    exec("cav_solve_time_list[%s].append(solve_time)"%i)
                    total_cav_solve_time.append(solve_time)
                    
                
                
                # link模块
                # 用于存储每个link的车的优化结果
                res_t_link = []
                for i in range(link_number):
                    res_t_link.append([])
                    
                for l in range(link_number):
                    # 判断进入CAV-CAV link controller
                    if len(link_list[l]) == 2:
                        link_cav_id = link_list[l]
                        
                        #link_cav1为后车，link_cav2为前车
                        link_cav1_x0 = cav_x0_list[link_cav_id[0]]
                        link_cav1_v0 = cav_v0_list[link_cav_id[0]]
                        z_t_link_cav1_list = 0
                        exec("z_t_link_cav1_list = z_t_%s_list"%cav_list[link_cav_id[0]])
                        lambda_t_cav1_link_list = lambda_t_total_cav_link_list[l][0]
                        
                        
                        link_cav2_x0 = cav_x0_list[link_cav_id[1]]
                        link_cav2_v0 = cav_v0_list[link_cav_id[1]]
                        z_t_link_cav2_list = 0
                        exec("z_t_link_cav2_list = z_t_%s_list"%cav_list[link_cav_id[1]])
                        lambda_t_cav2_link_list = lambda_t_total_cav_link_list[l][1]
                        
                        
                        res_cav1_t_link,res_cav2_t_link,solve_time = link_cav_cav.link_cav_cav(l,link_cav1_x0,link_cav1_v0,z_t_link_cav1_list,lambda_t_cav1_link_list,
                                                                                               link_cav2_x0,link_cav2_v0,z_t_link_cav2_list,lambda_t_cav2_link_list,
                                                                                               t0)
                             
                        res_t_link[l].append(res_cav1_t_link)
                        res_t_link[l].append(res_cav2_t_link)
                        
                        link_solve_time_list[l].append(solve_time)
                        total_link_solve_time.append(solve_time)
                        
                    # 判断进入CAV-CHV link controller
                    elif len(link_list[l]) == 1:
                        # 找到CAV前方的CHV编号，并找到该CHV的轨迹
                        link_cav_id = link_list[l][0]
                        for vehicles in adjacent_vehicles:
                            if vehicles[0] == cav_list[link_cav_id]:
                                link_chv_id = vehicles[1]
                        
                        chv_index = chv_list.index(link_chv_id)
                        
                        hv_t = chv_t_list[chv_index]
                        hv_range = chv_range_list[chv_index]
                        
                        link_cav_x0 = cav_x0_list[link_cav_id]
                        link_cav_v0 = cav_v0_list[link_cav_id]
                        
                        z_t_cav_list = 0
                        exec("z_t_cav_list =  z_t_%s_list"%cav_list[link_cav_id])
                        lambda_t_cav_link_list = lambda_t_total_cav_link_list[l][0]
                        
                        res_cav_t_link,solve_time = link_cav_chv.link_cav_chv(l,link_cav_x0,link_cav_v0,z_t_cav_list,lambda_t_cav_link_list,
                                                                               hv_t,hv_range,t0)
                        
                        
            
                        res_t_link[l].append(res_cav_t_link)
                        # res_t_link[l]=res_cav_t_link
                        
                        link_solve_time_list[l].append(solve_time)
                        total_link_solve_time.append(solve_time)
                
                
                
                # signal模块
                z_t_total_cav_list_total = []
                for i in range(cav_number):
                    exec("z_t_total_cav_list_total.append(z_t_%s_list)"%(cav_list[i]))
                res_cav_t_signal_list,res_u_s_s,res_u_s_e,solve_time = signal_controller.signal_controller(cav_number,cav_x0_list,
                                                                                                           cav_t_r_list,z_t_total_cav_list_total,lambda_t_total_cav_signal_list,
                                                                                                           z_s_s_list,lambda_s_s_signal_list,
                                                                                                           z_s_e_list,lambda_s_e_signal_list,
                                                                                                           round(s_s0[0]),round(s_e0[-1]),
                                                                                                           chv_list,chv_x0_list,chv_v0_list,chv_t_r_list,chv_phase_list,s_s)
                signal_solve_time_list.append(solve_time)
                total_signal_solve_time.append(solve_time)
                
                
                
                # 更新一致性变量
                
                # 首先判断cav的变量是否存在于link
                # 如果存在于link，那么有多项相加（cav、link、signal），一个cav可能存在于多个link中
                # 如果不存在，那么有2项相加（cav、signal）
                cav_in_link_num_list = []
                update_cav_z_link_list = []
                for i in range(cav_number):
                    cav_in_link_num = 0
                    update_z_link = 0
                    for l in range(link_number):
                        if i in link_list[l]:
                            cav_in_link_num += 1
                            cav_index = link_list[l].index(i)
                            update_z_link += np.array(res_t_link[l][cav_index])+np.array(lambda_t_total_cav_link_list[l][cav_index][-1])
                    
                    cav_in_link_num_list.append(cav_in_link_num)
                    update_cav_z_link_list.append(update_z_link)
                    
                
                for i in range(cav_number):
                    exec("z_t_%s_new = list((np.array(res_%s_t)+np.array(lambda_t_%s_list[-1])+np.array(update_cav_z_link_list[i])+np.array(res_cav_t_signal_list[i])+np.array(lambda_t_total_cav_signal_list[i][-1]))/(1+cav_in_link_num_list[i]+1))"
                             %(cav_list[i],cav_list[i],cav_list[i]))
                
                # 对于signal变量，共有cav_number+1(signal)项相加
                top_s = 0
                bottom_s = 0
                for i in range(cav_number):
                    exec("top_s += np.array(res_%s_s_s)+np.array(lambda_s_s_%s_list[-1])"%(cav_list[i],cav_list[i]))
                    bottom_s += 1
                top_s += np.array(res_u_s_s)+np.array(lambda_s_s_signal_list[-1])
                bottom_s += 1
                z_s_s_new = list(top_s/bottom_s)
                
                top_e = 0
                bottom_e = 0
                for i in range(cav_number):
                    exec("top_e += np.array(res_%s_s_e)+np.array(lambda_s_e_%s_list[-1])"%(cav_list[i],cav_list[i]))
                    bottom_e += 1
                top_e += np.array(res_u_s_e)+np.array(lambda_s_e_signal_list[-1])
                bottom_e += 1
                z_s_e_new = list(top_e/bottom_e)
                
                
                # 将更新后的一致性变量加入到储存z的list里面
                for i in range(cav_number):
                    exec("z_t_%s_list.append(z_t_%s_new)"%(cav_list[i],cav_list[i]))
                    #exec("print(z_t_%s_new)"%(cav_list[i]))
                z_s_s_list.append(z_s_s_new)
                z_s_e_list.append(z_s_e_new)
                
                
                
                
                
                # 更新对偶变量
                # CAV controller
                for i in range(cav_number):
                    exec("lambda_t_%s_new = np.array(lambda_t_%s_list[-1])+np.array(res_%s_t)-np.array(z_t_%s_list[-1])"%(cav_list[i],cav_list[i],cav_list[i],cav_list[i]))
                    exec("lambda_s_s_%s_new = np.array(lambda_s_s_%s_list[-1])+np.array(res_%s_s_s)-np.array(z_s_s_list[-1])"%(cav_list[i],cav_list[i],cav_list[i]))
                    exec("lambda_s_e_%s_new = np.array(lambda_s_e_%s_list[-1])+np.array(res_%s_s_e)-np.array(z_s_e_list[-1])"%(cav_list[i],cav_list[i],cav_list[i]))
                
                for i in range(cav_number):
                    exec("lambda_t_%s_list.append(lambda_t_%s_new)"%(cav_list[i],cav_list[i]))
                    exec("lambda_s_s_%s_list.append(lambda_s_s_%s_new)"%(cav_list[i],cav_list[i]))
                    exec("lambda_s_e_%s_list.append(lambda_s_e_%s_new)"%(cav_list[i],cav_list[i]))
            
                # link controller
                for l in range(link_number):
                    for v in range(len(link_list[l])):
                        lambda_t_cav_link_new = []
                        exec("lambda_t_cav_link_new = list(np.array(lambda_t_total_cav_link_list[l][v][-1])+np.array(res_t_link[l][v])-np.array(z_t_%s_list[-1]))"
                             %cav_list[link_list[l][v]])
                        lambda_t_total_cav_link_list[l][v].append(lambda_t_cav_link_new)
                        
                
                # signal controller
                for i in range(cav_number):
                    exec("lambda_t_%s_signal_new = np.array(lambda_t_total_cav_signal_list[i][-1])+np.array(res_cav_t_signal_list[i])-np.array(z_t_%s_list[-1])"%(cav_list[i],cav_list[i]))
                lambda_s_s_signal_new = np.array(lambda_s_s_signal_list[-1])+np.array(res_u_s_s)-np.array(z_s_s_list[-1])
                lambda_s_e_signal_new = np.array(lambda_s_e_signal_list[-1])+np.array(res_u_s_e)-np.array(z_s_e_list[-1])
                
                for i in range(cav_number):
                    exec("lambda_t_total_cav_signal_list[i].append(lambda_t_%s_signal_new)"%(cav_list[i]))
                lambda_s_s_signal_list.append(lambda_s_s_signal_new)
                lambda_s_e_signal_list.append(lambda_s_e_signal_new)
            
            
            
                # 更新cav的轨迹（cav_v_list和cav_t_list）
                for i in range(cav_number):
                    # exec("cav_v_list[i] = z_v_%s_new"%(cav_list[i]))
                    exec("cav_v_list[i] = res_%s_v"%(cav_list[i]))
                    exec("cav_t_list[i] = z_t_%s_list[-1]"%(cav_list[i]))
                    exec("cav_a_list[i] = res_%s_a"%(cav_list[i]))
            
            
                # 更新信号配时方案
                s_s = z_s_s_new
                s_e = z_s_e_new
                # # 因为s_e一般会收敛，所以根据s_e反推s_s
                # s_s = [s_s0[0],s_e[0]+3,s_e[1]+3,s_e[2]+3]
            
            
            
                # 计算primal error
                # CAV controller
                for i in range(cav_number):
                    exec("primal_error_t_%s = np.linalg.norm(list(np.array(res_%s_t)-np.array(z_t_%s_new)),ord=2)"
                         %(cav_list[i],cav_list[i],cav_list[i]))
                    exec("primal_error_s_s_%s = np.linalg.norm(list(np.array(res_%s_s_s)-np.array(z_s_s_new)),ord=2)"
                         %(cav_list[i],cav_list[i]))
                    exec("primal_error_s_e_%s = np.linalg.norm(list(np.array(res_%s_s_e)-np.array(z_s_e_new)),ord=2)"
                         %(cav_list[i],cav_list[i]))
                
                # link controller
                for l in range(link_number):
                    for v in range(len(res_t_link[l])):
                        exec("primal_error_t_%s_link%s = np.linalg.norm(list(np.array(res_t_link[l][v])-np.array(z_t_%s_new)),ord=2)"
                             %(cav_list[link_list[l][v]],l,cav_list[link_list[l][v]]))
                
                
                # for i in range(cav_number):
                #     if "lambda_t_%s_list"%(cav_list[i]) in globals():
                #         exec("primal_error_t_%s_link = np.linalg.norm(list(np.array(res_%s_t_link)-np.array(z_t_%s_new)),ord=2)"
                #              %(cav_list[i],cav_list[i],cav_list[i]))
                
                # signal controller
                for i in range(cav_number):
                    exec("primal_error_t_%s_signal = np.linalg.norm(list(np.array(res_cav_t_signal_list[i])-np.array(z_t_%s_new)),ord=2)"
                         %(cav_list[i],cav_list[i]))
                primal_error_s_s_signal = np.linalg.norm(list(np.array(res_u_s_s)-np.array(z_s_s_new)),ord=2)
                primal_error_s_e_signal = np.linalg.norm(list(np.array(res_u_s_e)-np.array(z_s_e_new)),ord=2)
                
                
                
                
                print('Primal residual:')
                print('CAV controller')
                for i in range(cav_number):
                    exec("print('%s_t: '+str(primal_error_t_%s))" % (cav_list[i],cav_list[i]))
                    exec("print('%s_s_s: '+str(primal_error_s_s_%s))" % (cav_list[i],cav_list[i]))
                    exec("print('%s_s_e: '+str(primal_error_s_e_%s))" % (cav_list[i],cav_list[i]))
                    print('---------------------------------------')
                
                
                print('Link controller')
                for l in range(link_number):
                    for v in range(len(res_t_link[l])):
                        exec("print('%s_t_link%s: '+str(primal_error_t_%s_link%s))" 
                             % (cav_list[link_list[l][v]],l,cav_list[link_list[l][v]],l))
                        
                # for i in range(cav_number):
                #     if "lambda_t_%s_list"%(cav_list[i]) in globals():
                #         exec("print('%s_t_link: '+str(primal_error_t_%s_link))" % (cav_list[i],cav_list[i]))
                # print('---------------------------------------')
                
                
                print('Signal controller')
                for i in range(cav_number):
                    exec("print('%s_t_s: '+str(primal_error_t_%s_signal))" % (cav_list[i],cav_list[i]))
                print('u_s_s: '+ str(primal_error_s_s_signal))
                print('u_s_e: '+ str(primal_error_s_e_signal))
                
                
                
                
                for i in range(cav_number):
                    exec("primal_error_t_%s_list.append(primal_error_t_%s)" % (cav_list[i],cav_list[i]))
                
                n = 0
                m = 0
                error = 0
                # CAV controller 的 primal error
                for i in range(cav_number):
                    # 如果误差小于可接受的误差，就输出
                    exec("error = primal_error_t_%s" % (cav_list[i]))
                    if error <= epslon:
                        n += 0
                    else:
                        n += 1
                    
                    # 如果连续两次的优化结果相差(二范数)小于0.1，那么也输出
                    exec("error = np.linalg.norm(list(np.array(z_t_%s_list[-2])-np.array(z_t_%s_list[-1])),ord=2)"
                          % (cav_list[i],cav_list[i]))
                    if error <= 0.1:
                        m += 0
                    else:
                        m += 1
                    
                    
                    exec("error = primal_error_s_s_%s" % (cav_list[i]))
                    if error <= epslon:
                        n += 0
                    else:
                        n += 1
                
                    exec("error = primal_error_s_e_%s" % (cav_list[i]))
                    if error <= epslon:
                        n += 0
                    else:
                        n += 1
                
                # # link controller 的 primal error
                # for i in range(cav_number):
                #     if "lambda_t_%s_list"%(cav_list[i]) in globals():
                #         exec("error = primal_error_t_%s_link" % (cav_list[i]))
                #         if error <= epslon:
                #             n += 0
                #         else:
                #             n += 1
                
                # signal controller 的 primal error
                # for i in range(cav_number):
                #     exec("error = primal_error_t_%s_signal" % (cav_list[i]))
                #     if error <= epslon:
                #         n += 0
                #     else:
                #         n += 1
                
                # error = primal_error_s_s_signal
                # if error <= epslon:
                #     n += 0
                # else:
                #     n += 1
                
                # error = primal_error_s_e_signal
                # if error <= epslon:
                #     n += 0
                # else:
                #     n += 1
                
                
                if n == 0 or m == 0:
                    break
            
            print('总迭代步数: '+str(a))
            total_a_list.append(a)
            #print('Primal residual: ' + str(primal_error_t_cav1_list[-1]))
            

                
            
            #再更新一次CHV轨迹
            for i in range(len(chv_list)):
                leader_flag = chv_leader_flag_list[i]
                chv_v0 = chv_v0_list[i]
                chv_x0 = chv_x0_list[i]
                chv_range = chv_range_list[i]
                phase_index = chv_phase_list[i]
                if leader_flag == 1:
                    #此时CHV为头车，没有front_t和front_v，chv_prediction也不会用到这两个参数，因此赋予空表
                    front_v = []
                    front_t = []
                    chv_t_idm, chv_v_idm = chv_prediction.chv_prediction(phase_index,front_v,front_t,s_s,s_e,
                                                                                    chv_v0,chv_x0,t0,chv_range,
                                                                                    leader_flag)
                    
                elif leader_flag == 0:
                    #此时CHV不是头车，要找到CHV为后车的那个初始link，并找到他的前车
                    for x in range(len(adjacent_vehicles)):
                        if adjacent_vehicles[x][0] == chv_list[i]:
                            front_vehicle = adjacent_vehicles[x][1]
                            if front_vehicle in cav_list:
                                #前车为CAV
                                front_index = cav_list.index(front_vehicle)
                                front_v = cav_v_list[front_index].copy()
                                front_t = cav_t_list[front_index].copy()
                                #需要再把front_v和front_t转化成和后车chv一样大小的list
                                for xx in range(chv_range_list[i]-cav_range_list[front_index]):
                                    front_v.insert(xx,vf) #假设前面这一段前车都按照vf行驶
                                    front_t.insert(xx,t0-(chv_range_list[i]-cav_range_list[front_index]-xx)/vf)
                                chv_t_idm, chv_v_idm = chv_prediction.chv_prediction(phase_index,front_v,front_t,s_s,s_e,
                                                                                                chv_v0,chv_x0,t0,chv_range,
                                                                                                leader_flag)
                                
                            elif front_vehicle in chv_list:
                                #前车为CHV
                                front_index = chv_list.index(front_vehicle)
                                front_v = chv_v_list[front_index].copy()
                                front_t = chv_t_list[front_index].copy()
                                #需要再把front_v和front_t转化成和后车chv一样大小的list
                                for xx in range(chv_range_list[i]-chv_range_list[front_index]):
                                    front_v.insert(xx,vf) #假设前面这一段前车都按照vf行驶
                                    front_t.insert(xx,t0-(chv_range_list[i]-chv_range_list[front_index]-xx)/vf)
                                chv_t_idm, chv_v_idm = chv_prediction.chv_prediction(phase_index,front_v,front_t,s_s,s_e,
                                                                                                chv_v0,chv_x0,t0,chv_range,
                                                                                                leader_flag)
                
                # 更新chv的轨迹（chv_v_list和chv_t_list）
                chv_v_list[i] = chv_v_idm
                chv_t_list[i] = chv_t_idm
                print('完成对人类驾驶车辆%s的IDM轨迹更新'%chv_list[i])
                # print(chv_v_list[i])
                # print(chv_t_list[i])
            
            
            print('---------------------------------------')
            print('各控制器的平均求解时间为：')
            for i in range(cav_number):
                avg_time = np.mean(cav_solve_time_list[i])
                print(cav_list[i] + ': ' + str(avg_time) + 's')
                
            for l in range(link_number):
                avg_time = np.mean(link_solve_time_list[l])
                print('link' + str(l) + ': ' + str(avg_time) + 's')
            
            avg_time = np.mean(signal_solve_time_list)
            print('signal: ' + str(avg_time) + 's')
            
            
            
            total_time = []
            for i in range(cav_number):
                total_time = total_time + cav_solve_time_list[i]
                
            for l in range(link_number):
                total_time = total_time + link_solve_time_list[l]
                
            total_time = total_time + signal_solve_time_list
            avg_time = np.mean(total_time)
            
            print('所有控制器的平均求解时间为：' + str(avg_time) + 's')
            
            print(s_s)
            print(s_e)
            
            
            # 根据z_t_cav_list反推z_v_cav_new
            for i in range(cav_number):
                exec("z_v_%s_new = [cav_v0_list[i]]"%(cav_list[i]))
                for x in range(cav_range_list[i]-1):
                    exec("next_v = delta_x/(z_t_%s_list[-1][x+1]-z_t_%s_list[-1][x])"%(cav_list[i],cav_list[i]))
                    exec("z_v_%s_new.append(next_v)"%(cav_list[i]))
                exec("cav_v_list[i] = z_v_%s_new"%(cav_list[i]))
            
            for i in range(cav_number):
                exec("z_a_%s_new = []"%(cav_list[i]))
                for x in range(cav_range_list[i]-1):
                    exec("next_a = (z_v_%s_new[x+1]-z_v_%s_new[x])/(z_t_%s_list[-1][x+1]-z_t_%s_list[-1][x])"%(cav_list[i],cav_list[i],cav_list[i],cav_list[i]))
                    exec("z_a_%s_new.append(next_a)"%(cav_list[i]))
                exec("cav_a_list[i] = z_a_%s_new"%(cav_list[i]))
                
            # last_cav_t_list = cav_t_list
            # last_cav_v_list = cav_v_list
                

    # 开始控制
    # 控制每辆车
    run_id = traci.vehicle.getIDList()

    for cav_id in cav_total_list:
        if cav_id not in cav_list and cav_id in run_id:
            #此时的CAV不存在于需要控制的list（cav_list），但仍存在于路网
            #将他的速度控制权交还给sumo
            traci.vehicle.setSpeed(cav_id,-1)
    
    
    for v_index in range(len(cav_list)):
        v = cav_list[v_index]
        phase = cav_phase_list[v_index]
        for x in range(len(cav_t_list[v_index])-1):
            if (step%200)/100 >= cav_t_list[v_index][x] and (step%200)/100 <= cav_t_list[v_index][x+1]:
                #此时需要执行第x个断面上的建议速度
                target_speed = cav_v_list[v_index][x+1]
                traci.vehicle.setSpeed(v,target_speed)
                print('编号为' + v +'的CAV目标速度为：'+str(target_speed))
                print('编号为' + v +'的CAV实际速度为：'+str(traci.vehicle.getSpeed(v)))
            
        sumo_control_flag = 0
        #如果此时为绿灯，且cav距离交叉口足够近
        if t0 > s_s[phase] and t0 < s_e[phase] and cav_x0_list[v_index] <= 120:
            for l in adjacent_vehicles:
                if l[0] == v:
                    leader = l[1]
                    if leader in chv_list:
                        leader_x0 = chv_x0_list[chv_list.index(leader)]
                        differ = cav_x0_list[v_index] - leader_x0
                    elif leader in cav_list:
                        leader_x0 = cav_x0_list[cav_list.index(leader)]
                        differ = cav_x0_list[v_index] - leader_x0
                    if differ > 15:
                        sumo_control_flag = 1
                else:
                    #没有前车，且此时为绿灯，且cav距离交叉口足够近，此时将cav控制权交还给sumo
                    sumo_control_flag = 1
        
        if sumo_control_flag == 1:
            print('此时将'+v+'控制权交还给CAV')
            traci.vehicle.setSpeed(v,-1)
    
    
    # 控制信号灯
    current_phase = traci.trafficlight.getPhase('0')
    if current_phase == 0:
        remain_duration = round(s_e[0],2)
        traci.trafficlight.setPhaseDuration('0',remain_duration)
        print('信号灯执行成功，当前绿灯相位还剩余'+str(remain_duration)+'s')
        print(s_s)
        print(s_e)
    elif current_phase == 2:
        remain_duration = round(s_e[0],2)
        traci.trafficlight.setPhaseDuration('0',remain_duration)
        print('信号灯执行成功，当前绿灯相位还剩余'+str(remain_duration)+'s')
        print(s_s)
        print(s_e)
    elif current_phase == 4:
        remain_duration = round(s_e[0],2)
        traci.trafficlight.setPhaseDuration('0',remain_duration)
        print('信号灯执行成功，当前绿灯相位还剩余'+str(remain_duration)+'s')
        print(s_s)
        print(s_e)
    elif current_phase == 6:
        remain_duration = round(s_e[0],2)
        traci.trafficlight.setPhaseDuration('0',remain_duration)
        print('信号灯执行成功，当前绿灯相位还剩余'+str(remain_duration)+'s')
        print(s_s)
        print(s_e)
    
    else:
        print('此时为黄灯，不进行控制')
        print(s_s)
        print(s_e)
    
    
    s_s = [t-0.01 for t in s_s]
    s_e = [t-0.01 for t in s_e]
    
    
    
    
    print('------------------------------------------------------------------')
    step += 1

traci.close()
sys.stdout.flush()



print('仿真运行结束，各控制器总体平均求解时间为：')
avg_time = np.mean(total_cav_solve_time)
print('cav' + ': ' + str(avg_time) + 's')
    
avg_time = np.mean(total_link_solve_time)
print('link' + str(l) + ': ' + str(avg_time) + 's')

avg_time = np.mean(total_signal_solve_time)
print('signal: ' + str(avg_time) + 's')

print('平均每次求解所需要的迭代次数为：' + str(np.mean(total_a_list)) + '次')



for number in range(total_vehicle_number):
    exec("veh_" + str(number)+"=pd.DataFrame(veh_" + str(number) + ")")
    exec("veh_" + str(number)+".to_csv(r'trajectory/veh_%i.csv'%number)")

total_cav_solve_time = pd.DataFrame(total_cav_solve_time)
total_cav_solve_time.to_csv(r'computation/cav_time.csv')

total_link_solve_time = pd.DataFrame(total_link_solve_time)
total_link_solve_time.to_csv(r'computation/link_time.csv')

total_signal_solve_time = pd.DataFrame(total_signal_solve_time)
total_signal_solve_time.to_csv(r'computation/signal_time.csv')


total_a_list = pd.DataFrame(total_a_list)
total_a_list.to_csv(r'computation/iteration.csv')

