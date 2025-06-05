# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 09:44:17 2023

@author: Jichen Zhu
"""


'''
CHV为后车，CAV为前车
当前即为一个信号周期刚开始的时刻（全局时钟=0）
'''

import numpy as np
import scipy.optimize
from math import log
from time import time
import gurobipy
import matplotlib.pyplot as plt


def cav_controller(cav_id,cav_x0,cav_v0,t0,cav_phase,
                   z_t_cav_list,lambda_t_cav_list,
                   lambda_s_s_cav_list,z_s_s_list,
                   lambda_s_e_cav_list,z_s_e_list,
                   s_s0,s_e0, res_cav_v_last, res_cav_t_last, res_cav_a_last,
                   start_time,end_time):
    
    print('Solving %s model......'%cav_id)
    # print(res_cav_v_last)
    # print(res_cav_t_last)
    
    vf = 50/3.6
    delta_x = 1
    sigma = 0.000000000000000001
    M = 1/sigma
    
    g_max = 40
    g_min = 10
    C = 100
    l = 3 #inter green time
    
    
    
    #构造reference轨迹
    cav_t_r = []
    cav_v_r = []
    cav_range = int(cav_x0/delta_x)+1 #第一个是当前的状态
    for x in range(int(cav_range)):
        if x == 0:
            cav_t_r.append(0) #第一个元素是通过每个断面点的时刻
            cav_v_r.append(cav_v0) #第二个元素是通过每个断面点的速度
        else:
            cav_t_r.append(cav_t_r[x-1]+delta_x/(cav_v_r[x-1]+sigma)) #第一个元素是通过每个断面点的时刻
            cav_v_r.append(vf) #第二个元素是通过每个断面点的速度
    
    
    
    len_to_stop = cav_range
    min_travel_time = len_to_stop/10
    if s_e0[cav_phase] <= min_travel_time: #对应相位的终止时间小于最快速度行驶到停车线的时间
        if cav_x0 <= 50:
            current_pass = 1 #此时距离交叉口已经比较近了，要求他必须这个周期过去
        else:
            current_pass = 0 #当前周期内的绿灯相位无法通过
    else:
        current_pass = 1 #可以在当前周期内的绿灯相位通过
    
    print('%s 当前周期内绿灯相位通过的状态为：'%cav_id + str(current_pass))

    # 目标函数
    # CAV模块
    
    
    # 目标函数1为控制轨迹与reference轨迹二范数最小（Li 2020，Liu 2022）
    def J_v1_t(cav_t): 
        value = 0
        # for x in range(len(cav_t)):
        #     value = value + (cav_t[x]-cav_t_r[x])
        value = cav_t[len(cav_t_r)-1]-cav_t_r[len(cav_t_r)-1]
        return value
    
    
    
    #目标函数2为车辆的加速度的绝对值之和最小（Ma 2021）
    def J_v2_smooth(cav_a_abs):
        value = 0
        for x in range(len(cav_a_abs)):
            value = value + cav_a_abs[x]
        return value
    
    
    
    def dynamic_t_cav(r1,r2):
        value = 0
        for x in range(cav_range-1):
            value = value + 10000*((r1[x] - r2[x]- delta_x)**2)
        return value
    
    
    
    def L_t_cav(cav_t):
        value = 0
        for x in range(len(cav_t)):
            value = value + (cav_t[x]-z_t_cav_list[-1][x]+lambda_t_cav_list[-1][x])**2
        return value
    
    def L_s_s_cav(cav_s_s):
        value = 0
        for x in range(len(cav_s_s)):
            value = value + (cav_s_s[x]-z_s_s_list[-1][x]+lambda_s_s_cav_list[-1][x])**2
        return value
    
    def L_s_e_cav(cav_s_e):
        value = 0
        for x in range(len(cav_s_e)):
            value = value + (cav_s_e[x]-z_s_e_list[-1][x]+lambda_s_e_cav_list[-1][x])**2
        return value
    



    
    #求解CAV模型
    max_time = 0.5
    rho = 200
    

    #求解CAV模型
    cav_model = gurobipy.Model("cav_model")
    # cav_model.Params.NonConvex = 2
    cav_model.setParam('Timelimit', max_time)
    
    cav_t = cav_model.addVars(range(0,len(cav_t_r)),lb=0,vtype=gurobipy.GRB.CONTINUOUS, name='cav_t')
    
    cav_v = cav_model.addVars(range(0,len(cav_v_r)),lb=0,ub=60/3.6,vtype=gurobipy.GRB.CONTINUOUS, name='cav_v')
    # cav_pi = cav_model.addVars(range(0,len(cav_v_r)),lb=1/16.67,vtype=gurobipy.GRB.CONTINUOUS, name='cav_pi')
    
    cav_a = cav_model.addVars(range(0,len(cav_v_r)-1),lb=-3,ub=3,vtype=gurobipy.GRB.CONTINUOUS, name='cav_a')
    cav_a_abs = cav_model.addVars(range(0,len(cav_v_r)-1),lb=0,ub=3,vtype=gurobipy.GRB.CONTINUOUS, name='abs_a')
    
    cav_s_s = cav_model.addVars(range(0,4),lb=-30,vtype=gurobipy.GRB.INTEGER, name='cav_s_s')
    cav_s_e = cav_model.addVars(range(0,4),lb=-1,vtype=gurobipy.GRB.INTEGER, name='cav_s_e')
    
    r1_cav_t = cav_model.addVars(range(0,cav_range-1),vtype=gurobipy.GRB.CONTINUOUS, name='r1_t')
    r2_cav_t = cav_model.addVars(range(0,cav_range-1),vtype=gurobipy.GRB.CONTINUOUS, name='r2_t')
    
    # b1 = cav_model.addVars(range(0,cav_range-1),lb=0,vtype=gurobipy.GRB.CONTINUOUS, name='b1')
    # b2 = cav_model.addVars(range(0,cav_range-1),lb=0,vtype=gurobipy.GRB.CONTINUOUS, name='b2')
        
    cav_model.addConstr(cav_t[0] == t0)
    
    cav_model.addConstr(cav_v[0] == cav_v0)
    # cav_model.addConstr(cav_pi[0] == 1/cav_v0)
    
    for x in range(cav_range-1):

        #System Dynamic
        #物理模型
        #约束1
        cav_model.addConstr(r1_cav_t[x] == cav_t[x+1]*res_cav_v_last[x])
        cav_model.addConstr(r2_cav_t[x] == cav_t[x]*res_cav_v_last[x])
        #约束2
        #泰勒展开
        cav_model.addConstr(2*res_cav_v_last[x+1]*cav_v[x+1]
                            -2*res_cav_v_last[x]*cav_v[x]
                            -res_cav_v_last[x+1]**2
                            +res_cav_v_last[x]**2 
                            == 2*cav_a[x]*delta_x)
        
        #微分动力学模型
        #约束1
        # cav_model.addConstr(cav_t[x+1] == cav_t[x]+cav_pi[x]*delta_x)
        #约束2
        #上一次结果直接带入
        # cav_model.addConstr(cav_pi[x+1]-cav_pi[x] == -((1/res_cav_v_last[x])**3)*cav_a[x]*delta_x)
        # cav_model.addConstr(b1[x] == cav_pi[x+1]-cav_pi[x])
        # cav_model.addConstr(b2[x] == -((1/res_cav_v_last[x])**3)*cav_a[x]*delta_x)
        #泰勒展开
        # c1 = (1/res_cav_v_last[x])**3*res_cav_a_last[x]
        # c2 = 3*((1/res_cav_v_last[x])**2)*res_cav_a_last[x]
        # c3 = (1/res_cav_v_last[x])**3
        # cav_model.addConstr(b1[x] == cav_pi[x+1]-cav_pi[x])
        # cav_model.addConstr(b2[x] == -delta_x*(c1+c2*(cav_pi[x]-1/res_cav_v_last[x])+c3*(cav_a[x]-res_cav_a_last[x])))
        
        
        
        cav_model.addGenConstrAbs(cav_a_abs[x], cav_a[x])
        
    #Green pass
    if current_pass == 1:
        cav_model.addConstr(cav_t[len(cav_t_r)-1] >= cav_s_s[cav_phase]-1)
        cav_model.addConstr(cav_t[len(cav_t_r)-1] <= cav_s_e[cav_phase]-2)
    else:
        cav_model.addConstr(cav_t[len(cav_t_r)-1] >= s_s0[cav_phase]+C)
        cav_model.addConstr(cav_t[len(cav_t_r)-1] <= s_e0[cav_phase]+C)
        
    #Signal plan
    for p in range(4):
        cav_model.addConstr(cav_s_e[p] >= cav_s_s[p] + g_min)
        cav_model.addConstr(cav_s_e[p] <= cav_s_s[p] + g_max)
    for p in range(3):
        cav_model.addConstr(cav_s_s[p+1] == cav_s_e[p] + l)
    cav_model.addConstr(cav_s_s[0] == start_time)
    cav_model.addConstr(cav_s_e[3] == end_time)
    
    cav_model.setObjective(10*J_v1_t(cav_t)
                           +1*J_v2_smooth(cav_a_abs)
                           # +10000*sum((b1[x]-b2[x])**2 for x in range(cav_range-1))
                            +dynamic_t_cav(r1_cav_t,r2_cav_t)
                            +rho/2*L_t_cav(cav_t)
                            +rho/2*L_s_s_cav(cav_s_s)
                            +rho/2*L_s_e_cav(cav_s_e)
                           ,
                           sense=gurobipy.GRB.MINIMIZE)
    
    cav_model.setParam('OutputFlag', 0)
    cav_model.params.Method = 2
    

    t0 = time()
    cav_model.optimize()
    t1 = time()
    solve_time = t1-t0
    print('求解时间为： '+str(solve_time)+'s')
    
    res_cav_t = []
    res_cav_v = []
    res_cav_a = []
    res_cav_s_s = []
    res_cav_s_e = []
    res_cav_a_abs = []
    # b1 = []
    # b2 = []
    res_r1_cav_t = []
    res_r2_cav_t = []


    
    
    for item in cav_model.getVars():
        if 'cav_t' in str(item.varName):
            res_cav_t.append(item.x)
        elif 'cav_v' in str(item.varName):
            res_cav_v.append(item.x)
        elif 'cav_a' in str(item.varName):
            res_cav_a.append(item.x)
        elif 'cav_s_s' in str(item.varName):
            res_cav_s_s.append(item.x)
        elif 'cav_s_e' in str(item.varName):
            res_cav_s_e.append(item.x)
        elif 'abs_a' in str(item.varName):
            res_cav_a_abs.append(item.x)
        elif 'r1_t' in str(item.varName):
            res_r1_cav_t.append(item.x)
        elif 'r2_t' in str(item.varName):
            res_r2_cav_t.append(item.x)
        # elif 'b1' in str(item.varName):
        #     b1.append(item.x)
        # elif 'b2' in str(item.varName):
        #     b2.append(item.x)
            
    # print(res_cav_t)
    # print(res_cav_v)
    # print(res_cav_a)
    print(dynamic_t_cav(res_r1_cav_t,res_r2_cav_t))
    # print(10000*sum((b1[x]-b2[x])**2 for x in range(cav_range-1)))



    return res_cav_t, res_cav_v, res_cav_a, res_cav_s_s,res_cav_s_e, solve_time
