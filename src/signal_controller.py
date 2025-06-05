# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 11:42:25 2023

@author: Jichen Zhu
"""

import numpy as np
import scipy.optimize
from math import log
from time import time
import gurobipy
import matplotlib.pyplot as plt

def signal_controller(cav_number,cav_x0_list,
                      cav_t_r_list,z_t_total_cav_list,lambda_t_total_cav_signal_list,
                      z_s_s_list,lambda_s_s_signal_list,
                      z_s_e_list,lambda_s_e_signal_list,
                      start_time,end_time,
                      chv_list,chv_x0_list,chv_v0_list,chv_t_r_list,chv_phase_list,s_s):
    
    vf = 50/3.6
    g_max = 40
    g_min = 10
    C = 100
    l = 3 #inter green time
    
    
    # 简化预测CHV的arrival time
    ta_chv_list = []
    for i in range(len(chv_list)):
        if chv_v0_list[i] > 1:
            ta = chv_x0_list[i]/chv_v0_list[i]
        else:
            ta = s_s[chv_phase_list[i]]+(chv_x0_list[i]/8)*2
        ta_chv_list.append(ta)
    
    
    
    
    
    # 信号灯优化模块
    # 目标函数
    # 计算每辆车的延误（CAV），最后要加起来
    def J_s(v_t_s,v_t_r):
        fftt = v_t_r[-1]
        actual_t = v_t_s[len(v_t_s)-1] #先试一下一致性变量为整个状态量，然后再尝试一下只让状态量的最后一个值作为一致性变量
        d = actual_t-fftt
        return d
    
    def J_s_chv(v_t_d,v_t_r):
        fftt = v_t_r[-1]
        actual_t = v_t_d
        d = actual_t-fftt
        return d

    def L_t_cav_signal(cav_t_s,z_t_cav_list,lambda_t_cav_signal_list):
        value = 0
        for x in range(len(cav_t_s)):
            value = value + (cav_t_s[x]-z_t_cav_list[-1][x]+lambda_t_cav_signal_list[-1][x])**2
        return value


    def L_s_s_signal(u_s_s,z_s_s_list,lambda_s_s_signal_list):
        value = 0
        for x in range(len(u_s_s)):
            value = value + (u_s_s[x]-z_s_s_list[-1][x]+lambda_s_s_signal_list[-1][x])**2
        return value

    def L_s_e_signal(u_s_e,z_s_e_list,lambda_s_e_signal_list):
        value = 0
        for x in range(len(u_s_e)):
            value = value + (u_s_e[x]-z_s_e_list[-1][x]+lambda_s_e_signal_list[-1][x])**2
        return value
    
    
    #求解Signal模型
    max_time = 0.5
    rho = 200
    
    signal_model = gurobipy.Model("signal_model")
    #signal_model.Params.NonConvex = 2
    signal_model.setParam('Timelimit', max_time)
    
    
    for i in range(cav_number):
        exec("cav%d_t_s = signal_model.addVars(range(0,len(cav_t_r_list[i])),lb=0,vtype=gurobipy.GRB.CONTINUOUS, name='cav%d_t_s')"%(i,i))
        # exec("print(cav%d_t_s)"%(i))
    # cav0_t_s = signal_model.addVars(range(0,len(cav_t_r_list[0])),lb=0,vtype=gurobipy.GRB.CONTINUOUS, name='cav0_t_s')
    # cav1_t_s = signal_model.addVars(range(0,len(cav_t_r_list[1])),lb=0,vtype=gurobipy.GRB.CONTINUOUS, name='cav1_t_s')
    u_s_s = signal_model.addVars(range(0,4),lb=-30,vtype=gurobipy.GRB.INTEGER, name='u_s_s')
    u_s_e = signal_model.addVars(range(0,4),lb=-1,vtype=gurobipy.GRB.INTEGER, name='u_s_e')
    
    td_chv_list = signal_model.addVars(range(0,len(chv_list)),lb=0,vtype=gurobipy.GRB.CONTINUOUS,name='td_chv_list')
    
    b1 = signal_model.addVars(range(0,len(chv_list)),vtype=gurobipy.GRB.BINARY, name='b1')
    b2 = signal_model.addVars(range(0,len(chv_list)),vtype=gurobipy.GRB.BINARY, name='b2')
    
    
    for p in range(4):
        signal_model.addConstr(u_s_e[p] >= u_s_s[p] + g_min)
        signal_model.addConstr(u_s_e[p] <= u_s_s[p] + g_max)
    
    for p in range(3):
        signal_model.addConstr(u_s_s[p+1] == u_s_e[p] + l)
    
    signal_model.addConstr(u_s_s[0] == start_time)
    signal_model.addConstr(u_s_e[3] == end_time)
    
    #HV轨迹估计
    for i in range(len(chv_list)):
        phase = chv_phase_list[i]
        signal_model.addGenConstrIndicator(b1[i], 1, u_s_s[phase]-ta_chv_list[i]>=0)
        signal_model.addGenConstrIndicator(b2[i], 1, ta_chv_list[i]-u_s_e[phase]>=0)
        
        signal_model.addConstr(td_chv_list[i] == b1[i]*u_s_s[phase]+(1-b1[i])*(1-b2[i])*ta_chv_list[i]+b2[i]*(u_s_s[phase]+C))
        
    
    # for i in range(len(chv_list)):
    #     phase = chv_phase_list[i]
    #     if u_s_s[phase] >= ta_chv_list[i]:
    #         signal_model.addConstr(td_chv_list[i] == u_s_s[phase])
    #     elif u_s_s[phase] < ta_chv_list[i] and u_s_e[phase] >= ta_chv_list[i]:
    #         signal_model.addConstr(td_chv_list[i] == ta_chv_list[i])
    #     else:
    #         signal_model.addConstr(td_chv_list[i] == u_s_s[phase]+C)
    
    
    
    
    # exec创建的新变量只存在于exec函数内部，如果在def函数内调用的话，是找不到这个变量的
    # 必须创建一个新的全局变量字典，从全局变量字典里面调用exec创建的变量
    # https://blog.csdn.net/m0_60862600/article/details/125002232
    d = {}
    exec("Js = 0",globals(), d)
    exec("Jl = 0",globals(), d)
    exec("Js_chv = 0",globals(), d)
    
    for i in range(cav_number):
        exec("d['Js'] = d['Js'] + J_s(cav%d_t_s,cav_t_r_list[i])" %(i))
        exec("d['Jl'] = d['Jl'] + rho/2*L_t_cav_signal(cav%d_t_s,z_t_total_cav_list[i],lambda_t_total_cav_signal_list[i])"
              %(i))
        # print(d['Js'])
        # print(d['Jl'])
    
    for i in range(len(chv_list)):
        exec("d['Js_chv'] = d['Js_chv'] + J_s_chv(td_chv_list[i],chv_t_r_list[i])")
    
    
    # Js = Js + J_s(cav_x0_list[0],cav0_t_s) + J_s(cav_x0_list[1],cav1_t_s)
    # Jl = Jl + rho/2*L_t_cav_signal(cav0_t_s,z_t_total_cav_list[0],lambda_t_total_cav_signal_list[0]) + rho/2*L_t_cav_signal(cav1_t_s,z_t_total_cav_list[1],lambda_t_total_cav_signal_list[1])
    # print(Js)
    
    signal_model.setObjective(d['Js'] 
                              + d['Js_chv']
                              + d['Jl']
                              # J_s(cav_x0_list[0],cav0_t_s) + J_s(cav_x0_list[1],cav1_t_s)
                              # +rho/2*L_t_cav_signal(cav0_t_s,z_t_total_cav_list[0],lambda_t_total_cav_signal_list[0])
                              # +rho/2*L_t_cav_signal(cav1_t_s,z_t_total_cav_list[1],lambda_t_total_cav_signal_list[1])
                              +rho/2*L_s_s_signal(u_s_s,z_s_s_list,lambda_s_s_signal_list)
                              +rho/2*L_s_e_signal(u_s_e,z_s_e_list,lambda_s_e_signal_list)
                            ,
                          sense=gurobipy.GRB.MINIMIZE)
    
    signal_model.setParam('OutputFlag', 0)
    # signal_model.params.Method = 2
    
    print('Solving signal model......')
    t0 = time()
    signal_model.optimize()
    t1 = time()
    solve_time = t1-t0
    print('求解时间为： '+str(solve_time)+'s')
    
    res_cav_t_list = []
    for i in range(cav_number):
        res_cav_t_list.append([])
    res_u_s_s = []
    res_u_s_e = []
    
    for item in signal_model.getVars():
        # print(item.varName)
        for i in range(cav_number):
            if 'cav%d_t_s'%(i) in str(item.varName):
                res_cav_t_list[i].append(item.x)
        if 'u_s_s' in str(item.varName):
            res_u_s_s.append(item.x)
        elif 'u_s_e' in str(item.varName):
            res_u_s_e.append(item.x)
    
    
    
    
    return res_cav_t_list,res_u_s_s,res_u_s_e, solve_time