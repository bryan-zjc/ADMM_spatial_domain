# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 11:23:43 2023

@author: Jichen Zhu
"""


'''
CAV1为后车，CAV2为前车
'''

import numpy as np
import scipy.optimize
from math import log
from time import time
import gurobipy
import matplotlib.pyplot as plt


def link_cav_cav(index,cav1_x0,cav1_v0,z_t_cav1_list,lambda_t_cav1_link_list,
                 cav2_x0,cav2_v0,z_t_cav2_list,lambda_t_cav2_link_list,
                 t0):
    
    print('Solving link%s model......'%index)
    
    u_min = -3 # 取自Zhang et al., 2022
    u_max = 3 # 取自Zhang et al., 2022
    
    vf = 50/3.6
    delta_x = 1
    sigma = 0.000000000000000001
    
    g_max = 30
    g_min = 10
    C = 60
    l = 3 #inter green time
    
    
    #构造reference轨迹
    cav1_t_r = []
    cav1_v_r = []
    cav1_range = int(cav1_x0/delta_x)+1 #第一个是当前的状态
    for x in range(int(cav1_range)):
        if x == 0:
            cav1_t_r.append(0) #第一个元素是通过每个断面点的时刻
            cav1_v_r.append(cav1_v0) #第二个元素是通过每个断面点的速度
        else:
            cav1_t_r.append(cav1_t_r[x-1]+delta_x/(cav1_v_r[x-1]+sigma)) #第一个元素是通过每个断面点的时刻
            cav1_v_r.append(vf) #第二个元素是通过每个断面点的速度
        
    cav2_t_r = []
    cav2_v_r = []
    cav2_range = int(cav2_x0/delta_x)+1
    for x in range(int(cav2_range)):
        if x == 0:
            cav2_t_r.append(0) #第一个元素是通过每个断面点的时刻
            cav2_v_r.append(cav2_v0) #第二个元素是通过每个断面点的速度
        else:
            cav2_t_r.append(cav2_t_r[x-1]+delta_x/(cav2_v_r[x-1]+sigma)) #第一个元素是通过每个断面点的时刻
            cav2_v_r.append(vf) #第二个元素是通过每个断面点的速度
    

    def J(cav_t_link):
        value = 0
        for x in range(len(cav_t_link)):
            value += cav_t_link[x]
        return value
    
    # 相邻车辆耦合关系模块
    # 对偶变量
    def L_t_cav_link(cav_t_link,z_t_cav_list,lambda_t_cav_link_list):
        value = 0
        for x in range(len(cav_t_link)):
            value = value + (cav_t_link[x]-z_t_cav_list[-1][x]+lambda_t_cav_link_list[-1][x])**2
        return value
    
    
    
    
    
    
    #求解link controller模型（CAV-CAV）
    max_time = 0.1
    rho = 200


    
    #求解link controler模型
    #即link controller只需要规范后车的行为保证安全就可以了
    link_model = gurobipy.Model("link_model")
    # link_model.Params.NonConvex = 2
    link_model.setParam('Timelimit', max_time)
        
    cav1_t_link = link_model.addVars(range(0,len(cav1_t_r)),lb=0,vtype=gurobipy.GRB.CONTINUOUS, name='cav1_t_link')
    cav2_t_link = link_model.addVars(range(0,len(cav2_t_r)),lb=0,vtype=gurobipy.GRB.CONTINUOUS, name='cav2_t_link')

    for x in range(len(cav2_t_r)): #前车的变量少
        link_model.addConstr(cav1_t_link[x+cav1_range-cav2_range] - cav2_t_link[x] >= 2)
    
    
    link_model.setObjective(rho/2*L_t_cav_link(cav1_t_link,z_t_cav1_list,lambda_t_cav1_link_list)
                            +rho/2*L_t_cav_link(cav2_t_link,z_t_cav2_list,lambda_t_cav2_link_list)
                            ,
                            sense=gurobipy.GRB.MINIMIZE)
    
    
    link_model.setParam('OutputFlag', 0)
    link_model.params.Method = 2
    
    
    t0 = time()
    link_model.optimize()
    t1 = time()
    solve_time = t1-t0
    print('求解时间为： '+str(solve_time)+'s')
    
    res_cav1_t_link = []
    res_cav2_t_link = []
    
        
    for item in link_model.getVars():
        if 'cav1_t_link' in str(item.varName):
            res_cav1_t_link.append(item.x)
        elif 'cav2_t_link' in str(item.varName):
            res_cav2_t_link.append(item.x)
    
    
    
    
    return res_cav1_t_link,res_cav2_t_link, solve_time