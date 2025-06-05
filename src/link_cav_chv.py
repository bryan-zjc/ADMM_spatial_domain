# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 11:39:03 2023

@author: Jichen Zhu
"""

'''
CAV为后车，CHV为前车
'''

import numpy as np
import scipy.optimize
from math import log
from time import time
import gurobipy
import matplotlib.pyplot as plt


def link_cav_chv(index,cav_x0,cav_v0,z_t_cav_list,lambda_t_cav_link_list,
                 hv_t_idm,hv_range,
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
    max_time = 0.5
    rho = 200


    
    #求解link controler模型
    #即link controller只需要规范后车的行为保证安全就可以了
    link_model = gurobipy.Model("link_model")
    # link_model.Params.NonConvex = 2
    link_model.setParam('Timelimit', max_time)
    
    cav_t_link = link_model.addVars(range(0,len(cav_t_r)),lb=0,vtype=gurobipy.GRB.CONTINUOUS, name='cav_t_link')
    
    for x in range(len(hv_t_idm)):
        link_model.addConstr(cav_t_link[x+cav_range-hv_range] - hv_t_idm[x] >= 2)
    
    link_model.setObjective(rho/2*L_t_cav_link(cav_t_link,z_t_cav_list,lambda_t_cav_link_list)
                            ,
                            sense=gurobipy.GRB.MINIMIZE)
    
    link_model.setParam('OutputFlag', 0)
    link_model.params.Method = 2
    

    t0 = time()
    link_model.optimize()
    t1 = time()
    solve_time = t1-t0
    print('求解时间为： '+str(solve_time)+'s')

    res_cav_t_link = []

    
    for item in link_model.getVars():
        if 'cav_t_link' in str(item.varName):
            res_cav_t_link.append(item.x)

    
    
    
    
    return res_cav_t_link,solve_time
