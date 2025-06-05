# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 11:04:38 2023

@author: Jichen Zhu
"""


'''
CAV为后车，CHV为前车
此时需要link controller，并且需要为CHV前方设置一个虚拟车辆
当前即为一个信号周期刚开始的时刻（全局时钟=0）
'''

import numpy as np
import scipy.optimize
from math import log
from time import time
import gurobipy
import matplotlib.pyplot as plt

def chv_prediction(phase_index,front_v,front_t,s_s0,s_e0,
                   hv_v0,hv_x0,t0,hv_range,
                   leader_flag): 
    idm_a = 1
    idm_b = 1.5
    delta = 4
    vd = vf = 50/3.6
    delta_x = 1
    delta_t = 1
    s0 = 2
    T = 1
    C = 100
    
    
    #将空间域模型转化为时间域
    def x_to_t(front_v,front_t):
        if leader_flag == 0:
            front_index = len(front_v)-sum(1 for x in front_t if x >= 0)-1
            front_v_t = [front_v[front_index]]
            front_x_t = [hv_range-front_index]
            
            for t in range(1000000):
                for i in range(len(front_t)-1):
                    if t >= front_t[i]:
                        k = (front_t[i+1] - front_t[i])/1
                        t0 = front_t[i]
                        x0 = i
                        next_x = hv_range - ((t-t0)/k+x0)
                        next_v = front_v[i]
                        front_x_t.append(next_x)
                        front_v_t.append(next_v)
                        break
                if next_x <= 0:
                    break
            front_v_t = front_v_t[1:]
            front_x_t = front_x_t[1:]
        else:
            front_v_t = []
            front_x_t = []
        return front_v_t,front_x_t
    
    
    
    def IDM_t(front_v_t,front_x_t,v0,x0):
        hv_v = [v0]
        hv_x = [hv_range]
        if leader_flag == 0:
            #此时前车是有CAV或者CHV的
            for t in range(1000000):
                if t <= len(front_v_t)-1:
                    #此时前车还在link上，本车还是跟随他
                    delta_v = front_v_t[t]-hv_v[t]
                    s_star = s0 + hv_v[t]*T+hv_v[t]*delta_v/(2*(idm_a*idm_b)**0.5)
                    s = front_x_t[t] - hv_x[t]
                    a = idm_a*(1-(hv_v[t]/vd)**delta-(s_star/s)**2)
                    next_v = max(hv_v[t] + a*delta_t,0)
                    next_x = max(hv_x[t] - (hv_v[t]*delta_t + 0.5*a*(delta_t**2)),0)
                    hv_v.append(next_v)
                    hv_x.append(next_x)
                else:
                    #此时前车已经驶离，此时leader_flag=1，本车为头车，根据下面的判断计算
                    if t >= s_s0[phase_index] and t <= s_e0[phase_index]:
                        #绿灯期间
                        a = idm_a*(1-(hv_v[t]/vd)**delta)
                        # a = idm_a
                        next_v = max(hv_v[t] + a*delta_t,0)
                        next_x = max(hv_x[t] - (hv_v[t]*delta_t + 0.5*a*(delta_t**2)),0)
                        hv_v.append(next_v)
                        hv_x.append(next_x)
                    else:
                        #红灯期间
                        delta_v = 0-hv_v[t]
                        s_star = s0 + hv_v[t]*T+hv_v[t]*delta_v/(2*(idm_a*idm_b)**0.5)
                        s = hv_x[t]
                        a = idm_a*(1-(hv_v[t]/vd)**delta-(s_star/s)**2)
                        next_v = max(hv_v[t] + a*delta_t,0)
                        next_x = max(hv_x[t] - (hv_v[t]*delta_t + 0.5*a*(delta_t**2)),0)
                        hv_v.append(next_v)
                        hv_x.append(next_x)
                        
                if next_x <= 0:
                    break
        elif leader_flag == 1:
            for t in range(1000000):
                if t >= s_s0[phase_index] and t <= s_e0[phase_index]:
                    #绿灯期间
                    a = idm_a*(1-(hv_v[t]/vd)**delta)
                    # a = idm_a
                    next_v = max(hv_v[t] + a*delta_t,0)
                    next_x = max(hv_x[t] - (hv_v[t]*delta_t + 0.5*a*(delta_t**2)),0)
                    hv_v.append(next_v)
                    hv_x.append(next_x)
                else:
                    #红灯期间
                    delta_v = 0-hv_v[t]
                    s_star = s0 + hv_v[t]*T+hv_v[t]*delta_v/(2*(idm_a*idm_b)**0.5)
                    s = hv_x[t]
                    a = idm_a*(1-(hv_v[t]/vd)**delta-(s_star/s)**2)
                    next_v = max(hv_v[t] + a*delta_t,0)
                    next_x = max(hv_x[t] - (hv_v[t]*delta_t + 0.5*a*(delta_t**2)),0)
                    hv_v.append(next_v)
                    hv_x.append(next_x)
                if next_x <= 0:
                    break
        
        return hv_v, hv_x
    
    
    def IDM_adjust(hv_v, hv_x):
        if len(hv_v) < s_s0[phase_index]:
            #车辆在绿灯开始前到达交叉口
            for tt in range(len(hv_v),round(s_s0[phase_index])):
                hv_v.append(0)
                hv_x.append(0)
            
            hv_v.append(1)
            hv_x.append(-1)
            
        elif len(hv_v) > s_e0[phase_index]:
            #车辆在绿灯结束后到达交叉口
            for tt in range(len(hv_v),round(s_s0[phase_index]+C)):
                hv_v.append(0)
                hv_x.append(0)
            
            hv_v.append(1)
            hv_x.append(-1)

        return hv_v, hv_x
    
    
    #将时间域转换为空间域
    def t_to_x(hv_v_t,hv_x_t):
        hv_v_x = [hv_v_t[0]]
        hv_t_x = [0]
        count = 0
        for x in range(hv_range):
            for i in range(len(hv_x_t)):
                if hv_range-x >= hv_x_t[i]:
                    if i < len(hv_v_t)-1:
                        k = hv_v_t[i+1]
                        x0 = hv_x_t[i+1]
                        t0 = i+1
                    else:
                        k = hv_v_t[i]
                        x0 = hv_x_t[i]
                        t0 = i
                    if k < 0.5:
                        count+=1
                        #此时本车会在交叉口停车等待红灯
                        if i <= s_s0[phase_index]:
                            next_t = s_s0[phase_index]+1*count
                        elif i >= s_e0[phase_index]:
                            next_t = s_s0[phase_index]+C+1*count
                        else:
                            next_t = s_s0[phase_index]+1*count
                    else:
                        next_t = -((hv_range-x)-x0)/k+t0
                    if i < len(hv_v_t)-1:
                        next_v = hv_v_t[i+1]
                    else:
                        next_v = hv_v_t[i]
                    hv_v_x.append(next_v)
                    hv_t_x.append(next_t)
                    break
        return hv_v_x,hv_t_x
    
    
    
    front_v_t,front_x_t = x_to_t(front_v,front_t)
    
    hv_v_t,hv_x_t = IDM_t(front_v_t,front_x_t,hv_v0,hv_x0)
    hv_v_t,hv_x_t = IDM_adjust(hv_v_t, hv_x_t)
    # print(hv_v_t)
    hv_v_x,hv_t_x = t_to_x(hv_v_t,hv_x_t)
    # print(hv_t_x[:-1])
    # print(hv_v_x[1:])
    return hv_t_x[:-1],hv_v_x[:-1]
