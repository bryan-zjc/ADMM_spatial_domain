# ADMM_spatial_domain

The code for our study: 

**Cooperative decision-making of multi-vehicles and the traffic signal: A parallel approach in spatial domain**

![image](https://github.com/user-attachments/assets/67c19d5d-76bc-4d9a-ae6b-ddebaa465c6a)

_Fig.1. System structure of the proposed MVSC controller._

**Abstract:**

With the emerging Vehicle-Road-Cloud Integration System (VRCIS) and Connected and Automated Vehicle (CAV) technologies, cooperatively optimizing CAV trajectories and the traffic signal plan helps enhance vehicle mobility and traffic efficiency. However, current studies on the cooperation among multi-vehicles and the traffic signal still have limitations. Firstly, current cooperation methods between vehicle and signal cannot ensure decision-making optimality. Secondly, their computing complexity significantly increases with the increasing number of Connected and Automated Vehicles (CAVs). Thirdly, conventional studies cannot be implemented in actual Human-driven Vehicles (HVs) and CAVs mixed traffic. In this research, a new Multi-Vehicles and Signal Cooperation (MVSC) planner is proposed to solve the aforementioned limitations via the following novel designs. (i) The cooperative decision-making problem is formulated into the spatial domain. It combines multi-vehicles’ trajectory planning and signal planning in one optimal control problem. (ii) A parallel solving algorithm is designed based on Alternating Direction Method of Multipliers (ADMM). It ensures real-time computation efficiency even in the cooperation of large-scale vehicles. (iii) HVs are considered in cooperative decision-making to enable the real-world implementation. Simulation results show that the proposed MVSC planner can enhance global traffic mobility and ecology by 23.60% and 15.63%. At CAV’s penetration rate of 40% and V/C ratio of 0.75, the proposed planner shows its full potential in performance enhancement. The average computation time of the parallel computing approach is only within 10 milliseconds. It confirms the proposed planner’s real-time implementation capability in real traffic environment.


![image](https://github.com/user-attachments/assets/e8d83f22-8af5-4299-9bb6-9e90e940ac1b)

_Fig. 2. A comparison between spatial and temporal domain._



