# 武科大工程视觉
## 已完成：
### 状态机转换闭环：detector<->mtc_place<->Wust_driver  
LOST->TRACKING->LOCKED->PLANNING->EXECUTING->LOST（成功）  
LOST->TRACKING->LOCKED->PLANNING->LOCKED（PLANNING失败）  
LOST->TRACKING->LOCKED->PLANNING->EXECUTING->LOCKED（PLANNING失败）  
### 比较完善的MTC流程，但关节限位有问题（mtc_place）  
### 通过自定义Action Server接收规划结果并进行控制反馈（Wust_driver），目前只是虚拟串口
