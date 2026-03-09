# 武科大工程视觉
## 已完成的事项：
### 状态机转换闭环：detector<->mtc_place<->Wust_driver  
### LOST->TRACKING->LOCKED->PLANNING->EXECUTING->LOST（成功）  
### LOST->TRACKING->LOCKED->PLANNING->LOCKED（PLANNING失败时）  
### LOST->TRACKING->LOCKED->PLANNING->EXECUTING->LOCKED（PLANNING失败时）  
### 比较完善的MTC流程，但关节限位有问题（mtc_place）  
### 通过自定义Action Server接收规划结果并进行控制反馈（Wust_driver），目前只是虚拟串口
### 传统视觉识别兑换框并解算（from 华南虎）  
### 将slot_in_world转换为TCP_in_world，确定规划终点  
## 未完成的事项：
### 添加机械限位避免自碰撞
### 和真实串口通信，验证时间戳＋关节角度的控制模式
