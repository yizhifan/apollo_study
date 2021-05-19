# Piecewise_jerk_path_optimizer 
***
## 一、主要任务
对不同任务下的reference line以及对应的可行驶区域（Boundary）通过optimizer求出最优path.
## 二、流程图
``` flow
st=>start: task.process
e=>end: 输出轨迹
op1=>operation: 预处理
op2=>operation: 规划求解器

st->op1->op2->e
```
