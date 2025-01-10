## Semester project notes

#### 项目需求

- 项目的project需求包含两个部分，coverage和exploration
- 主要还是利用钢架的信息来做exploration



#### 已完成的工作

- 看了之前的文章
  - 分析了各种视觉检测方法，最后选定了3D迭代霍夫变换
  - 基于霍夫变换进行了改进
  - 增加了视觉里程计的算法来对真个几何结构的构建

- 配置了环境测试了代码

#### 疑问

- 现在的路径是提前定义好的吗？

- planning的目的是全面覆盖钢架吗？


#### 未来要做的

- 熟悉代码和传感器数据
- 看exploration相关的论文
- 思考如何利用几何信息
  - 利用钢架的交点的旅行商问题来引导？
  - 首先看能建立出钢架如何的几何特征

### 时间规划

- 文献回顾：

  - 9.9-9.15 week 1：熟悉之前的工作和代码环境

  - 9.16-9.22.week 2：阅读推荐论文
- 建模问题并求解：

  - 9.23-9.29 week 3：阅读其他论文，建模数学问题，制定时间表

  - 9.30-10.6 week 4：思考算法细节，精读代码确定输入输出接口
- 实现

  - 10.7-10.13 week 5：尝试代码实现
  - 10.14-10.20 week 6：代码实现
  - 10.21-10.27 week 7：放假（复习）
- 改进算法

  - 10.28-11.3 week 8：实验记录进行问题分析 （考试凸优化）
  - 11.4-11.10 week 9：阅读论文寻找改进方法（考试智能代理）
  - 11.11-11.17 week 10：算法改进
- 分析实验结果写报告
  - 11.18-11.24 week 11: 写报告初稿
  - 11.25-12.1 week 12：代码整理，完善论文报告



#### 相关论文：

- （TSP）Distributed Infrastructure Inspection Path Planning for Aerial Robotics subject to Time Constraints
  - 先随机采样兴趣点
  - 求解TCP问题
  - 迭代优化
- (Frontier)Surface-based Exploration for Autonomous 3D Modeling
  - **frontier-based**
    - information-gain
    - navigation-cost 
  - NBV
  - surface-based method
- (TSP)A Two-Stage Optimized Next-View Planning Framework for 3-D Unknown Environment Exploration, and Structural Reconstruction
  
  - combines a viewpoints coverage planning with a Fixed Start Open Traveling Sales Man Problem solving
  
- (TSP)FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes

  - 基于skeleton的提取进行划分subspace，分全局和局部的规划
  - 高效的采样，避免重复
  - 可以多个subspace并行计算路径，从而实现高效的路径规划
  - 全局的子空间访问TSP问题求解
  - 增加了子空间之间转换的丝滑程度

- (TSP)Hierarchical Coverage Path Planning in Complex 3D Environments

  - Viewpoint sampling from prior map
  - 上层算法：分子空间
    - 先采样然后针对进行可行的viewpoint来划分子空间
    - GTSP求解访问顺序
  - 下层算法：在子空间采样viewpoint做规划
    - 针对viewpoint的优先级进行取点，取完点之后求解TSP寻找最短路径
  - 分化子空间的优点：
    - TSP问题的计算量是爆炸式增长的
    - 可以提前并行计算
    - Avoid fall into local optimal solution caused by greedy strategy.

- (Frontier)Receding Horizon “Next–Best–View” Planner for 3D Exploration

  - cubical volumes occupancy map

  - RRT* sampling

    - 节点信息增益的计算是体素增加量加上距离惩罚，再不断累加以往节点。

      Gain(n k ) = Gain(n k−1 ) + Visible(M, ξ k )e −λc(σ k−1 ) ,

  - receding horizon optimization

    - 只取第一个节点运行
    - 保留之前的评估信息
    - 当节点大于阈值，或者没有信息增益增加时结束
  
- (TSP)TARE



#### 当前处理可以提供的几何信息

- 钢架分割的编号，起点和终点的三维坐标
- 钢架之间的交点三维坐标

#### 可能方法

- 基于信息增益选择下一视角
  - 主要是如何评估这个信息增益：视角覆盖率的问题
  - 后期可以参考滚动优化的思想
- 结合TSP问题的路径规划
  - 但是主要想关注新生成的那些点，需要动态的构成几何图
  - 构成几何图先，钢架的端点和交点，然后求解TSP问题依次访问各个点
  - 问题是端点是不断变化的，解决方法是采用不断更新的动态的求解方法
- 滚动规划



#### 初步idea:

- ~~idea 1:也可以均匀采样，通过评估附近包含多少的端点来评估信息增益，然后裁剪选择viewpoint。~~
  - ~~viewpoint的表达是位置加角度，可以定义针对每个viewpoint一个距离D，垂直钢架一段距离进行采样~~
  - ~~**不够高效**~~
  
- idea 2:根据感知信息初步评估边界点的权重，如距离和信息增益等
  - **更有目的性**
  - 避免贪心策略导致的局部最优
  - 评估可能的影响因素：
    - 定义一个视野场 field of view, ray casting: dynamic papameters, vertical, horizon and distance.
    - 端点可以增加的覆盖长度
    - 端点领域的边界端点数量
  
- 构建图，求解开式TSP问题
  - OR-Tools优化工具
  - 不一定要TSP
- 不断迭代动态更新裁剪图
  - 不断更新每个点的信息增益来动态调整，如果在视野范围内，没有segment端点的移动就remove
  - 划分局部horizon，适当保留local horizon之外的信息



#### 实际问题：

- 无人机访问每个点都是面对整个铁塔的  而不是直接到达
  - viewpoint的表达是位置加角度，可以定义针对每个viewpoint一个距离D
  - only change **yaw** angle, fix **pitch** and **roll** angle.



#### Pipeline

- ##### STEP 1: Frontier evaluation
  
  - Define the FoV
    - 定义一个水平和垂直角度都为90度，距离为4M的视野锥
    - 确定是锥面还是截头锥
      - 截头锥 frustum
  - Information gain: distance, segments incresment , endpoint number.
    - 首先对于每个要判定的点，无人机在其前方2m距离
    - 如下条件判断端点是否在FOV内：
      - 端点和无人机的距离是否小于4m
      - 端点和无人机的夹角是否在范围内
    - 计算可能增加的覆盖长度：
      - 已知钢架的方向和端点，通过到计算到边界平面的投影来计算可能增加的长度
      - 通过几何方法计算射线和超平面的交点
      - 然后去掉不再视野场里面的点
      - 可以得到在视野场里面未覆盖的长度
- ##### STEP 2: optimal path planning
  
  - 方法一：求解TSP问题
    - 建立加权之后的图
    - or-tools求解
  - 方法二：分层的NBV策略，模仿tare的方法
    - 全局：没有局部点的时候引导
    - 局部：聚类frontier来确定局部的目标点
      - 每次只选出一个点
  - 方法三：建模为强化学习的方法
- ##### STEP 3: Dynamic update and optimization
  
  - optimize trajectory when considering collision
  - update the local horizon centered on drone
  - repeat



#### Interface:

- 输入：分割钢架段的几何信息
  - seg id
  - 端点的三维坐标
- 输出：给出一条路径轨迹
  - 一系列waypoints：(x, y, z, yaw, duration)





#### Implementation:

- ##### STEP 1: Frontiers evaluation

  - 发布了端点坐标
  - 解码了无人机位姿和端点坐标
  - 判断点是否在FoV里面，判断是否是边界点还是端点
    - 如果在机器人的FoV里面就是端点
  - 评估点的时候，这个FoV的姿态是怎么样的？
    - 与法平面垂直，相隔一定距离
  - 设定一个local horizon
    - 以机器人的坐标为中心的三维球体
  - 代码结构：
    - 2个回调，分别进行端点收集和机器人坐标获取
    - 2个线程：processingThread(), recordFrontiersThread()
      - 边界点评估：evaluateVisibilityGain()
        - 点是否在FoV里面：isPointInFoV()
        - 计算法向量：calculatePlaneNormal()
        - 计算射线和超平面的交点：calculateIntersection()
        - 计算FoV里面的交点：IntersectionWithinFoV()
        - 距离惩罚:evaluateDistance()
      - 记录访问过的点
        - 利用特化后的unordered_set来装访问过的点
  
- ##### STEP 2：optimal path

  - method 1：TSP-based
    - 局部：使用or-tools工具库求解
    - 全局：也是求解TSP
    
  - method 2：NBV-based
    - 局部：使用聚类来取高增益点的中心，引入动量惩罚概念
    - 全局：当没有局部目标点的时候，NBV
    
  - RRT* path planing:
    - 局部路径规划，主要进行避障

- ##### STEP 3: optimal and update 

  - 主循环
  - 建立运动指令链接
  - 更新一个局部框
  
- ##### Visualization

  - STEP1:
    - 选中的frontier
    - FoV里面的端点：
      - 这是一个点集合，可以采用sensor_msg::pointcloud发布点集的方式
    - 访问过的点：
  - STEP2:
    - 选中的goal point
    - 经过的路径



##### 待解决的问题：

- (*)对于每个要评估的边界点，需要找到一个对应的最佳的viewpoint
  - 实现了，碰撞问题，是在采样的时候考虑还是交给local path planning处理
    - 在采样时候去掉和segments碰撞的点，实现一个函数，点与segments的碰撞检测
- (*)对于visibility gain, density gian, distance punishment三个因素增加调节参数, 本来是想的先归一化，然后进行权重的分配，但是不知道最大最小值
  - 去掉density gain
  - 动态的进行归一化
  - 不断调试三个参数
- (*)利用优先队列和unordered_set来管理最佳视点
  - 实现了，存在管理的问题，对于NBV-based的方法，也许没有必要；
- (*)打通运动控制的包，发送什么指令
  - rosservice call /mavros/cmd/arming "value: true"
  - 往topic /mavros/setpoint_raw/local发送mavros_msgs/PositionTarget的消息类型，参考trajectory.py的节点。
- (*)存在目标点超出了local horizon的情况，时序的问题，给足够运动规划时间
  - 另外用一个定时器回调来进行读取queue，接受到的要保证是最新的
- (*)判断真正的端点，用一个unordered_set来记录，然后进行查询
- (*)分割有重叠：是感知的问题，可能降低速度
- (*)修改策略，每次评估，评估所有的边界点，只清除端点。然后在planning阶段，进行local和global的planning



#### week 11

##### 功能性问题：

- ( * )机器人的位姿要用ground truth, 从topic里面订阅：/macortex_bridge/starling/pose

- (*)**整合RRT*，输入输出，可视化路径**
  - (*)**碰撞检测有问题**
  
- ( * )添加结束条件，发送回到原点的指令:
  - 没有可用的frontier的时候
- (*）反复跑的问题，还是对于endpoints的判断：
  - 哈希表的相等判断重写有问题

##### 

#### week 12

- (周二)RRT计算速度太慢，分支的长度
  - (*)可视化RRT采样过程
  - 碰撞检测，不用检测所有的segments.
    - 调大了jump_size减少采样可以减少计算量
    - 比如先局限于以当前位置到目标位置为半径的球里面
- **（*）思考办法在探索途中也保持最大的visibility gain：**
  - **对于RRT*计算出来的点进行插值，在目标viewpoint之前的点，全部面向z轴**
    - **目的是在移动过程中依然保持更多的覆盖**
    - **另一方面是为了定位减少飘逸**
- （*）**优化局部路径：**
  - **平滑轨迹，也许无人机不需要轨迹平滑，因为没有动力学约束**
- (*)实验的时候使用corrected position，因为不要丢失视野
- (*)容易卡死：实时进行RRT规划，因为在移动过程中会遇到新的障碍物
  - (*)当直线情况增加步长
  - **直线碰撞检测存在问题**
    - 对RRT分支离散采样，判断采样点会不会和圆柱体碰撞
- (*)viewpoint的不能贴地



#### Week 13

- (*)collision check还是要尝试geometric tools.
  - 提取所有的依赖文件
- (*)增加对侧目标点的惩罚，原因是RRT的寻找路径不利于，现在没必要了
- (*)修改空间权重来优先水平扫描，然后逐渐向上:
  - 在空间上z轴上的距离惩罚项
  - 把local queue从球体变成水平带
- (*)保持视线面对当前已知道的结构，主要是知道结构的中心，同样在水平和垂直方向赋予不同的权重权重
  - step1:获得已知结构的中心轴
  - step2:插值朝向中心轴
- (*)RRT采样为了估计寻找横向的路径，在z轴采样使用正太分布
- (*)重新定义探索完成的条件:
  - segment大于一个数量
  - 访问过的边界点/所有的边界点 > 95%

- **drift太严重，以至于到达目标点的判断有问题,会导致RRT 搜索失败**
  - estimated pose始终更加靠近障碍物
  - 确定坐标系:map是真实坐标系
- 思考一些实验的metrics
  - **量化评估指标:**
    - 能量消耗，因为上升需要克服重力
    - 探索效率:单位时间内增长的segments长度
      - 有个问题,就是sgement会出现重复的
    - 路径总长度
    - planning每个iteration需要的峰值时间,反映了对环境的动态反映
  - **评估对象：**
    - jump_size: 
    - 增加z轴权重 or 不加
    - 增加动量奖励 or 不加
    - RRT在z轴上是均匀采用 or 高斯采样
    - 最近优先 or 水平带优先

- trade off，机器人的传感器是前向的，如果要保持观察主体就会看不到后面的障碍物



#### Week 13:

##### 使用GT位姿进行实验，保证探索算法的可行性

- 探索完成：
  - 超时
    - 考虑卡住
  - 全面覆盖

##### 进行量化实验评估：

- **评估对象：**

  - jump_size: 
  - 增加z轴权重 or 不加
  - RRT在z轴上是均匀采用 or 高斯采样
  - 最近优先 or 水平带优先
  - 评估函数的参数

- ##### 评估指标：

  - 能耗
  - 探索效率：单位时间内增长的segments长度
  - 轨迹长度
  - 成功率
  - 完成的时长
  - 每个迭代的时长

##### 实验：

1. With fixed definitions: 
   1. FOV, speed, hardware, space size, exploration time limitation, etc.
      1. FoV: [pi/3, pi/3, 1.5]
      2. i7-7700 3.6GHz x 8
      3. exploration limitation: 800s
      4. space size: 5m * 5m * 18m
      5. max velocity: 1.0m/s; max accelarate:0.8
2. **Algorithm performance experiment, the metrics:** 
   1. average time-consuming every iteration
      1. 最大评估时间
      2. 最大规划时间
   2. total time cost
   3. probability of successful exploration：10次实验
   4. exploration efficiency:图表，matplotlib
3. **Comparison experiment:**

- horizontal-first planning or vicinity-first planning
- RRT jump_size large or small：
  - 0.6
  - 0.2

- factors of gain function
  - visibility gain: 6.0
  - visibility gain: 1.0

the same metrics:    

- average time-consuming every iteration

- energy consumption，主要区分垂直路程和水平路程
  - 分成两部分，一个是运行时间，一个是克服空气阻力的路程

- exploration efficiency：图表，matplotlib

All the experiments are under ground truth pose. 



##### 实验问题记录：

- 视野有限，会卡死
- 在遇到上面的结构比较复杂的时候，会出现分割整体偏移
  - 如果是邻近优先的方法，偏移更加严重



#### 实验记录：

- ~~电脑硬件性能~~：CPU: i7-7700 3.6GHz x 8
- ~~ROS节点图~~：
- 比较实验: 
  - 比较对象：
    - strategy: 
    - ~~jump size: 1.0 or 0.2~~
    - ~~visibility gian的权重6 or 0.5~~
  - 总时间
  - 算法效率，分别保存表格
  - 能耗计算
    - 向上：克服重力 mg= 9.8*0.295 2.891 + 0.069
    - 向下：free
    - 水平：induced drag: tau 2.89, d = 0.024 s/m , v = 1 m/s. 0.069  
- ~~录视频： 一次最好的~~
- ~~保存数据到csv:~~
  - ~~evaluation time ： 一次最好的~~
    - 评估次数
    - 最大次数
  - ~~planning time ：一次最好的~~
    - 评估次数
    - 最大次数
  - ~~exploration efficency ： 一次最好的~~
    - 差分可以知道增长率
    - 平均效率
  - ~~energy consumption: 能耗，图表~~
- ~~成功率：70%~~
  - 10次7次成功
  - 一次提前结束
  - 2次卡死
- ~~轨迹图比较~~
  - horizon
  - vicinity
- ~~出问题的图：~~
  - ~~卡死~~
  - ~~偏移~~
  - ~~提前结束~~





论文：

- 差算法伪代码说明

