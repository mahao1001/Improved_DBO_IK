%% 采用原始的DBO算法，不能修改DBO.m
%% 不适用Get_Functions_details.m，这些都是算法的测试函数
%% 需要新建目标优化函数rbt_ik_Function.m
%% 求解逆运动学需要用要机器人，主要是正运学函数fkine，所以一定要在函数中包含机器人建模
%% 第一次是使用的MDH建立机器人，这个地方的qlim限定关节是没用的
%% 问题就是输入位置xyz，姿态rpy，求关节转角Theta的问题
%% 假设输入了变量Theta，通过正运动fkine可以计算出转换矩阵Ti，分离出新的位置xyzi，新的姿态rpyi。
%% 现在就可以构造目标函数了
%% 第一种最简单的：xyzi-xyz的距离+转角差值rpyi-rpy，加权即可构造出。
    P = xyzi - xyz;        %位差值矩阵
    Q = rpyi - rpy;        %姿差值矩阵
    o =  0.55*sqrt (P(1)^2+P(2)^2+P(3)^2) + 0.45*sqrt(Q(1)^2+Q(2)^2+Q(3)^2);
%% 这个目标函数被优化函数DBO.m调用，还需要定义几个参数：
    SearchAgents_no=30;                          % 种群数
    Max_iteration=100;                           % 最大迭代次数
    lb = [-360,-118,-225,-360,-97,-360];         % 设置每个关节的转角下限制
    ub = [360,120,11,360,180,360];               % 设置每个关节的转角上限制
    dim = 6;                                     % 因为是6个关节，所以变量Theta是1*6的
    fobj = @(x)rbt_ik_Function(x);               % 用@构造fobj函数，把变量x输如到function函数
%% 优化计算完成后的输出有三个：
    % fMin 是最小的误差
    % bestX 是最优解，也就是6个角度
    % DBO_curve 是迭代计算次数曲线
%% 按照最优解bestX控制机器人的位姿，使用fkine函数，故主函数中也需要使用MDH建模。
%% 改进的算法需要再计算，每次的DBO_curve迭代计算次数曲线和bestX最优解需要保持下来。
%% 运行结果，都迭代500次：
%% 第一次运行
   % first.fig 是第一个运行图，对应的DBO_curve数据保存在了first.mat
   % The best solution obtained by SSA is : -331.0763     -5.265419     -15.71843      43.69305     -4.473759     -17.25292
   % The best optimal value of the objective funciton found by SSA is : 11.0199
   % 位置精度还是很高的，就是这个角度精度很低
%% 第二次运行
