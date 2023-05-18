% -----------------------------------------------------------------------------------------------------------
% Dung Beetle Optimizer: (DBO) (demo)
% Programmed by Jian-kai Xue    
% Updated 28 Nov. 2022.                     
%
% This is a simple demo version only implemented the basic         
% idea of the DBO for solving the unconstrained problem.    
% The details about DBO are illustratred in the following paper.    
% (To cite this article):                                                
%  Jiankai Xue & Bo Shen (2022) Dung beetle optimizer: a new meta-heuristic
% algorithm for global optimization. The Journal of Supercomputing, DOI:
% 10.1007/s11227-022-04959-6
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear 
clc
%% 加载机器人
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%建立机器人修改的M-DH参数，初始状态
% 连杆偏移d,连杆长度a,连杆扭转角alpha
L1=Link('d',267,'a',0,'alpha',0,'modified'); 
L2=Link('d',0,'a',0,'alpha',-pi/2,'offset',-1.3849179,'modified');
L3=Link('d',0,'a',289.48866,'alpha',0,'offset',1.3849179,'modified');
L4=Link('d',342.5,'a',77.5,'alpha',-pi/2,'modified');
L5=Link('d',0,'a',0,'alpha',pi/2,'modified');
L6=Link('d',97,'a',76,'alpha',-pi/2,'modified');
% qlim=[-360,360;-118,120;-225,11;-360,360;-97,180;-360,360];  % 设置每个关节的转角限制
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','Cobot-xArm6');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

SearchAgents_no=30; % Number of search agents

% Function_name='x'; % Name of the test function that can be from F1 to F23 (Table 1,2,3 in the paper)

Max_iteration=500; % Maximum numbef of iterations

% Load details of the selected benchmark function
% R[lb,ub,dim,fobj]=Get_Functions_details(Function_name);
%% 设置约束条件
lb = [-360,-118,-225,-360,-97,-360];         % 设置每个关节的转角下限制
ub = [360,120,11,360,180,360];                % 设置每个关节的转角上限制
dim = 6;
fobj = @(x)test_rbt_ik_Function2(x);

tic;  % 开始计时
[fMin_DBO,bestX_DBO,I_DBO_curve]=Improved_DBO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
DBO_time = toc;  %结束技术保存在DBO_time

figure;
semilogy(I_DBO_curve,'Color','r','LineWidth',1)    %画出迭代图
title('Robot Inverse Kinematic based Improves-DBO Algorithm')
xlabel('Iteration');
ylabel('Best score obtained so far');
% axis tight
grid on
box on
legend('I-DBO')
display(['The best solution obtained by I-DBO is : ', num2str(bestX_DBO)]);
display(['The best optimal value of the objective funciton found by DBO is : ', num2str(fMin_DBO)]);
display(['The time calculate by I-DBO is : ', num2str(DBO_time)]);
%% 按最优解控制机器人，验证逆解的正确性
Theta_new=bestX_DBO/180*pi;               %换算成弧度
T=robot.fkine(Theta_new);             %求正解的齐次变换矩阵
W=[-800,+800,-800,+800,-800,+800];
figure;
robot.plot(Theta_new,'tilesize',150,'workspace',W);  %显示三维动画
qi=robot.ikcon(T)*180/pi;      %求逆解验证关节角,ikcon(with joint limits),ikine(without joint limits)
rot = t2r(T);                  %齐次矩阵变换为旋转矩阵
Qua= UnitQuaternion(T);         %求四元数 
rpy=tr2rpy(T, 'zyx')*180/pi;    %求末端姿态，工具法兰为绕XYZ轴旋转 tr2rpy输出值默认对应的顺序是ZYX
robot.teach(T,'rpy' )          %显示roll/pitch/yaw angles，GUI可调界面