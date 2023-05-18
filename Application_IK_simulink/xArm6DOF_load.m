clear
clc

Ts = 0.001;

% [DOF6_Arm,ArmInfo]=importrobot('xArm6DOF');
Ma1001 = importrobot('xArm6.urdf');

load wp.mat;
load wp6joint.mat;
load circle_P.mat;P = P';
load N_1_61.mat;
% load wp6JTorque.mat;

% showdetail(Ma1001) %显示连杆间父子关系
show(Ma1001,'Frames','on','Visuals','on') %figure显示坐标
% Ma1001_SM = smimport(Ma1001); %利用smimport函数导出Simscape文件，做可视化仿真。