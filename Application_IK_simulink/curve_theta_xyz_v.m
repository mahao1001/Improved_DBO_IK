%% 绘制迭代次数曲线
clear;
clc;
%% 导入数据 6个关节的角度
load('outjoint-theta6.mat'); % simout 是导出的曲线数据文件名
xyz = out.simout; % 信号数据
xyz1 = reshape(xyz(1,:,:),1,[]) *180/pi; % 信号数据 joint1
xyz2 = reshape(xyz(2,:,:),1,[]) *180/pi; % 信号数据 joint2
xyz3 = reshape(xyz(3,:,:),1,[]) *180/pi; % 信号数据 joint3
Theta4 = reshape(xyz(4,:,:),1,[]) *180/pi; % 信号数据 joint4
Theta5 = reshape(xyz(5,:,:),1,[]) *180/pi; % 信号数据 joint5
Theta6 = reshape(xyz(6,:,:),1,[]) *180/pi; % 信号数据 joint6

% 绘制图形
figure;
hold on;
semilogy(xyz1,'Color','red', 'LineWidth',2);
semilogy(xyz2, 'LineWidth',2);
semilogy(xyz3, 'LineWidth',2);
semilogy(Theta4, 'LineWidth',2);
semilogy(Theta5, 'LineWidth',2);
semilogy(Theta6, 'LineWidth',2.5, 'LineStyle','--');
xlabel('Number of nodes for imputation calculation (N)');
ylabel('Joint angle (unit: deg)');
title('Joint angle obtained by inverse kinematics solution');
legend('Joint1','Joint2','Joint3','Joint4','Joint5','Joint6')      % 给每条线加标签

%% 导入数据 终点的位置xyz
load('outjoint-eexyz.mat'); % simout 是导出的曲线数据文件名
xyz = out.simout1; % 信号数据
xyz1 = reshape(xyz(1,:,:),1,[]) *1000 ; % 信号数据 xyz1
xyz2 = reshape(xyz(2,:,:),1,[]) *1000; % 信号数据 xyz2
xyz3 = reshape(xyz(3,:,:),1,[]) *1000; % 信号数据 xyz3


% 绘制图形
figure;
hold on;
semilogy(xyz1,'Color','red', 'LineWidth',2);
semilogy(xyz2, 'Color','b','LineWidth',2);
semilogy(xyz3,'Color','black', 'LineWidth',2, 'LineStyle','-');
xlabel('Number of nodes for imputation calculation (N)');
ylabel('Position of end effector (unit: mm)');
title('Position of end effector by forward kinematics');
legend('X axis','Y axis','Z axis')      % 给每条线加标签


%% 末端速度也可以计算，这里不再设计
