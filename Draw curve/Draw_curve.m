%% 绘制迭代次数曲线
clear;
clc;
%% 导入不同算法的数据
% load('I_DBO_curve.mat');
% I_DBO_data = I_DBO_curve;

load('first_DBO.mat');
DBO_data = DBO_curve;
load('first_GWO.mat');
GWO_data = GWO_cg_curve;
load('first_WOA.mat');
WOA_data = WOA_cg_curve;
load('first_ALO1.mat');
ALO_data = cg_curve;
load('first_SSA.mat');
SSA_data = SSA_curve;

%% 绘制对应的曲线
% 调整线条的颜色：'red'、'green'、'blue'、'cyan'、'magenta'、'yellow'、'black'、'white'
figure;
% semilogy(I_DBO_data,'Color','red','LineWidth',2.5);
hold on;
semilogy(DBO_data,'Color','red','LineWidth',2,'LineStyle','-');  
semilogy(GWO_data,'Color','blue','LineWidth',2);
semilogy(WOA_data,'Color','green','LineWidth',2);
semilogy(ALO_data,'Color','black','LineWidth',1.5);
semilogy(SSA_data,'Color','cyan','LineWidth',2);
%%
title('Robot Inverse Kinematic based swarm intelligence (SI) optimization algorithms')
xlabel('Iteration');
ylabel('Best score obtained so far');


axis tight
grid on
box on
% legend('I-DBO','DBO','GWO','WOA','ALO','SSA')      % 给每条线加标签
legend('DBO','GWO','WOA','ALO','SSA')      % 给每条线加标签

% display(['The best solution obtained by WOA is : ', num2str(Best_pos)]);
% display(['The best optimal value of the objective funciton found by WOA is : ', num2str(Best_score)]);