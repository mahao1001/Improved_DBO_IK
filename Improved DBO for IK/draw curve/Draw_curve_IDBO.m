%% 绘制迭代次数曲线
clear;
clc;
%% 导入不同算法的数据
load('I_DBO_curve_8.mat');
I_DBO_1 = I_DBO_curve;
load('I_DBO_curve_1.mat');
I_DBO_2 = I_DBO_curve;

load('I_DBO_curve_2.mat');
I_DBO_3 = I_DBO_curve;
load('I_DBO_curve_7.mat');
I_DBO_4 = I_DBO_curve;
load('I_DBO_curve_4.mat');
I_DBO_5 = I_DBO_curve;
load('I_DBO_curve_5.mat');
I_DBO_6 = I_DBO_curve;
load('I_DBO_curve_9.mat');
I_DBO_7 = I_DBO_curve;
load('I_DBO_curve_10.mat');
I_DBO_8 = I_DBO_curve;
load('I_DBO_curve_11.mat');
I_DBO_9 = I_DBO_curve;

%% 绘制对应的曲线
% 调整线条的颜色：'red'、'green'、'blue'、'cyan'、'magenta'、'yellow'、'black'、'white'
figure;
semilogy(I_DBO_6,'Color','cyan','LineWidth',2,'LineStyle',':');
hold on;
semilogy(I_DBO_9,'Color','red','LineWidth',2,'LineStyle',':');  
semilogy(I_DBO_3,'Color','blue','LineWidth',2);
semilogy(I_DBO_8,'Color','green','LineWidth',2,'LineStyle',':');
semilogy(I_DBO_5,'Color','black','LineWidth',1.5);
semilogy(I_DBO_1,'Color','red','LineWidth',2);
semilogy(I_DBO_4,'Color','green','LineWidth',2,'LineStyle','-');
semilogy(I_DBO_7,'Color','blue','LineWidth',2,'LineStyle',':');
semilogy(I_DBO_2,'Color','black','LineWidth',2,'LineStyle','-.');
%%
title('Robot Inverse Kinematic based on Improved Dung beetle optimizer algorithm')
xlabel('Iteration');
ylabel('Best score obtained so far');


axis tight
grid on
box on
legend('I-DBO-1','I-DBO-2','I-DBO-3','I-DBO-4','I-DBO-5','I-DBO-6','I-DBO-7','I-DBO-8','I-DBO-9')      % 给每条线加标签

% display(['The best solution obtained by WOA is : ', num2str(Best_pos)]);
% display(['The best optimal value of the objective funciton found by WOA is : ', num2str(Best_score)]);