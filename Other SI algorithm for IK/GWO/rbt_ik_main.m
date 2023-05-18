%___________________________________________________________________%
%  Grey Wolf Optimizer (GWO) source codes version 1.0               %
%                                                                   %
%  Developed in MATLAB R2011b(7.13)                                 %
%                                                                   %
%  Author and programmer: Seyedali Mirjalili                        %
%                                                                   %
%         e-Mail: ali.mirjalili@gmail.com                           %
%                 seyedali.mirjalili@griffithuni.edu.au             %
%                                                                   %
%       Homepage: http://www.alimirjalili.com                       %
%                                                                   %
%   Main paper: S. Mirjalili, S. M. Mirjalili, A. Lewis             %
%               Grey Wolf Optimizer, Advances in Engineering        %
%               Software , in press,                                %
%               DOI: 10.1016/j.advengsoft.2013.12.007               %
%                                                                   %
%___________________________________________________________________%

% You can simply define your cost in a seperate file and load its handle to fobj 
% The initial parameters that you need are:
%__________________________________________
% fobj = @YourCostFunction
% dim = number of your variables
% Max_iteration = maximum number of generations
% SearchAgents_no = number of search agents
% lb=[lb1,lb2,...,lbn] where lbn is the lower bound of variable n
% ub=[ub1,ub2,...,ubn] where ubn is the upper bound of variable n
% If all the variables have equal lower bound you can just
% define lb and ub as two single number numbers

% To run GWO: [Best_score,Best_pos,GWO_cg_curve]=GWO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj)
%__________________________________________

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
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','Arm6');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
SearchAgents_no=30; % Number of search agents

% Function_name='F10'; % Name of the test function that can be from F1 to F23 (Table 1,2,3 in the paper)

Max_iteration=100; % Maximum numbef of iterations

% Load details of the selected benchmark function
% [lb,ub,dim,fobj]=Get_Functions_details(Function_name);


%% 设置约束条件
lb = [-360,-118,-225,-360,-97,-360];         % 设置每个关节的转角下限制
ub = [360,120,11,360,180,360];                % 设置每个关节的转角上限制
dim = 6;
fobj = @(x)rbt_ik_Function(x);

tic
[Best_score,Best_pos,GWO_cg_curve]=GWO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
GWO_time = toc;

figure;
%Draw search space
% subplot(1,2,1);
% func_plot(rbt_ik_Function);
% title('Parameter space')
% xlabel('x_1');
% ylabel('x_2');
% zlabel([Function_name,'( x_1 , x_2 )'])

%Draw objective space
% subplot(1,2,2);
semilogy(GWO_cg_curve,'Color','g','LineWidth',1)
title('Robot Inverse Kinematic based GWO Algorithm')
xlabel('Iteration');
ylabel('Best score obtained so far');

% axis tight
grid on
box on
legend('GWO')

display(['The best solution obtained by GWO is : ', num2str(Best_pos)]);
display(['The best optimal value of the objective funciton found by GWO is : ', num2str(Best_score)]);

%% 按最优解控制机器人，验证逆解的正确性
Theta=Best_pos/180*pi;               %换算成弧度
T=robot.fkine(Theta);             %求正解的齐次变换矩阵
W=[-800,+800,-800,+800,-800,+800];
figure;
robot.plot(Theta,'tilesize',150,'workspace',W);  %显示三维动画
qi=robot.ikcon(T)*180/pi;      %求逆解验证关节角,ikcon(with joint limits),ikine(without joint limits)
rot = t2r(T);                  %齐次矩阵变换为旋转矩阵
Qua= UnitQuaternion(T);         %求四元数 
rpy=tr2rpy(T, 'zyx')*180/pi;    %求末端姿态，工具法兰为绕XYZ轴旋转 tr2rpy输出值默认对应的顺序是ZYX
robot.teach(T,'rpy' )          %显示roll/pitch/yaw angles，GUI可调界面
        



