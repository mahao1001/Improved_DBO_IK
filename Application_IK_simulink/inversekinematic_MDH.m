clear,clc,close all;
% %% 建立机器人标准S-DH参数，初始姿态
% L1 = Link('d',267,'a',0,'alpha',-pi/2,'standard');
% L2 = Link('d',0,'a',289.48866,'alpha',0,'offset',-1.3849179,'standard');
% L3 = Link('d',0,'a',77.5,'alpha',-pi/2,'offset',1.3849179,'standard');
% L4 = Link('d',342.5,'a',0,'alpha',pi/2,'standard');
% L5 = Link('d',0,'a',76,'alpha',-pi/2,'standard');
% L6 = Link('d',97,'a',0,'alpha',0,'standard');
% qlim=[-360,360;-118,120;-225,11;-360,360;-97,180;-360,360]  % 设置每个关节的转角限制
% robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Cobot');
%% 建立机器人修改的M-DH参数，初始状态
% 连杆偏移d,连杆长度a,连杆扭转角alpha
L1=Link('d',267,'a',0,'alpha',0,'modified'); 
L2=Link('d',0,'a',0,'alpha',-pi/2,'offset',-1.3849179,'modified');
L3=Link('d',0,'a',289.48866,'alpha',0,'offset',1.3849179,'modified');
L4=Link('d',342.5,'a',77.5,'alpha',-pi/2,'modified');
L5=Link('d',0,'a',0,'alpha',pi/2,'modified');
L6=Link('d',97,'a',76,'alpha',-pi/2,'modified');
qlim=[-360,360;-118,120;-225,11;-360,360;-97,180;-360,360];  % 设置每个关节的转角限制
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','Arm6');
%% 显示机械臂的信息
robot.links
robot.name
robot.display;
%% 正解，给定关节角，求末端位姿
% Theta = [0 0 0 0 0 0];  %给定6个关节角度值
% Theta=[-30 -30 -30 -30 -30 -30];
%% 逆解，给定末端位姿求关节角，(X,Y,Z单位为mm)，(Rall,Pitch,Yaw单位为deg)
% Pos=[308.0304 -367.2397  524.1307  43.8979  -25.6589   56.3099];         %给定末端位姿，前面三个是位置，后面三个是RPY角
Pos=[300 100 200 0 0 0];
AllSloverTheta=InverseSolver_MDH(Pos);              %根据末端位姿求所有解
Theta=AllSloverTheta(1,1:end);                   %选择第一组解，PTP程序是默认为第一组解

%% 验证正逆解的结果
Theta=Theta/180*pi;                         %换算成弧度
T=robot.fkine(Theta);             %求正解的齐次变换矩阵
W=[-800,+800,-800,+800,-800,+800];
robot.plot(Theta,'tilesize',150,'workspace',W);  %显示三维动画
qi=robot.ikcon(T)*180/pi;      %求逆解验证关节角,ikcon(with joint limits),ikine(without joint limits)
rot = t2r(T);                  %齐次矩阵变换为旋转矩阵
Qua= UnitQuaternion(T);         %求四元数 

rpy=tr2rpy(T, 'zyx')*180/pi;    %求末端姿态，工具法兰为绕XYZ轴旋转 tr2rpy输出值默认对应的顺序是ZYX
robot.teach(T,'rpy' )          %显示roll/pitch/yaw angles，GUI可调界面
         