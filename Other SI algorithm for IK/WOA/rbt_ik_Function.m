function o = rbt_ik_Function(Theta)
% clear;clc;

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
xyz = [300 100 200];    % 输入目标xyz末端位置
rpy = [-180 0 0];       % 输入目标rpy姿态角
% Txyz =transl(xyz);       % 末端位置xyz转化为齐次坐标
% Trpy =rpy2tr(rpy,'xyz');       % 末端姿态rpy角转化为齐次坐标形式，输入顺序是z，y，x。单位是deg.
% T = Txyz*Trpy;          % 变换矩阵
% Qua= UnitQuaternion(T);        %求四元数
% rpy=tr2rpy(T, 'xyz')*180/pi;    %验证末端姿态，工具法兰为绕XYZ轴旋转
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  
    Theta=Theta/180*pi;          %换算成弧度
    Ti = robot.fkine(Theta);     % 正向运动学得到当前的变换矩阵
    rpyi = tr2rpy(Ti, 'zyx')*180/pi;    %验证末端姿态，工输出为绕 X Y Z轴旋转值
    xyzi = Ti.t';                       %新的末端位置xyz  
    %% 目标函数
    P = xyzi - xyz;        %位差值矩阵
    Q = rpyi - rpy;        %姿差值矩阵
    o =  0.55*sqrt (P(1)^2+P(2)^2+P(3)^2) + 0.45*sqrt (Q(1)^2+Q(2)^2+Q(3)^2);   % 求这个位差最小

end
