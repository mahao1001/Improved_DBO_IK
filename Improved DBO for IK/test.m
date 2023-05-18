 clear
 clc

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
Theta0 = [0 0 0 0 0 0];        %当前的转角，相当于知道当前的位姿

%%
xyz = [300 100 200];    % 输入目标xyz末端位置
rpy = [-180 0 0];       % 输入目标rpy姿态角 分别为 x旋转-180 y0 z0
Txyz =transl(xyz);       % 末端位置xyz转化为齐次坐标
Trpy =rpy2tr(rpy,'zyx');       % 末端姿态rpy角转化为齐次坐标形式，输入顺序是x，y，z。单位是deg.
T = Txyz*Trpy             % 目标变换矩阵
Qua = UnitQuaternion(T);   % 目标四元数
q = double(Qua)
%%
Theta = [0     -14.4     14.4     3.6     0    -3.6];
% Theta = [0 0 0 0 0 0];
Theta_new=Theta/180*pi;    %换算成弧度
Ti=robot.fkine(Theta_new)      %求当前正解的齐次变换矩阵
Quai = UnitQuaternion(Ti);        %求当前的四元数
qi = double(Quai)                % 目标四元数 

% Quat = quaternion(rpy,'eulerd','XYZ','frame')    %角度求四元数
% eulerd(Quat,'XYZ','frame')        %四元数向量可以逆求出rpy角
%%
similarity = abs(dot(qi, q))    % 第一种方法 趋近于1相同   dot为点积
%%  %%%%%%%%%%%%%%%%%%%%%%%% 第二种方法
% q_diff = quatmultiply(qi, quatinv(q));
% q_diff = quatnormalize(q_diff);
% % o = norm(q_diff(2:4))              %向量范数和矩阵范数    趋于0代表相同
% q1_unit = quatnormalize(qi);
% q2_unit = quatnormalize(q);
% dot_product = abs(dot(qi, q))
%%

