%% 六足机器人的节律运动步态(3/4/5足步态)
%**************************************************************************
%******************【Authors：AndyGao；Date：2021/1】**********************
% 为了模块化设计，设置机器人在每个步幅结束后都恢复到初始位形
% 机体运动是【先平移再旋转】，关于各个方向的实际运动步幅都是“足量”的
%**************************************************************************
clear;    clc;

%**************************************************************************
% 【机器人的结构与位形参数】
%**************************************************************************
global BR L1 L2 L3 L4 Alpha
% 机身半径R，腿部各连杆参数L，吸盘半径r （单位：m）
L1 = 0.09;    L2 = 0.15;    L3 = 0.16;    L4 = 0.15;    BR = 0.18;
% 六条支链分布角 （单位：rad）
Alpha = [pi/6, pi/2, 5*pi/6, -5*pi/6, -pi/2, -pi/6];

global Theta_0 beta_0 phi_0
Theta_0 = zeros(6,4);   % 各关节角度初值 （单位：rad）
beta_0 = (pi/180)*(0);  % 吸盘的相对初始转角 （单位：rad）
phi_0 = (pi/180)*(0);   % 机体平台的初始俯仰角 （单位：rad）

%**************************************************************************
% 【单步幅周期内，足端的运动参数设置】
%**************************************************************************
global liftH Xstride Ystride Zrotate
liftH = 0.10;           % 摆动相的抬腿高度(LiftH)（单位：m）
% 机体质心(在三个自由方向)的期望步幅(Stride)
Xstride = input('***提示*** 请输入机体质心沿着 X轴 方向的期望步幅Xstride（单位：m）：');
Ystride = input('***提示*** 请输入机体质心沿着 Y轴 方向的期望步幅Ystride（单位：m）：');
Zrotate = input('***提示*** 请输入机体质心绕着 Z轴 自旋的期望步幅Zrotate（单位：deg）：') * (pi/180);

% 摆动相-支撑相标志符（1：摆动相；-1：支撑相）
gait_flag = input('***提示*** 请确定机器人的步态模式【3/4/5 足？】：');
switch gait_flag
    case {3},   SwiSur = [1 -1 1 -1 1 -1;  -1 1 -1 1 -1 1];
    case {4},   SwiSur = [1 -1 -1 1 -1 -1;  -1 1 -1 -1 1 -1;  -1 -1 1 -1 -1 1];
    case {5},   SwiSur = [1 -1 -1 -1 -1 -1;  -1 1 -1 -1 -1 -1;  -1 -1 1 -1 -1 -1;  -1 -1 -1 1 -1 -1;  -1 -1 -1 -1 1 -1;  -1 -1 -1 -1 -1 1];
    otherwise,  error('！！！步态模式错误！！！');
end
[numGroup, ~] = size(SwiSur);   % 摆动周期数
floorfactor = gait_flag / 6;    % 占地系数
dutyfactor = 1 - floorfactor;   % 占空比

%**************************************************************************
% 【单摆动周期内，足端的运动参数设置】
%**************************************************************************
swigTend = 1;           % 单摆动周期时长【可自定义】
swigNum = 100;          % 单摆动周期插值点数【若改动，则vrep_main中也需作相应修改】
dt = swigTend / swigNum;% 采样时间间隔
SwigTimeSeq = linspace(dt,swigTend, swigNum)'; % 单摆动周期的时间序列
% 【单摆动周期内的Sigmoid型插值函数】
const = 20;             % Sigmoid插值参数
Tg = swigTend / 2;      % Sigmoid插值中的给定时间参数
Gamma = 1 ./ (1 + exp(-const .* (SwigTimeSeq - Tg)));	% Sigmoid函数  % -Tg相当于给S函数作平移

%**************************************************************************
% 【单步幅周期内，机体(相对于初始机体系)的瞬时位移】
%**************************************************************************
Gamma_B = [];           % 机体位移的插值函数(单步幅周期)
for r = 1 : numGroup
    Gamma_B = [Gamma_B; (r-1)*dutyfactor + dutyfactor*Gamma];  % 相当于机体的位移在一个步态周期内的变化曲线也为S函数
end
Xstride_i = Xstride .* Gamma_B; % 机体相对于初始机体系x轴的瞬时平移
Ystride_i = Ystride .* Gamma_B; % 机体相对于初始机体系y轴的瞬时平移
Zrotate_i = Zrotate .* Gamma_B; % 机体相对于初始机体系z轴的瞬时旋转
% 【单步幅】机心相对于初始机体系的瞬时位姿 [srBx;srBy;srByaw]
StriRelBodyPose = [Xstride_i'; Ystride_i'; Zrotate_i']; % [srBx;srBy;srByaw]

%**************************************************************************
% 【单步幅周期内，足端(相对于瞬时机体系)的位置】
%**************************************************************************
StriRelFootPose = cell(6,1);	% 【单步幅】足端相对于瞬时机体系的位置 [srFx;srFy;srFz]
StriTheta = cell(6,1);          % 【单步幅】关节角度变量 [q1;q2;q3;q4]

for r = 1 : numGroup
for L = 1 : 6
    alphaL = Alpha(L); % 当前支链的方位角
    % 足端相对于机体系的初始位置
    Px_0 = BR*cos(alphaL) + (L1 + L2*cos(Theta_0(L,2)) + L3*sin(Theta_0(L,2)+Theta_0(L,3)) + L4*sin(Theta_0(L,2)+Theta_0(L,3)+Theta_0(L,4))) * cos(alphaL + Theta_0(L,1));
    Py_0 = BR*sin(alphaL) + (L1 + L2*cos(Theta_0(L,2)) + L3*sin(Theta_0(L,2)+Theta_0(L,3)) + L4*sin(Theta_0(L,2)+Theta_0(L,3)+Theta_0(L,4))) * sin(alphaL + Theta_0(L,1));
    Pz_0 = L2*sin(Theta_0(L,2)) - L3*cos(Theta_0(L,2)+Theta_0(L,3)) - L4*cos(Theta_0(L,2)+Theta_0(L,3)+Theta_0(L,4));
    % 腿部状态的符号标签：摆动相为1，支撑相为-1
    sgnL = SwiSur(r,L);
    
    for k = 1 : swigNum
        kk = k + (r - 1) * swigNum; %  ？？
        %--------------------------------------------------------------------------
        % 单步幅周期内，足端相对于瞬时机体系的位置
        %--------------------------------------------------------------------------
        % 判断当前足是否已摆动过，更新该足相对于初始机体系的位置
        Px_0c = Px_0;
        Py_0c = Py_0;
        if ( (r > 1) && ismember(1, SwiSur(1:r-1,L)) ) % 真—>说明该足已摆动过
            Px_0c = Px_0 * cos(Zrotate) - Py_0 * sin(Zrotate) + Xstride;
            Py_0c = Px_0 * sin(Zrotate) + Py_0 * cos(Zrotate) + Ystride;
        end
        % 足端相对于瞬时机体系的瞬时位置坐标
        StriRelFootPose{L}(1,kk) = (Px_0c - Xstride_i(kk)) * cos(Zrotate_i(kk)) + (Py_0c - Ystride_i(kk)) * sin(Zrotate_i(kk)) ...
            + (1+sgnL)/2 * Gamma(k) * (Px_0 * cos(Zrotate - Zrotate_i(kk)) - Py_0 * sin(Zrotate - Zrotate_i(kk)) - (Px_0 - Xstride) * cos(Zrotate_i(kk)) - (Py_0 - Ystride) * sin(Zrotate_i(kk)));
        StriRelFootPose{L}(2,kk) =-(Px_0c - Xstride_i(kk)) * sin(Zrotate_i(kk)) + (Py_0c - Ystride_i(kk)) * cos(Zrotate_i(kk)) ...
            + (1+sgnL)/2 * Gamma(k) * (Px_0 * sin(Zrotate - Zrotate_i(kk)) + Py_0 * cos(Zrotate - Zrotate_i(kk)) + (Px_0 - Xstride) * sin(Zrotate_i(kk)) - (Py_0 - Ystride) * cos(Zrotate_i(kk)));
        StriRelFootPose{L}(3,kk) = Pz_0 + (1+sgnL)/2 * liftH * (-0.5 * cos(2*pi/swigTend * SwigTimeSeq(k)) + 0.5);
        
        %--------------------------------------------------------------------------
        % 单步幅周期内，运动学逆解
        %--------------------------------------------------------------------------
        Px_cur = StriRelFootPose{L}(1,kk); % 当前时刻的足端横坐标
        Py_cur = StriRelFootPose{L}(2,kk); % 当前时刻的足端纵坐标
        Pz_cur = StriRelFootPose{L}(3,kk); % 当前时刻的足端竖坐标
        phi_i  = phi_0;                 % 机体平台的瞬时俯仰角
        beta_i = beta_0;                % 吸盘的转角
        % 当前时刻的关节变量
        theta1 = atan2(Py_cur - BR*sin(alphaL), Px_cur - BR*cos(alphaL)) - alphaL;
        tempA = Pz_cur + L4*cos(beta_i-sign(alphaL)*phi_i);
        tempB = (Py_cur - BR*sin(alphaL))/sin(alphaL+theta1) - L1 - L4*sin(beta_i-sign(alphaL)*phi_i);
        tempC = L2;
        tempD = -L3;
        theta3 = asin((tempA^2+tempB^2-tempC^2-tempD^2) / (-2*tempC*tempD));
        theta2 = atan((-2*tempC*tempD*cos(theta3)) / (tempA^2+tempB^2+tempC^2-tempD^2)) + atan2(tempA , tempB);
        theta4 = beta_i - sign(alphaL)*phi_i - theta2 - theta3;
        % 存储关节变量
        Theta_ik = [theta1; theta2; theta3; theta4];
        StriTheta{L} = [StriTheta{L}, Theta_ik]; % 单步幅内各关节角度值
    end
end
end


%% **************************************************************************
% 【多步幅连续节律运动】
%**************************************************************************
% 节律运动参数设置
rhyN = 10;                       % 周期节律运动的步幅数【可自定义】
rhyTend = numGroup*swigTend*rhyN;	% 周期节律运动的总时长
rhyNum = numGroup*swigNum*rhyN;      % 周期节律运动的采样点数
RhyRelFootPose = cell(6,1);     % 节律运动的足端(相对于瞬时机体系)相对位姿 % [rrFx;rrFy;rrFz]
RhyAbsFootPose = cell(6,1);     % 节律运动的足端(相对于世界系)绝对位姿 % [raFx;raFy;raFz]
RhyTheta = cell(6,1);           % 节律运动的关节位移 [q1;q2;q3;q4]
RhyRelBodyPose = cell(1,rhyN);  % 节律运动的机心(相对于初始机体系)相对位姿 [rrBx;rrBy;rrByaw]
RhyAbsBodyPose = cell(1,rhyN);  % 节律运动的机心(相对于世界系)绝对位姿 [raBx;raBy;raByaw]
RhyTimeSeq = linspace(dt,rhyTend, rhyNum); % 节律运动的仿真时间序列

% 节律运动的相对位移
for n = 1 : rhyN
    for L = 1 : 6
        RhyRelFootPose{L} = [RhyRelFootPose{L}, StriRelFootPose{L}];	% [rrFx;rrFy;rrFz]
        RhyTheta{L} = [RhyTheta{L}, StriTheta{L}];                      % [q1;q2;q3;q4]
    end
    RhyRelBodyPose{n} = StriRelBodyPose; % [rrBx;rrBy;rrByaw]
end

% 节律运动的机体绝对位姿
for n = 1 : rhyN
    if (n == 1)
        RhyAbsBodyPose{n} = RhyRelBodyPose{n};
        continue;
    end
    for i = 1 : numGroup*swigNum
        RhyAbsBodyPose{n}(3,i) = RhyAbsBodyPose{n-1}(3,end) + RhyRelBodyPose{n}(3,i);
        RhyAbsBodyPose{n}(1:2,i) = RhyAbsBodyPose{n-1}(1:2,end) + rot2(RhyAbsBodyPose{n-1}(3,end)) * RhyRelBodyPose{n}(1:2,i);
    end
end
% 节律运动的足端绝对位置
RhyAbsBodyPose_mat = cell2mat(RhyAbsBodyPose);
for i = 1 : rhyNum
    for L = 1 : 6
        RhyAbsFootPose{L}(1:2,i) = RhyAbsBodyPose_mat(1:2,i) + rot2(RhyAbsBodyPose_mat(3,i)) * RhyRelFootPose{L}(1:2,i);
        RhyAbsFootPose{L}(3,i) = RhyRelFootPose{L}(3,i);
    end
end


%% **************************************************************************
% 【绘图】
%**************************************************************************
for L = 1 : 6
    figure(L);    clf;
    hold on;    box on;
    plot(RhyTimeSeq, RhyTheta{L}(1,:), '-k','LineWidth',2);
    plot(RhyTimeSeq, RhyTheta{L}(2,:), '-r','LineWidth',2);
    plot(RhyTimeSeq, RhyTheta{L}(3,:), '-g','LineWidth',2);
    plot(RhyTimeSeq, RhyTheta{L}(4,:), '-b','LineWidth',2);
    plot(RhyTimeSeq, RhyRelFootPose{L}(1,:), '--r','LineWidth',3);
    plot(RhyTimeSeq, RhyRelFootPose{L}(2,:), '--g','LineWidth',3);
    plot(RhyTimeSeq, RhyRelFootPose{L}(3,:), '--b','LineWidth',3);
end

figure(98);    clf;
hold on;    box on;
plot(RhyTimeSeq, RhyAbsBodyPose_mat(1:2,:), 'LineWidth',2);

figure(99);    clf;
hold on;    box on;
plot3(RhyAbsBodyPose_mat(1,:), RhyAbsBodyPose_mat(2,:), zeros(1,rhyNum), 'r-','LineWidth',5);
for L = 1 : 6
    plot3(RhyAbsFootPose{L}(1,:), RhyAbsFootPose{L}(2,:), RhyAbsFootPose{L}(3,:), 'LineWidth',2);
end
xlabel('x');    ylabel('y');    zlabel('z');
view([-1,-1,1]);
legend('Body','Foot1','Foot2','Foot3','Foot4','Foot5','Foot6', 'Location','northeast');
hold off;


%% 存储各关节角度值
JointAngles = RhyTheta;
save(['Joint_Datas\Gait',num2str(gait_flag),'_JointAngles'], 'JointAngles');

disp('***** 执行完毕！*****');
