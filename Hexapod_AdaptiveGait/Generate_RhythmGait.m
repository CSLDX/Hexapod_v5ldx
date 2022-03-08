%% 六足机器人的节律运动步态(3/4/5足步态)
%**************************************************************************
%******************【Authors：LDX；Date：2021/7】**********************
% 为了模块化设计，设置机器人在每个步幅结束后都恢复到初始位形
% 将足端轨迹连续的函数离散化，离散点可调，将离散点按步态相位差规划离散点
%**************************************************************************
clear;    clc;
%**************************************************************************
% 【机器人的结构与位形参数】
%**************************************************************************
global BR Alpha LL1 LL2 LL3 Theta1 Theta2 Theta3
% 机身半径R（单位：m）
BR = 0.0800602;
LL1 = 0.0514982;
LL2 = 0.0723751;
LL3 = 0.1165047;
Theta1 = pi/180*0;
Theta2 = -pi/180*30;
Theta3 = pi/180*120;
% 六条支链分布角 （单位：rad）
Alpha = [0, pi/3, 2*pi/3, pi, -2*pi/3, -pi/3];
offset_x = 0;
offset_y = 0;
offset_z = 0;

ST = 0;
SW_UP = 1;
SW_DOWN = 2;

%% 【单周期内，足端的运动参数设置】 仿真步长10ms
% 摆动相-支撑相标志符（1：摆动相；-1：支撑相）
global stepHeight stepAmplitude Zrotate
stepHeight = input('***提示*** 请输入摆动腿高度stepHeight（单位：m）：'); 
stepAmplitude = input('***提示*** 请输入期望步幅stepAmplitude（单位：m）：');
Zrotate = input('***提示*** 请输入前进方向角（单位：deg）：') * (pi/180);
gait_flag = input('***提示*** 请确定机器人的步态模式【3/4/5 足？】：');
% e.g: 0.03 0.03 any 3/4/5

switch gait_flag
    case {3},   SwiSur = [1 -1 1 -1 1 -1;  -1 1 -1 1 -1 1];
    case {4},   SwiSur = [1 -1 -1 1 -1 -1;  -1 1 -1 -1 1 -1;  -1 -1 1 -1 -1 1];
    case {5},   SwiSur = [1 -1 -1 -1 -1 -1;  -1 1 -1 -1 -1 -1;  -1 -1 1 -1 -1 -1;  -1 -1 -1 1 -1 -1;  -1 -1 -1 -1 1 -1;  -1 -1 -1 -1 -1 1];
    otherwise,  error('！！！步态模式错误！！！');
end
[numGroup, ~] = size(SwiSur);   % 周期数
floorfactor = gait_flag / 6;    % 占地系数
dutyfactor = 1 - floorfactor;   % 占空比
ksw = round(floorfactor/dutyfactor);    % 放作摆动相的时间系数

%% 足端轨迹函数，离散化
% igTend = 1;           % 单摆动周期时长
% swigNum = 15;          % 单摆动周期插值点数
% stigNum = ksw*swigNum;  % 单支撑周期插值点数
% dt = igTend / swigNum;% 采样时间间隔
% SwigTimeSeq = linspace(0,igTend, swigNum)./igTend; % 单摆动周期的时间序列
% StigTimeSeq = linspace(0,igTend, stigNum)./igTend;
% 
% % 【单摆动周期内的正弦函数】
% Swz = stepHeight*sin(SwigTimeSeq.*pi);
% Swx = SwigTimeSeq.*stepAmplitude*cos(Zrotate);
% Swy = SwigTimeSeq.*stepAmplitude*sin(Zrotate);
% % 【单支撑周期内的正弦函数】
% Stz = zeros(1, length(StigTimeSeq));
% Stx = fliplr(StigTimeSeq).*stepAmplitude*cos(Zrotate);
% Sty = fliplr(StigTimeSeq).*stepAmplitude*sin(Zrotate);

% new
swigNum_sw = 40;          % 单摆动周期插值点数
swigNum_up = 30;
swigNum_down = 30;

swigNum = swigNum_sw+swigNum_up+swigNum_down;
stigNum = ksw*swigNum;  % 单支撑周期插值点数
% 单摆动周期的时间序列
SwigTimeSeq_sw = linspace(0,1, swigNum_sw); 
SwigTimeSeq_up = linspace(0,1, swigNum_up);
SwigTimeSeq_down = linspace(0,1, swigNum_down);
SwigTimeSeq = linspace(0,1, swigNum);
% 单支撑周期的时间序列
%     StigTimeSeq = linspace(0,1, stigNum);
Gait_num = swigNum + stigNum;
% 【单摆动周期内的正弦函数】  
Swx = [0*ones(1,swigNum_up), stepAmplitude*cos(Zrotate)*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*cos(Zrotate)*ones(1,swigNum_down)];
Swy = [0*ones(1,swigNum_up), stepAmplitude*sin(Zrotate)*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*sin(Zrotate)*ones(1,swigNum_down)];
Swz = stepHeight*(1-(0.5*(cos(SwigTimeSeq.*2*pi)+1)));
% 【单支撑周期内的正弦函数】
stx = [zeros(1,swigNum_down), stepAmplitude*cos(Zrotate)*1/ksw*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*cos(Zrotate)*1/ksw*ones(1,swigNum_up)];
sty = [zeros(1,swigNum_down), stepAmplitude*sin(Zrotate)*1/ksw*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*sin(Zrotate)*1/ksw*ones(1,swigNum_up)];
Stx = stx;
Sty = sty;
for i = 1:ksw-1
    Stx = [Stx, stx+stepAmplitude*cos(Zrotate)*i/ksw];
    Sty = [Sty, sty+stepAmplitude*sin(Zrotate)*i/ksw];
end
Stx = fliplr(Stx);
Sty = fliplr(Sty);
Stz = zeros(1, stigNum);


%% 按相位差规划足端轨迹离散点
leg_pos = cell(6,1);
for leg = 1:6
    % 步态，延时各腿足端轨迹
    flag = mod(leg,numGroup);
    if(flag == 1)
        leg_z = [Swz,Stz];
        leg_y = [Swy,Sty];
        leg_x = [Swx,Stx];
        pos_state = [SW_UP*ones(1,sum(gradient(Swz)>=0)),SW_DOWN*ones(1,sum(gradient(Swz)<0)),ST*ones(1,length(Stz))];
    else
        if(flag == 0)
            leg_z = [Stz,Swz];
            leg_y = [Sty-stepAmplitude*sin(Zrotate),Swy-stepAmplitude*sin(Zrotate)];
            leg_x = [Stx-stepAmplitude*cos(Zrotate),Swx-stepAmplitude*cos(Zrotate)];
            pos_state = [ST*ones(1,length(Stz)),SW_UP*ones(1,sum(gradient(Swz)>=0)),SW_DOWN*ones(1,sum(gradient(Swz)<0))];
        else
            leg_z = [Stz(end-floor(length(Stz)*(mod(leg,numGroup)-1)/(numGroup-1)):end),Swz,Stz(1:end-floor(length(Stz)*(mod(leg,numGroup)-1)/(numGroup-1))-1)];
            leg_y = [Sty(end-floor(length(Sty)*(mod(leg,numGroup)-1)/(numGroup-1)):end)-stepAmplitude*sin(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1),...
                     Swy-stepAmplitude*sin(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1),...
                     Sty(1:end-floor(length(Sty)*(mod(leg,numGroup)-1)/(numGroup-1))-1)-stepAmplitude*sin(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1)];

            leg_x = [Stx(end-floor(length(Stx)*(mod(leg,numGroup)-1)/(numGroup-1)):end)-stepAmplitude*cos(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1),...
                     Swx-stepAmplitude*cos(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1),...
                     Stx(1:end-floor(length(Stx)*(mod(leg,numGroup)-1)/(numGroup-1))-1)-stepAmplitude*cos(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1)];
            pos_state = [ST*ones(1,length(Stz(end-floor(length(Stz)*(mod(leg,numGroup)-1)/(numGroup-1)):end))),SW_UP*ones(1,sum(gradient(Swz)>=0)),SW_DOWN*ones(1,sum(gradient(Swz)<0)),ST*ones(1,length(Stz(1:end-floor(length(Stz)*(mod(leg,numGroup)-1)/(numGroup-1))-1)))];
        end
    end
    % 足端相对于legbase的偏移量【可以与vrep通信读取】
    offset_x = cos(Alpha(leg))*BR+cos(Alpha(leg)+Theta1)*(LL1+LL2*cos(Theta2)+LL3*cos(Theta3+Theta2));
    offset_y = sin(Alpha(leg))*BR+sin(Alpha(leg)+Theta1)*(LL1+LL2*cos(Theta2)+LL3*cos(Theta3+Theta2));
    offset_z = 0;
    % 一个步态周期的足端轨迹位置
    leg_pos{leg} = [offset_x+leg_x; offset_y+leg_y; offset_z+leg_z; pos_state];
end



%% **************************************************************************
% 【绘图】
%**************************************************************************
figure(1);
for L = 1 : 2
    plot(1:swigNum+stigNum, leg_pos{L,1}(1,:));
    hold on;
end
xlabel('Seq');
ylabel('/m');


figure(2);
for L = 1:6
    plot3(leg_pos{L,1}(1,:), leg_pos{L,1}(2,:), leg_pos{L,1}(3,:));

    hold on;
end
xlabel('x/m');
ylabel('y/m');
zlabel('z/m')
legend('Leg1','Leg2','Leg3','Leg4','Leg5','Leg6');
% xlim([-0.7 0.7])
% ylim([-0.4 0.4])
% zlim([0 0.5])
% view([45,45])
% 输入参数：0.2 0.5 20 3
%% 存储各关节角度值
% save(['Leg_Pos',num2str(gait_flag),'.mat'], 'leg_pos');
% disp('***** 执行完毕！*****');
