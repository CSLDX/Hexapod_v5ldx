%% 连接vrep
clear; clc;
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',3000,true,true,5000,5);

%% 读取CPG网络输出生成关节角度
bool_flag = 0;
while (bool_flag == 0)
    gait_flag = input('***提示*** 机器人以几足步态运动（3/4/5 足？）：');
    if (gait_flag == 3)
        load('jointJ_legI_3.mat');
        bool_flag = 1;
    elseif (gait_flag == 4)
        load('jointJ_legI_4.mat');
        bool_flag = 1;
    elseif (gait_flag == 5)
        load('jointJ_legI_5.mat');
        bool_flag = 1;
    end
end

%% 足端轨迹生成器
global gait_cir3;
global gait_cir4;
global gait_cir5;

gait_cir3 = [1 0 1 0 1 0; 0 1 0 1 0 1];
gait_cir4 = [1 0 0 1 0 0; 0 0 1 0 0 1; 0 1 0 0 1 0];
gait_cir5 = [1 0 0 0 0 0; 0 0 0 0 0 1; 0 1 0 0 0 0; 0 0 0 0 1 0; 0 0 1 0 0 0; 0 0 0 1 0 0];


%% 给机器人写入关节角度
pause_time = 0.01;% 停顿时长（单位：秒）
% 关节初始角度补偿
joint_offset(1) = 0;
joint_offset(2) = -pi/6;
joint_offset(3) = 2*pi/3;

if clientID>-1
    disp('Connected to remote API server！');
    % get handle for joints 【获取关节句柄】
    for j = 0:5
        for i = 1:3
            hexa_jointI_J = ['hexa_joint',num2str(i),'_',num2str(j)];           
            % eval函数的功能是将字符串转换为matlab可执行语句
            % handle_hexa_jointI_J = vrep.simxGetObjectHandle(clientID,hexa_jointI_J,vrep.simx_opmode_oneshot_wait);
            eval(['[~,handle_',hexa_jointI_J,'] = vrep.simxGetObjectHandle(clientID,hexa_jointI_J,vrep.simx_opmode_oneshot_wait);'])
        end
    end
    % Set the position of every joint 【设置关节角位移】
    while vrep.simxGetConnectionId(clientID) ~= -1  % while v-rep connection is still active      
        % Moving! 【以节律步态运动】
        for m = 1 : length(Output_jointJ_legI(:,1,1))
            vrep.simxPauseCommunication(clientID,1);
            for j = 1:6 
                for joint_i = 1:3
                    % 角度单位：弧度rad
                    vrep.simxSetJointTargetPosition(clientID, eval(['handle_hexa_joint',num2str(joint_i),'_',num2str(j-1)]),...
                        Output_jointJ_legI(m,joint_i,j)+joint_offset(joint_i), vrep.simx_opmode_oneshot);
                end
            end
            vrep.simxPauseCommunication(clientID,0);
            pause(pause_time);
        end

        break;
    end
    % pause(); % 按任意键继续执行程序
    input('提示——请按任意键继续执行程序：'); 
    %**********************************************************************
    %*************************【停止仿真并断开连接】************************
    %**********************************************************************
    % Stop simulation and close the connection to V-REP:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID);
    % Now close the connection to V-REP:
    vrep.simxFinish(clientID);
else
    fprintf('\n!!!Warning: Failed connecting to remote API server !!! \n');
end

%% 程序结束
vrep.delete(); % call the destructor!
disp('! Program ended !');











