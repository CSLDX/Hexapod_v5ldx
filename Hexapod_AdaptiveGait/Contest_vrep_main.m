%% main_function 【动力学：力矩控制机器人】
%**************************************************************************
%*****************【Authors：AndyGao/LDX；Date：2021/07】**********************
%**************************************************************************
clear all;  clc;
disp('Program started');
%% 加载足端轨迹
% load('Leg_Pos5.mat');
% gait_num = length(leg_pos{1});
%% 六足机器结构参数 长度单位：m 质量单位：kg 角度单位：rad
% LL1 = 0.0514982;
% LL2 = 0.0723751;
% LL3 = 0.1165047;
% ML1 = 0.05;
% ML2 = 0.05;
% ML3 = 0.05;
% theta_c1 = pi/180*0;
% theta_c2 = -pi/180*30;
% theta_c3 = pi/180*120;

%% ******************* 【MATLAB与V-rep建立连接】 *********************
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
% 停顿时长（单位：秒）
pause_time = 0.05;
dt = 0.05;
%% ********************** 【机器人足端轨迹规划】 ********************
if (clientID>-1)
    disp('Connected to remote API server！');
    %**********************************************************************
    % get handle for joints 【获取关节句柄】
    [~,handle_hexa_legBase] = vrep.simxGetObjectHandle(clientID,'hexa_legBase',vrep.simx_opmode_oneshot_wait);
    for j = 1 : 6
        hexa_footTargetx = ['hexa_footTarget',num2str(j-1)];
        %eval函数的功能是将字符串转换为matlab可执行语句
        eval(['[~,handle_',hexa_footTargetx,'] = vrep.simxGetObjectHandle(clientID,hexa_footTargetx,vrep.simx_opmode_oneshot_wait);'])
    end
    %******************************************************************
    vrep.simxSynchronous(clientID,1); % 使能同步
    %******************************************************************
    % 足端相对位置初始化
    for i = 1:6
        [~,offset(i,:)] = vrep.simxGetObjectPosition(clientID, eval(['handle_hexa_footTarget',num2str(i-1)]),handle_hexa_legBase,vrep.simx_opmode_streaming);
    end    
    vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);
    % 足端相对位置读取
    for i = 1:6
        [~,offset(i,:)] = vrep.simxGetObjectPosition(clientID, eval(['handle_hexa_footTarget',num2str(i-1)]),handle_hexa_legBase,vrep.simx_opmode_buffer);
    end    
    vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);
    % N 步数
    N = 1000;
    Zrotate = 0;
    %**********************************************************************
    while(vrep.simxGetConnectionId(clientID) ~= -1)  % while v-rep connection is still active
        for j = 1:N
            [gait_num, leg_pos] = GRG(0.03,0.03,Zrotate,3);
            for tt = 1:gait_num         
                % 输出足端轨迹
                for i = 1 : 6
                    vrep.simxSetObjectPosition(clientID, eval(['handle_hexa_footTarget',num2str(i-1)]),handle_hexa_legBase,[leg_pos{i,1}(1,tt)+offset(i,1),leg_pos{i,1}(2,tt)+offset(i,2),leg_pos{i,1}(3,tt)+offset(i,3)],vrep.simx_opmode_oneshot);              
                end
                vrep.simxSynchronousTrigger(clientID);
                vrep.simxGetPingTime(clientID);       
            end
            if ~mod(j,2)
                Zrotate = Zrotate+10;
            end
        end
        break;
    end
    
%% 作图
%     figure(11);
% %     plot(t,theta_d(1,:),t,theta_c(1,:),t,torque(1,:));
%     plot(t,theta_d(1,:),t,theta_c(1,:));
%     xlabel('t');
%     ylabel('角度/rad');
% %     legend('Theta1期望角度','Theta1实际角度','输出力矩1');
%     legend('Theta1期望角度','Theta1实际角度');
%     set(gca,'XTick',1:200:length(t));
%     figure(22);
% %     plot(t,theta_d(2,:),t,theta_c(2,:),t,torque(2,:));
%     plot(t,theta_d(2,:),t,theta_c(2,:));
%     xlabel('t');
%     ylabel('角度/rad');
% %     legend('Theta2期望角度','Theta2实际角度','输出力矩2');
%     legend('Theta2期望角度','Theta2实际角度');
%     set(gca,'XTick',1:200:length(t));
%     figure(33);
% %     plot(t,theta_d(3,:),t,theta_c(3,:),t,torque(3,:));
%     plot(t,theta_d(3,:),t,theta_c(3,:));
%     xlabel('t');
%     ylabel('角度/rad');
% %     legend('Theta3期望角度','Theta3实际角度','输出力矩3');
%     legend('Theta3期望角度','Theta3实际角度');
%     set(gca,'XTick',1:200:length(t));
%     save('train_torque.mat','torque');
%     save('train_theta_c.mat','theta_c');
%% 结束    
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