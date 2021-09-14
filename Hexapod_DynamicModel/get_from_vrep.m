%% 获取参数
%**************************************************************************
%*****************【Authors：AndyGao；Date：2020/05】**********************
%**************************************************************************
clear all;  clc;
disp('Program started');
%% ******************* 【MATLAB与V-rep建立连接】 *********************
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%% ********************** 【读关节角度坐标】 ********************
if (clientID>-1)
    disp('Connected to remote API server！');
    %**********************************************************************
    % get handle for joints 【获取关节句柄】 e.g. handle_hexa_joint2_0
    for i = 1 : 3
        hexa_jointI_J = ['hexa_joint',num2str(i),'_0'];
        %eval函数的功能是将字符串转换为matlab可执行语句
        eval(['[~,handle_',hexa_jointI_J,'] = vrep.simxGetObjectHandle(clientID,hexa_jointI_J,vrep.simx_opmode_oneshot_wait);'])
    end
    [~,handle_hexa_footTip0] = vrep.simxGetObjectHandle(clientID,'hexa_footTip0',vrep.simx_opmode_oneshot_wait);
    [~,handle_hexa_body] = vrep.simxGetObjectHandle(clientID,'hexa_body',vrep.simx_opmode_oneshot_wait);
    
    %**********************************************************************
    while(vrep.simxGetConnectionId(clientID) ~= -1)  % while v-rep connection is still active
        %******************************************************************
        [num1,link1_pos] = vrep.simxGetObjectPosition(clientID,handle_hexa_joint2_0,handle_hexa_joint1_0,vrep.simx_opmode_blocking); % must use simx_opmode_blocking, or you will get nothing!
        [num2,link2_pos] = vrep.simxGetObjectPosition(clientID,handle_hexa_joint3_0,handle_hexa_joint2_0,vrep.simx_opmode_blocking);
        [num3,link3_pos] = vrep.simxGetObjectPosition(clientID,handle_hexa_footTip0,handle_hexa_joint3_0,vrep.simx_opmode_blocking);
        [num0,link0_pos] = vrep.simxGetObjectPosition(clientID,handle_hexa_body,handle_hexa_joint1_0,vrep.simx_opmode_blocking);
        length_link0 = norm(link0_pos);
        length_link1 = norm(link1_pos);
        length_link2 = norm(link2_pos);
        length_link3 = norm(link3_pos);
        disp('Already get ！');
        %******************************************************************
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