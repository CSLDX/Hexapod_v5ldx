%% 连接vrep
clear; clc;
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',3000,true,true,5000,5);

%% 发送数据点buffer 
if clientID>-1
    disp('Connected to remote API server！');
    filename = '.\ReferAbsBodyPosition.csv';
    Body_Pos = csvread(filename);
    [n,~] = size(Body_Pos);
    Body_Pos = Body_Pos.*10000;
    for i = 1:n
        vrep.simxPauseCommunication(clientID,1);
        vrep.simxSetIntegerSignal (clientID,'point_cloud_x',Body_Pos(i,1),vrep.simx_opmode_oneshot);
        vrep.simxSetIntegerSignal (clientID,'point_cloud_y',Body_Pos(i,2),vrep.simx_opmode_oneshot);
        vrep.simxSetIntegerSignal (clientID,'point_cloud_z',Body_Pos(i,3),vrep.simx_opmode_oneshot);
        vrep.simxPauseCommunication(clientID,0);
        pause(0.005);
    end 

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

