%% 【基于蜿蜒步态的幅值调整法转弯问题研究代码】
% 将关节角度数据发送给机器人
close all;  clear;  clc;
disp('Program start');

%% MATLAB与V-rep建立连接
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%% 步态参数初始化
disp('转弯问题研究')
A = deg2rad(30);
K = deg2rad(60);
W = deg2rad(90);
h = 4.70;
g = 0.98;
deltaA = deg2rad(0);
deltaA1 = deg2rad(0);
TimeStep = 0.05;                      % 设置每次循环时间
T = 0.0;                              % 设置相对时间
JointNum = 8;                        % 设置关节个数
A1 = A*(1+deltaA*sign(sin(K*(j-1)+W*T)));
a = A1/(-2*sin(K/2));


% 辅助数据存储变量
DataNum = 1;
DataT = zeros(1,10000);
Datas = zeros(6,10000);
SnakePostion = zeros(2,10000);

x1=-0.629;y1=-1.748;
x2=-0.812;y2=-1.991;
x3=-1.486;y3=-1.988;
x4=-1.713;y4=-1.913;
x5=-1.737;y5=-1.405;



%% 控制主体
% 连接成功
if (clientID>-1)
    disp('Connected to remote API server');
    % 获取关节句柄
    for i = 1 : JointNum
        JointName = ['Joint',num2str(i)];
        eval(['[~,Handle',JointName,'] = vrep.simxGetObjectHandle(clientID,JointName,vrep.simx_opmode_oneshot_wait);'])
    end
    
    % 获取关节基坐标句柄
    for i = 1 : (JointNum/2+1)
        DummyName = ['Dummy',num2str(i)];
        eval(['[~,Handle',DummyName,'] = vrep.simxGetObjectHandle(clientID,DummyName,vrep.simx_opmode_oneshot_wait);'])
    end
    
    % 获取连杆句柄
    [~,HandleLink0] = vrep.simxGetObjectHandle(clientID,'Snake',vrep.simx_opmode_oneshot_wait);
    for i = 1 : (JointNum-1)
        LinkName = ['Link',num2str(i)];
        eval(['[~,Handle',LinkName,'] = vrep.simxGetObjectHandle(clientID,LinkName,vrep.simx_opmode_oneshot_wait);'])
    end
 
    
    % 获取相机句柄
    [~,HandleVs] = vrep.simxGetObjectHandle(clientID,'Vs',vrep.simx_opmode_oneshot_wait);

    vrep.simxSynchronous(clientID,1); 
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    vrep.simxAddStatusbarMessage(clientID,'deltaA = deg2rad(0)',vrep.simx_opmode_oneshot);


    % Vrep保持运行状态
    while(vrep.simxGetConnectionId(clientID) ~= -1)
 
        % 获取实际关节角
        RealJoints = zeros(1,JointNum); % 关节角
        for m = 1:JointNum
            [~,RealJoints(m)] = vrep.simxGetJointPosition(clientID, eval((['HandleJoint',num2str(m)])), vrep.simx_opmode_oneshot);
        end
        % 平均关节角，注意由于俯仰关节的存在，所以需要乘以2
        AveRealJoints = mean(RealJoints)*2;
        
        % 获取实际连杆角，计算方向角
        Dir = 0;
        for m = 1:(JointNum/2+1)
            [~,DummyDirection] = vrep.simxGetObjectOrientation(clientID, eval((['HandleDummy',num2str(m)])), -1, vrep.simx_opmode_oneshot);
            Dir = Dir + DummyDirection(3);
        end
        Dir = Dir/(JointNum/2+1);
        
        % 获取质心位置
        LinkPosition = zeros(3,JointNum);
        for m = 1:JointNum
            [~,LinkPosition(:,m)] = vrep.simxGetObjectPosition(clientID,eval((['HandleLink',num2str(m-1)])),-1,vrep.simx_opmode_oneshot);
        end
        SnakePostion(1,DataNum) = mean(LinkPosition(1,:));
        SnakePostion(2,DataNum) = mean(LinkPosition(2,:));
 
         % 测试转弯效果
        if SnakePostion(1,DataNum) > x1 && SnakePostion(2,DataNum) > y1
            deltaA1 = deg2rad(-50);
        elseif SnakePostion(1,DataNum) > x2 && SnakePostion(2,DataNum) > y2 && SnakePostion(1,DataNum) < x1 && SnakePostion(2,DataNum) < y1
            deltaA1 = deg2rad(-100);
        elseif SnakePostion(1,DataNum) < x3 && SnakePostion(2,DataNum) < y3 && SnakePostion(1,DataNum) > x4 && SnakePostion(2,DataNum) > y4
            deltaA1 = deg2rad(-100);
        elseif SnakePostion(1,DataNum) > -1.648 && SnakePostion(2,DataNum) < y5 && SnakePostion(2,DataNum) > y4
            deltaA1 = deg2rad(-110);
        elseif SnakePostion(2,DataNum) > y5
            deltaA1 = deg2rad(120);
        end


        % 计算蜿蜒步态此刻各关节角度值
        Joints = zeros(1,JointNum);
        Joints(2)=A*(1+deltaA1*sign(sin(K+W*T)))*(1-exp(-T))*sin(K+W*T)+g*a*cos(W*T+h*K);
        for j = 4:2:JointNum
            Joints(j) = A*(1+deltaA*sign(sin(K*(j-1)+W*T)))*(1-exp(-T))*sin(K*(j-1)+W*T);
        end

        % 发送给机器人期望角度
        vrep.simxPauseCommunication(clientID,1);
        for m = 1 : JointNum
            vrep.simxSetJointTargetPosition(clientID, eval(['HandleJoint',num2str(m)]),Joints(m), vrep.simx_opmode_oneshot);
        end
        vrep.simxPauseCommunication(clientID,0);

        vrep.simxSynchronousTrigger(clientID); 
        vrep.simxGetPingTime(clientID);         
           
        if T>=75&&T<75.05
            vrep.simxAddStatusbarMessage(clientID,'turn1',vrep.simx_opmode_oneshot);
        end
        T = T + TimeStep;
    end
    DataNum = DataNum - 1;
    


    % 停止仿真并断开连接
    disp('仿真已停止，正在断开连接...');
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    vrep.simxGetPingTime(clientID);
    vrep.simxFinish(clientID);
else
    fprintf('Warning: Failed connecting to remote API server!\n');
end

%% 程序结束
vrep.delete();
disp('Program end');
