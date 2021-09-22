%% main_function 【VK 机器人比赛】
%**************************************************************************
%*****************【Authors：AndyGao/LDX；Date：2021/07】**********************
%**************************************************************************
clear all;  clc;
disp('Program started');
MT = MakeTrack;
C = Control;
%% 加载足端轨迹
% load('Leg_Pos5.mat');
% gait_num = length(leg_pos{1});
%% 六足机器人结构参数 长度单位：m 质量单位：kg 角度单位：rad

%% 蛇形机器人 
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


%% ******************* 【MATLAB与V-rep建立连接】 *********************
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
% 停顿时长（单位：秒）
pause_time = 0.05;
dt = 0.05;
%% ********************** 【机器人比赛Vk_hexa测试】 ********************
if (clientID>-1)
    disp('Connected to remote API server！');
    %**********************************************************************
    % get handle 【获取关节句柄】
    %**********************************************************************
    [~,handle_footBase] = vrep.simxGetObjectHandle(clientID,'footBase',vrep.simx_opmode_oneshot_wait);
    for j = 1 : 6
        footTargetx = ['footTarget',num2str(j)];
        eval(['[~,handle_',footTargetx,'] = vrep.simxGetObjectHandle(clientID,footTargetx,vrep.simx_opmode_oneshot_wait);'])
        footCollisionx = ['Collision',num2str(j)];
        eval(['[~,handle_',footCollisionx,'] = vrep.simxGetCollisionHandle(clientID,footCollisionx,vrep.simx_opmode_blocking);'])
    end
    [~,handle_cam_rgb] = vrep.simxGetObjectHandle(clientID,'hexa_cam_rgb', vrep.simx_opmode_blocking);
    [~,handle_cam_depth] = vrep.simxGetObjectHandle(clientID,'hexa_cam_depth', vrep.simx_opmode_blocking);    
    [~,handle_bodyBase] = vrep.simxGetObjectHandle(clientID,'bodyBase',vrep.simx_opmode_oneshot_wait);
    
    % 蛇形机器人
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
    
    
    %******************************************************************
    vrep.simxSynchronous(clientID,1); % 使能同步
    %******************************************************************
    % 图像读取初始化   fx = 554.2563 
    [~,~,~] = vrep.simxGetVisionSensorImage2(clientID, handle_cam_rgb, 0, vrep.simx_opmode_streaming);
    [~,~,~] = vrep.simxGetVisionSensorDepthBuffer2(clientID, handle_cam_depth, vrep.simx_opmode_streaming);
    % 足端相对位置初始化,碰撞检测初始化
    for i = 1:6
        [~,~] = vrep.simxGetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,vrep.simx_opmode_streaming);
        [~,~] = vrep.simxGetObjectOrientation(clientID, eval(['handle_footTarget',num2str(i)]), handle_bodyBase, vrep.simx_opmode_streaming);
        [~,~] = vrep.simxReadCollision(clientID, eval(['handle_Collision',num2str(i)]), vrep.simx_opmode_streaming);
    end    
    % 机身位姿初始化
    [~,~] = vrep.simxGetObjectOrientation(clientID, handle_bodyBase, -1, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_streaming);
    
    vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);
    
    % 足端相对位置读取,碰撞检测
    Leg2Body_offset = zeros(6,3);
    Leg2Body_rot = zeros(6,3);
    Is_collision = zeros(6);
    Leg2Body_T = cell(6,1);
    for i = 1:6
        [~, Leg2Body_offset(i,:)] = vrep.simxGetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,vrep.simx_opmode_buffer);
        [~, Leg2Body_rot(i,:)] = vrep.simxGetObjectOrientation(clientID, eval(['handle_footTarget',num2str(i)]), handle_bodyBase, vrep.simx_opmode_buffer);
        [~,Is_collision(i)] = vrep.simxReadCollision(clientID, eval(['handle_Collision',num2str(i)]), vrep.simx_opmode_buffer);
        Leg2Body_T{i} = MT.Leg2Body_trans(Leg2Body_offset(i,:),Leg2Body_rot(i,:));
    end
    % 图像数据读取
    [~,size_hexa_rgb, hexa_rgb_image] = vrep.simxGetVisionSensorImage2(clientID, handle_cam_rgb, 0, vrep.simx_opmode_buffer);
    [~,size_hexa_depth, hexa_depth_image] = vrep.simxGetVisionSensorDepthBuffer2(clientID, handle_cam_depth, vrep.simx_opmode_buffer);  
    % 机身初始位姿读取 alpha, beta, gamma
    [~,body_angles] = vrep.simxGetObjectOrientation(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
    [~,body_position] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
    
    vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);
      
    % N0 调整机身位姿步数
    N0 = 10;
    % N 步数
    N = 110;
    % 初始化
    stepRotate = [0 0 0 0 0 0];
    stepHeight = 0.03*ones(1,6);
    stepAmplitude = [0 0 0 0 0 0];
    num = 0;
    flag = 1;
    imageAcquisitionTime = 0;
    last_imageAcquisitionTime = 0;  
    lastX = body_position(1);
    lastY = body_position(2);
    lastBody_rotate = body_angles(3);
    
    LegPos_offset = zeros(3,6);
    legpos_offset = zeros(3,6);
    last_LegPos_offset = zeros(3,6);
    Body2Body_T = eye(4);
    Is_adaptive = 0; % 开启和关闭自适应功能
    %% 主循环
    %**********************************************************************
    while(vrep.simxGetConnectionId(clientID) ~= -1)  % while v-rep connection is still active

        % 六足机器人机身调整N0  
        for j = 1:N0        
            % 期望机身坐标系相对当前机身坐标系的偏移和旋转
            Body2Body_offset = [0,0,0];
            Body2Body_rot = [0,0,0];  % [翻滚 俯仰 偏航]
            Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % 输入角度单位为弧度
            for i = 1 : 6
                vrep.simxSetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,Body2Body_T*Leg2Body_T{i}*[0;0;0;1],vrep.simx_opmode_oneshot);              
            end
            % 同步
            vrep.simxSynchronousTrigger(clientID);
            vrep.simxGetPingTime(clientID);            
        end
        % 读取调整后位姿信息
        for i = 1:6
            [~, Leg2Body_offset(i,:)] = vrep.simxGetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,vrep.simx_opmode_buffer);
            [~, Leg2Body_rot(i,:)] = vrep.simxGetObjectOrientation(clientID, eval(['handle_footTarget',num2str(i)]), handle_bodyBase, vrep.simx_opmode_buffer);
            Leg2Body_T{i} = MT.Leg2Body_trans(Leg2Body_offset(i,:),Leg2Body_rot(i,:));
        end
        [~,body_position] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
        body_height = body_position(3);
        % 同步
        vrep.simxSynchronousTrigger(clientID);
        vrep.simxGetPingTime(clientID);   
      
        % 六足机器人步行N步
        for j = 1:N
            [~,body_angles] = vrep.simxGetObjectOrientation(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
            [~,body_position] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
            % 同步
            vrep.simxSynchronousTrigger(clientID);
            vrep.simxGetPingTime(clientID);      
            [gait_num, leg_pos, flag] = walk_path(body_position(1),body_position(2),body_angles(3),lastX,lastY,lastBody_rotate,flag);       
           
            for tt = 1:gait_num
                for i = 1 : 6
                    % 碰撞检测
                    [~,Is_collision(i)] = vrep.simxReadCollision(clientID, eval(['handle_Collision',num2str(i)]), vrep.simx_opmode_buffer);
                end
                % 姿态检测
                [~,body_angles] = vrep.simxGetObjectOrientation(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
                [~,body_position] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
                vrep.simxSynchronousTrigger(clientID);
                vrep.simxGetPingTime(clientID);  
               
                % 自适应足端轨迹
                if Is_adaptive
                    [~, leg_pos, legpos_offset] = MT.Adaptive_gait(gait_num, leg_pos, Is_collision, tt, last_LegPos_offset);
                    % 身体姿态的转换矩阵
                    % 期望机身坐标系相对当前机身坐标系的偏移和旋转
                    Body2Body_offset = [0,0,body_position(3)-body_height];
                    Body2Body_rot = [0-body_angles(1),0-body_angles(2),0];
                    Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,Body2Body_rot);  
                end
                % 输出足端轨迹
                for i = 1 : 6
                    vrep.simxSetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,Body2Body_T*Leg2Body_T{i}*[leg_pos{i,1}(1,tt);leg_pos{i,1}(2,tt);leg_pos{i,1}(3,tt);1],vrep.simx_opmode_oneshot);              
                    % 更新补偿值                 
                    if legpos_offset(3,i) ~= 0 && Is_adaptive
                        LegPos_offset(:,i) = legpos_offset(:,i);          
                    end
                end

                % 获取相机数据
%                 [~,size_hexa_rgb, hexa_rgb_image] = vrep.simxGetVisionSensorImage2(clientID, handle_cam_rgb, 0, vrep.simx_opmode_buffer);
%                 [~,size_hexa_depth, hexa_depth_image] = vrep.simxGetVisionSensorDepthBuffer2(clientID, handle_cam_depth, vrep.simx_opmode_buffer); 
%                 imageAcquisitionTime=vrep.simxGetLastCmdTime(clientID);
%                 fps = 1/((imageAcquisitionTime - last_imageAcquisitionTime)/1000);
%                 last_imageAcquisitionTime = imageAcquisitionTime;
%                 imwrite(hexa_rgb_image, ['./img/rgb/rgb' num2str(num) '.png']);
%                 imwrite(hexa_depth_image, ['./img/depth/depth' num2str(num) '.png']);

                body_Angles(num+1,:) = rad2deg(body_angles);
                body_Position(num+1,:) = body_position;

                lastX = body_position(1);
                lastY = body_position(2);
                lastBody_rotate = body_angles(3);

                num = num + 1;
                
                % 蛇形机器人
                % 获取实际关节角
                RealJoints = zeros(1,JointNum); % 关节角
                for m = 1:JointNum
                    [~,RealJoints(m)] = vrep.simxGetJointPosition(clientID, eval((['HandleJoint',num2str(m)])), vrep.simx_opmode_oneshot);
                end
                % 平均关节角，注意由于俯仰关节的存在，所以需要乘以2
                AveRealJoints = mean(RealJoints)*2;
                
                % 获取质心位置
                LinkPosition = zeros(3,JointNum);
                for m = 1:JointNum
                    [~,LinkPosition(:,m)] = vrep.simxGetObjectPosition(clientID,eval((['HandleLink',num2str(m-1)])),-1,vrep.simx_opmode_oneshot);
                end
                SnakePostion(1,DataNum) = mean(LinkPosition(1,:));
                SnakePostion(2,DataNum) = mean(LinkPosition(2,:));

                % 获取实际连杆角，计算方向角
                Dir = 0;
                for m = 1:(JointNum/2+1)
                    [~,DummyDirection] = vrep.simxGetObjectOrientation(clientID, eval((['HandleDummy',num2str(m)])), -1, vrep.simx_opmode_oneshot);
                    Dir = Dir + DummyDirection(3);
                end
                Dir = Dir/(JointNum/2+1);

                % 测试转弯效果
                if SnakePostion(1,DataNum) > x1 && SnakePostion(2,DataNum) > y1
                    deltaA1 = deg2rad(-50);
                elseif SnakePostion(1,DataNum) > x2 && SnakePostion(2,DataNum) > y2 && SnakePostion(1,DataNum) < x1 && SnakePostion(2,DataNum) < y1
                    deltaA1 = deg2rad(-140);
                elseif SnakePostion(1,DataNum) < x3 && SnakePostion(2,DataNum) < y3 && SnakePostion(1,DataNum) > x4 && SnakePostion(2,DataNum) > y4
                    deltaA1 = deg2rad(-100);
                elseif SnakePostion(1,DataNum) > -1.648 && SnakePostion(2,DataNum) < y5 && SnakePostion(2,DataNum) > y4
                    deltaA1 = deg2rad(-110);
                elseif SnakePostion(2,DataNum) > y5
                    deltaA1 = deg2rad(120);
                end


                % 计算蜿蜒步态此刻各关节角度值
                Joints = zeros(1,JointNum);
                Joints(2)=A*(1+deltaA1*sign(sin(K+W*T)))*sin(K+W*T)+g*a*cos(W*T+h*K);
                for jj = 4:2:JointNum
                     Joints(jj) = A*(1+deltaA*sign(sin(K*(jj-1)+W*T)))*sin(K*(jj-1)+W*T);
                end

                % 发送给机器人期望角度
                for m = 1 : JointNum
                    vrep.simxSetJointTargetPosition(clientID, eval(['HandleJoint',num2str(m)]),Joints(m), vrep.simx_opmode_oneshot);
                end
    
                T = T + TimeStep;   
                DataNum = DataNum + 1;
                % 同步
                vrep.simxSynchronousTrigger(clientID);
                vrep.simxGetPingTime(clientID);       
            end

            last_LegPos_offset = LegPos_offset; 
            LegPos_offset = zeros(3,6);
        end
        break;
    end
    
%% 作图
%     count = 430;
%     figure(11);
%     plot(1:count,nbody_Angles(1:count,1),1:count,abody_Angles(1:count,1));
% %     xlabel('t');
%     ylabel('Alpha/deg');
%     legend('normal','adaptive');
%     figure(22);
%     plot(1:count,nbody_Angles(1:count,2),1:count,abody_Angles(1:count,2));
% %     xlabel('t');
%     ylabel('Beta/deg');
%     legend('normal','adaptive');
%     figure(33);
%     plot(1:count,nbody_Position(1:count,3),1:count,abody_Position(1:count,3));
% %     xlabel('t');
%     ylabel('H/m');
%     legend('normal','adaptive');


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