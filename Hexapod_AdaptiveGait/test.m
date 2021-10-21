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
%% 记录实验数据

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
    
    % 相关参数变量初始化      
    N0 = 10; % 调整机身位姿步数   
    N = 100; % 步数
    stepRotate = [0 0 0 0 0 0];   % 迈步方向 
    stepHeight = 0.03*ones(1,6);  % 抬腿高度
    stepAmplitude = [0 0 0 0 0 0];% 迈步步幅
    num = 0;  % 保存数据的计数值
    flag = 1; % 行走路径点的标志位
    imageAcquisitionTime = 0;       % vrep图像获取的时间戳
    last_imageAcquisitionTime = 0;  % 上个vrep图像获取的时间戳，用于计算帧率
    lastX = body_position(1);  % 前一时刻机身x坐标
    lastY = body_position(2);  % 前一时刻机身y坐标
    lastBody_rotate = body_angles(3); % 前一时刻机身绕z轴旋转角度    
    LegPos_offset = zeros(3,6);
    legpos_offset = zeros(3,6);
    last_LegPos_offset = zeros(3,6);
    Body2Body_T = eye(4);
    body_height = 0;
    body_rotatex = 0; % 期望的机身翻滚
    body_rotatey = 0; % 期望的机身俯仰
    footTip_pos = zeros(6,4);
    Is_adaptive = 1; % 开启和关闭自适应功能
    %**********************************************************************
    while(vrep.simxGetConnectionId(clientID) ~= -1)  % while v-rep connection is still active       
%% 六足机器人机身转动测试
%         for ishow = 1:5
%             for j = 1:N0   
%                 switch ishow
%                     case {1}
%                         Body2Body_offset = [0,0,-0.003*N0];
%                         Body2Body_rot = [0,-0.5*N0,0];  % [翻滚 俯仰 偏航]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % 输入角度单位为弧度
%                     case {2}
%                         Body2Body_offset = [0,0,0];
%                         Body2Body_rot = [0,0.5*N0,0];  % [翻滚 俯仰 偏航]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % 输入角度单位为弧度
%                     case {3}
%                         Body2Body_offset = [0,0,0];
%                         Body2Body_rot = [0,0,0.8*N0];  % [翻滚 俯仰 偏航]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % 输入角度单位为弧度
%                     case {4}
%                         Body2Body_offset = [0,0,0];
%                         Body2Body_rot = [0,0,-0.8*N0];  % [翻滚 俯仰 偏航]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % 输入角度单位为弧度
%                     case {5}
%                         Body2Body_offset = [0,0,-0.002*N0];
%                         Body2Body_rot = [0,0,0];  % [翻滚 俯仰 偏航]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % 输入角度单位为弧度
%                 end      
%                 for i = 1 : 6
%                     vrep.simxSetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,Body2Body_T*Leg2Body_T{i}*[0;0;0;1],vrep.simx_opmode_oneshot);              
%                 end
%                 % 同步
%                 vrep.simxSynchronousTrigger(clientID);
%                 vrep.simxGetPingTime(clientID);            
%             end
%         end
%% 六足机器人机身初始化调整          
        for j = 1:N0        
            % 期望机身坐标系相对当前机身坐标系的偏移和旋转
            Body2Body_offset = [0,0,-0.003*N0];
            Body2Body_rot = [0,0,0];  % [翻滚 俯仰 偏航]
            Body2Body_T = MT.Body2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % 输入角度单位为弧度
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
        body_height = body_position(3); % 机身的离地高度
        % 同步
        vrep.simxSynchronousTrigger(clientID);
        vrep.simxGetPingTime(clientID);   
        
%% 六足机器人步行N步
        for j = 1:N
%             [gait_num, leg_pos] = MT.Three_leg_gait(stepHeight,stepAmplitude,stepRotate);
%             [gait_num, leg_pos] = MT.Output_track(0.03,0.03,60,3);
%             [stepHeight,stepAmplitude,stepRotate,error_dis,error_rot] = Body_walk_control(dX,dY,dBody_rotate,rX,rY,rBody_rotate,lastX,lastY,lastBody_rotate);
%             [gait_num, leg_pos] = walk_path(stepHeight,stepAmplitude,stepRotate,j);
            [~,body_angles] = vrep.simxGetObjectOrientation(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
            [~,body_position] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
            % 同步
%             vrep.simxSynchronousTrigger(clientID);
%             vrep.simxGetPingTime(clientID);      
%             [stepHeight,stepAmplitude,stepRotate,error_dis,eror_rot] = C.Body_walk_control(body_position(1)+0.1, body_position(2), 0,...
%                                                                         body_position(1),body_position(2),body_angles(3),...
%                                                                         lastX,lastY,lastBody_rotate,0.04,0.03);
%             [gait_num, leg_pos] = MT.Three_leg_gait(stepHeight,stepAmplitude,stepRotate);
            [gait_num, leg_pos, flag] = walk_path(body_position(1),body_position(2),body_angles(3),lastX,lastY,lastBody_rotate,flag);       
            % 遍历设计的的足端轨迹并发送给机器人           
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
                    
                    Turn_z = [0,0,-body_angles(3)]; % 对于绝对坐标的读数进行转换，x轴方向与bodybase方向一致
                    Turn_AbsoluteOrientation = MT.Trans_Matirx([0 0 0],Turn_z);
                    body_angles_impro = Turn_AbsoluteOrientation*[body_angles';1];
                    body_rotate_impro = Turn_AbsoluteOrientation*[body_rotatex;body_rotatey;0;1];
                    
                    Body2Body_offset = [0,0,body_position(3)-body_height];  % 期望的机身高度body_height
                    Body2Body_rot = [body_angles_impro(1)-body_rotate_impro(1),body_angles_impro(2)-body_rotate_impro(2),0];  % 期望的机身翻滚角度Alpha、俯仰角度Beta 
                    Body2Body_T = MT.Body2Body_trans(Body2Body_offset,Body2Body_rot);  
                end

                % 输出足端轨迹
                for i = 1 : 6
                    footTip_pos(i,:) = (Body2Body_T*Leg2Body_T{i}*[leg_pos{i,1}(1,tt);leg_pos{i,1}(2,tt);leg_pos{i,1}(3,tt);1])';
                    vrep.simxSetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,Body2Body_T*Leg2Body_T{i}*[leg_pos{i,1}(1,tt);leg_pos{i,1}(2,tt);leg_pos{i,1}(3,tt);1],vrep.simx_opmode_oneshot);              
                    % 如开启自适应，更新补偿值                 
                    if Is_adaptive
                        if legpos_offset(3,i) ~= 0
                            LegPos_offset(:,i) = legpos_offset(:,i); % 更新对应足端轨迹的补偿值
                        end 
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
                body_Angles_impro(num+1,:) = rad2deg(body_angles_impro);
%                 body_Position(num+1,:) = body_position;
%                 body2body_M{num+1} = Body2Body_T;

                lastX = body_position(1);
                lastY = body_position(2);
                lastBody_rotate = body_angles(3);

                num = num + 1;
                % 同步
                vrep.simxSynchronousTrigger(clientID);
                vrep.simxGetPingTime(clientID);       
            end
            
%             if Is_adaptive
%                 for i = 1 : 6                             
%                     Leg2Body_T{i} = Body2Body_T*Leg2Body_T{i}; % 更新足端坐标相对于机身坐标系的变换矩阵，一个步态周期更新一次 
%                 end 
%             end

            % 改变机身期望方向,以各足端相对于机身的位置作为计算值,进行平面计算姿态
            k_xz1 = (footTip_pos(1,3) - footTip_pos(3,3))/(footTip_pos(1,1) - footTip_pos(3,1));
            k_xz2 = (footTip_pos(6,3) - footTip_pos(4,3))/(footTip_pos(6,1) - footTip_pos(4,1));
            Beta = atan(mean([k_xz1 k_xz2])); % yz平面
            k_yz1 = (footTip_pos(1,3) - footTip_pos(6,3))/(footTip_pos(1,2) - footTip_pos(6,2));
            k_yz2 = (footTip_pos(2,3) - footTip_pos(5,3))/(footTip_pos(2,2) - footTip_pos(5,2));
            k_yz3 = (footTip_pos(3,3) - footTip_pos(4,3))/(footTip_pos(3,2) - footTip_pos(4,2));
            Alpha = atan(mean([k_yz1 k_yz2 k_yz3])); % xz平面
            
            body_rotatex = -Alpha;
            body_rotatey = -Beta;

            Body2Body_T = eye(4); % 重置矩阵   
            last_LegPos_offset = LegPos_offset; % 记录上一次足端轨迹的补偿值
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