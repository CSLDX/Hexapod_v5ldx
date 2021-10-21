%% main_function ��VK �����˱�����
%**************************************************************************
%*****************��Authors��AndyGao/LDX��Date��2021/07��**********************
%**************************************************************************
clear all;  clc;
disp('Program started');
MT = MakeTrack;
C = Control;
%% ������˹켣
% load('Leg_Pos5.mat');
% gait_num = length(leg_pos{1});
%% ��¼ʵ������

%% ******************* ��MATLAB��V-rep�������ӡ� *********************
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
% ͣ��ʱ������λ���룩
pause_time = 0.05;
dt = 0.05;
%% ********************** �������˱���Vk_hexa���ԡ� ********************
if (clientID>-1)
    disp('Connected to remote API server��');
    %**********************************************************************
    % get handle ����ȡ�ؽھ����
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
    vrep.simxSynchronous(clientID,1); % ʹ��ͬ��
    %******************************************************************
    % ͼ���ȡ��ʼ��   fx = 554.2563 
    [~,~,~] = vrep.simxGetVisionSensorImage2(clientID, handle_cam_rgb, 0, vrep.simx_opmode_streaming);
    [~,~,~] = vrep.simxGetVisionSensorDepthBuffer2(clientID, handle_cam_depth, vrep.simx_opmode_streaming);
    % ������λ�ó�ʼ��,��ײ����ʼ��
    for i = 1:6
        [~,~] = vrep.simxGetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,vrep.simx_opmode_streaming);
        [~,~] = vrep.simxGetObjectOrientation(clientID, eval(['handle_footTarget',num2str(i)]), handle_bodyBase, vrep.simx_opmode_streaming);
        [~,~] = vrep.simxReadCollision(clientID, eval(['handle_Collision',num2str(i)]), vrep.simx_opmode_streaming);
    end    
    % ����λ�˳�ʼ��
    [~,~] = vrep.simxGetObjectOrientation(clientID, handle_bodyBase, -1, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_streaming);
    
    vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);
    
    % ������λ�ö�ȡ,��ײ���
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
    % ͼ�����ݶ�ȡ
    [~,size_hexa_rgb, hexa_rgb_image] = vrep.simxGetVisionSensorImage2(clientID, handle_cam_rgb, 0, vrep.simx_opmode_buffer);
    [~,size_hexa_depth, hexa_depth_image] = vrep.simxGetVisionSensorDepthBuffer2(clientID, handle_cam_depth, vrep.simx_opmode_buffer);  
    % �����ʼλ�˶�ȡ alpha, beta, gamma
    [~,body_angles] = vrep.simxGetObjectOrientation(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
    [~,body_position] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
    
    vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);
    
    % ��ز���������ʼ��      
    N0 = 10; % ��������λ�˲���   
    N = 100; % ����
    stepRotate = [0 0 0 0 0 0];   % �������� 
    stepHeight = 0.03*ones(1,6);  % ̧�ȸ߶�
    stepAmplitude = [0 0 0 0 0 0];% ��������
    num = 0;  % �������ݵļ���ֵ
    flag = 1; % ����·����ı�־λ
    imageAcquisitionTime = 0;       % vrepͼ���ȡ��ʱ���
    last_imageAcquisitionTime = 0;  % �ϸ�vrepͼ���ȡ��ʱ��������ڼ���֡��
    lastX = body_position(1);  % ǰһʱ�̻���x����
    lastY = body_position(2);  % ǰһʱ�̻���y����
    lastBody_rotate = body_angles(3); % ǰһʱ�̻�����z����ת�Ƕ�    
    LegPos_offset = zeros(3,6);
    legpos_offset = zeros(3,6);
    last_LegPos_offset = zeros(3,6);
    Body2Body_T = eye(4);
    body_height = 0;
    body_rotatex = 0; % �����Ļ�����
    body_rotatey = 0; % �����Ļ�����
    footTip_pos = zeros(6,4);
    Is_adaptive = 1; % �����͹ر�����Ӧ����
    %**********************************************************************
    while(vrep.simxGetConnectionId(clientID) ~= -1)  % while v-rep connection is still active       
%% ��������˻���ת������
%         for ishow = 1:5
%             for j = 1:N0   
%                 switch ishow
%                     case {1}
%                         Body2Body_offset = [0,0,-0.003*N0];
%                         Body2Body_rot = [0,-0.5*N0,0];  % [���� ���� ƫ��]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % ����Ƕȵ�λΪ����
%                     case {2}
%                         Body2Body_offset = [0,0,0];
%                         Body2Body_rot = [0,0.5*N0,0];  % [���� ���� ƫ��]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % ����Ƕȵ�λΪ����
%                     case {3}
%                         Body2Body_offset = [0,0,0];
%                         Body2Body_rot = [0,0,0.8*N0];  % [���� ���� ƫ��]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % ����Ƕȵ�λΪ����
%                     case {4}
%                         Body2Body_offset = [0,0,0];
%                         Body2Body_rot = [0,0,-0.8*N0];  % [���� ���� ƫ��]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % ����Ƕȵ�λΪ����
%                     case {5}
%                         Body2Body_offset = [0,0,-0.002*N0];
%                         Body2Body_rot = [0,0,0];  % [���� ���� ƫ��]
%                         Body2Body_T = MT.Leg2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % ����Ƕȵ�λΪ����
%                 end      
%                 for i = 1 : 6
%                     vrep.simxSetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,Body2Body_T*Leg2Body_T{i}*[0;0;0;1],vrep.simx_opmode_oneshot);              
%                 end
%                 % ͬ��
%                 vrep.simxSynchronousTrigger(clientID);
%                 vrep.simxGetPingTime(clientID);            
%             end
%         end
%% ��������˻����ʼ������          
        for j = 1:N0        
            % ������������ϵ��Ե�ǰ��������ϵ��ƫ�ƺ���ת
            Body2Body_offset = [0,0,-0.003*N0];
            Body2Body_rot = [0,0,0];  % [���� ���� ƫ��]
            Body2Body_T = MT.Body2Body_trans(Body2Body_offset,deg2rad(Body2Body_rot)); % ����Ƕȵ�λΪ����
            for i = 1 : 6
                vrep.simxSetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,Body2Body_T*Leg2Body_T{i}*[0;0;0;1],vrep.simx_opmode_oneshot);              
            end
            % ͬ��
            vrep.simxSynchronousTrigger(clientID);
            vrep.simxGetPingTime(clientID);            
        end
        % ��ȡ������λ����Ϣ
        for i = 1:6
            [~, Leg2Body_offset(i,:)] = vrep.simxGetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,vrep.simx_opmode_buffer);
            [~, Leg2Body_rot(i,:)] = vrep.simxGetObjectOrientation(clientID, eval(['handle_footTarget',num2str(i)]), handle_bodyBase, vrep.simx_opmode_buffer);
            Leg2Body_T{i} = MT.Leg2Body_trans(Leg2Body_offset(i,:),Leg2Body_rot(i,:));
        end
        [~,body_position] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
        body_height = body_position(3); % �������ظ߶�
        % ͬ��
        vrep.simxSynchronousTrigger(clientID);
        vrep.simxGetPingTime(clientID);   
        
%% ��������˲���N��
        for j = 1:N
%             [gait_num, leg_pos] = MT.Three_leg_gait(stepHeight,stepAmplitude,stepRotate);
%             [gait_num, leg_pos] = MT.Output_track(0.03,0.03,60,3);
%             [stepHeight,stepAmplitude,stepRotate,error_dis,error_rot] = Body_walk_control(dX,dY,dBody_rotate,rX,rY,rBody_rotate,lastX,lastY,lastBody_rotate);
%             [gait_num, leg_pos] = walk_path(stepHeight,stepAmplitude,stepRotate,j);
            [~,body_angles] = vrep.simxGetObjectOrientation(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
            [~,body_position] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
            % ͬ��
%             vrep.simxSynchronousTrigger(clientID);
%             vrep.simxGetPingTime(clientID);      
%             [stepHeight,stepAmplitude,stepRotate,error_dis,eror_rot] = C.Body_walk_control(body_position(1)+0.1, body_position(2), 0,...
%                                                                         body_position(1),body_position(2),body_angles(3),...
%                                                                         lastX,lastY,lastBody_rotate,0.04,0.03);
%             [gait_num, leg_pos] = MT.Three_leg_gait(stepHeight,stepAmplitude,stepRotate);
            [gait_num, leg_pos, flag] = walk_path(body_position(1),body_position(2),body_angles(3),lastX,lastY,lastBody_rotate,flag);       
            % ������Ƶĵ���˹켣�����͸�������           
            for tt = 1:gait_num
                for i = 1 : 6
                    % ��ײ���
                    [~,Is_collision(i)] = vrep.simxReadCollision(clientID, eval(['handle_Collision',num2str(i)]), vrep.simx_opmode_buffer);
                end
                % ��̬���
                [~,body_angles] = vrep.simxGetObjectOrientation(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
                [~,body_position] = vrep.simxGetObjectPosition(clientID, handle_bodyBase, -1, vrep.simx_opmode_buffer);
                vrep.simxSynchronousTrigger(clientID);
                vrep.simxGetPingTime(clientID);  
               
                % ����Ӧ��˹켣
                if Is_adaptive
                    [~, leg_pos, legpos_offset] = MT.Adaptive_gait(gait_num, leg_pos, Is_collision, tt, last_LegPos_offset);
                    % ������̬��ת������
                    % ������������ϵ��Ե�ǰ��������ϵ��ƫ�ƺ���ת
                    
                    Turn_z = [0,0,-body_angles(3)]; % ���ھ�������Ķ�������ת����x�᷽����bodybase����һ��
                    Turn_AbsoluteOrientation = MT.Trans_Matirx([0 0 0],Turn_z);
                    body_angles_impro = Turn_AbsoluteOrientation*[body_angles';1];
                    body_rotate_impro = Turn_AbsoluteOrientation*[body_rotatex;body_rotatey;0;1];
                    
                    Body2Body_offset = [0,0,body_position(3)-body_height];  % �����Ļ���߶�body_height
                    Body2Body_rot = [body_angles_impro(1)-body_rotate_impro(1),body_angles_impro(2)-body_rotate_impro(2),0];  % �����Ļ������Ƕ�Alpha�������Ƕ�Beta 
                    Body2Body_T = MT.Body2Body_trans(Body2Body_offset,Body2Body_rot);  
                end

                % �����˹켣
                for i = 1 : 6
                    footTip_pos(i,:) = (Body2Body_T*Leg2Body_T{i}*[leg_pos{i,1}(1,tt);leg_pos{i,1}(2,tt);leg_pos{i,1}(3,tt);1])';
                    vrep.simxSetObjectPosition(clientID, eval(['handle_footTarget',num2str(i)]),handle_bodyBase,Body2Body_T*Leg2Body_T{i}*[leg_pos{i,1}(1,tt);leg_pos{i,1}(2,tt);leg_pos{i,1}(3,tt);1],vrep.simx_opmode_oneshot);              
                    % �翪������Ӧ�����²���ֵ                 
                    if Is_adaptive
                        if legpos_offset(3,i) ~= 0
                            LegPos_offset(:,i) = legpos_offset(:,i); % ���¶�Ӧ��˹켣�Ĳ���ֵ
                        end 
                    end 
                end
                
                % ��ȡ�������
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
                % ͬ��
                vrep.simxSynchronousTrigger(clientID);
                vrep.simxGetPingTime(clientID);       
            end
            
%             if Is_adaptive
%                 for i = 1 : 6                             
%                     Leg2Body_T{i} = Body2Body_T*Leg2Body_T{i}; % ���������������ڻ�������ϵ�ı任����һ����̬���ڸ���һ�� 
%                 end 
%             end

            % �ı������������,�Ը��������ڻ����λ����Ϊ����ֵ,����ƽ�������̬
            k_xz1 = (footTip_pos(1,3) - footTip_pos(3,3))/(footTip_pos(1,1) - footTip_pos(3,1));
            k_xz2 = (footTip_pos(6,3) - footTip_pos(4,3))/(footTip_pos(6,1) - footTip_pos(4,1));
            Beta = atan(mean([k_xz1 k_xz2])); % yzƽ��
            k_yz1 = (footTip_pos(1,3) - footTip_pos(6,3))/(footTip_pos(1,2) - footTip_pos(6,2));
            k_yz2 = (footTip_pos(2,3) - footTip_pos(5,3))/(footTip_pos(2,2) - footTip_pos(5,2));
            k_yz3 = (footTip_pos(3,3) - footTip_pos(4,3))/(footTip_pos(3,2) - footTip_pos(4,2));
            Alpha = atan(mean([k_yz1 k_yz2 k_yz3])); % xzƽ��
            
            body_rotatex = -Alpha;
            body_rotatey = -Beta;

            Body2Body_T = eye(4); % ���þ���   
            last_LegPos_offset = LegPos_offset; % ��¼��һ����˹켣�Ĳ���ֵ
            LegPos_offset = zeros(3,6);
        end
        break;
    end
    
%% ��ͼ
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


%% ����    
    % pause(); % �����������ִ�г���
    input('��ʾ�����밴���������ִ�г���'); 
    %**********************************************************************
    %*************************��ֹͣ���沢�Ͽ����ӡ�************************
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

%% �������
vrep.delete(); % call the destructor!
disp('! Program ended !');