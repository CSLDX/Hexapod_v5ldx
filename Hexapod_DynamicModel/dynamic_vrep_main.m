%% main_function ������ѧ�����ؿ��ƻ����ˡ�
%**************************************************************************
%*****************��Authors��AndyGao/LDX��Date��2021/07��**********************
%**************************************************************************
clear all;  clc;
disp('Program started');
%% ���عؽڽǶ�
% load('Torque.mat');
load('Theta_d.mat');
t = 1:length(theta_d);
%% ��������ṹ���� ���ȵ�λ��m ������λ��kg �Ƕȵ�λ��rad
% LL1 = 0.0514982;
% LL2 = 0.0723751;
% LL3 = 0.1165047;
% ML1 = 0.05;
% ML2 = 0.05;
% ML3 = 0.05;
% theta_c1 = pi/180*0;
% theta_c2 = -pi/180*30;
% theta_c3 = pi/180*120;
%% �������
power = 16.32; % ������ʣ���λ��W   P = FV  (N/m)(deg/s) 
max_velocity = 9999; % Ŀ���ٶ�
max_torque = 10; % ���Ť��
%% ����������������
theta_c = zeros(3,length(theta_d));
last_theta_c = [0;0;0];
theta_wc = zeros(3,length(theta_d));
last_theta_wc = [0;0;0];
theta_ac = zeros(3,length(theta_d));

e = zeros(3,length(theta_d));
last_e = [0,0,0];
last_last_e = [0,0,0];
de = [0,0,0];
u = zeros(3,length(theta_d)+1);
du = zeros(3,length(theta_d));
torque = zeros(3,length(theta_d));

% λ����pid
Kp = [0.6,5,1];
Kd = [5,12,2];

% ����Ĳ���
% Kp = [8,20,20];
% Kd = [4,6,5];
% Kp = [5,40,40];
% Kd = [80,800,800];

% ������pid
% Kp = [0.04,0.04,0.04];
% Ki = [0.5,0.5,0.5];
% Kd = [0.00,0.00,0.00];

%% ******************* ��MATLAB��V-rep�������ӡ� *********************
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
% ͣ��ʱ������λ���룩
pause_time = 0.01;
dt = 0.05;
%% ********************** �����ؿ��ƻ������Ȳ�֧���˶��� ********************
if (clientID>-1)
    disp('Connected to remote API server��');
    %**********************************************************************
    % get handle for joints ����ȡ�ؽھ����
    for j = 1 : 6
        for i = 1 : 3
            hexa_jointI_J = ['hexa_joint',num2str(i),'_',num2str(j-1)];
            %eval�����Ĺ����ǽ��ַ���ת��Ϊmatlab��ִ�����
            eval(['[~,handle_',hexa_jointI_J,'] = vrep.simxGetObjectHandle(clientID,hexa_jointI_J,vrep.simx_opmode_oneshot_wait);'])
        end
    end
    %**********************************************************************
    while(vrep.simxGetConnectionId(clientID) ~= -1)  % while v-rep connection is still active
        %******************************************************************
        vrep.simxSynchronous(clientID,1); % ʹ��ͬ��
        %******************************************************************
        % �ؽڻ�ȡ�Ƕȳ�ʼ��
        for i = 1:3
            [~,theta_c(i,1)] = vrep.simxGetJointPosition(clientID, eval(['handle_hexa_joint',num2str(i),'_',num2str(0)]),vrep.simx_opmode_streaming);
        end    
        vrep.simxSynchronousTrigger(clientID);
        vrep.simxGetPingTime(clientID);
        % ����ʼ��
        for i = 1:3
            [~,theta_c(i,1)] = vrep.simxGetJointPosition(clientID, eval(['handle_hexa_joint',num2str(i),'_',num2str(0)]),vrep.simx_opmode_buffer);
            e(i,1) = theta_d(i,1) - theta_c(i,1);
            last_theta_c(i) = theta_c(i,1);
            last_e(i) = e(i,1);
        end
        vrep.simxSynchronousTrigger(clientID);
        vrep.simxGetPingTime(clientID);
        % ����ѭ�������ؿ���
        for tt = 1:length(theta_d)          
            % ��ȡ��ǰ�ؽڽǶ�ֵ
            for i = 1:3
                % �����ĽǶȵ�λ��rad
                [~,theta_c(i,tt)] = vrep.simxGetJointPosition(clientID, eval(['handle_hexa_joint',num2str(i),'_',num2str(0)]),vrep.simx_opmode_buffer);              
                e(i,tt) = theta_d(i,tt) - theta_c(i,tt);
                de(i) = e(i,tt) - last_e(i);
                u(i,tt) = Kp(i)*e(i,tt) + Kd(i)*de(i);
                last_e(i) = e(i,tt);
                
                theta_wc(i,tt) = (theta_c(i,tt) - last_theta_c(i))/dt;
                theta_ac(i,tt) = (theta_wc(i,tt) - last_theta_wc(i))/dt;
                
            end            
            % ��������޷�   
            for j = 1:3 
                if abs(u(j,tt)) > max_torque
                    torque(j,tt) = sign(u(j,tt))*max_torque;
                else
                    torque(j,tt) = u(j,tt);
                end
            end
            % ������������ؽ�
            for i = 1 : 3
                vrep.simxSetJointTargetVelocity(clientID, eval(['handle_hexa_joint',num2str(i),'_',num2str(0)]),sign(torque(i,tt))*max_velocity,vrep.simx_opmode_oneshot);
                vrep.simxSetJointForce(clientID, eval(['handle_hexa_joint',num2str(i),'_',num2str(0)]),abs(torque(i,tt)), vrep.simx_opmode_oneshot);               
            end
            vrep.simxSynchronousTrigger(clientID);
            vrep.simxGetPingTime(clientID);
%             pause(pause_time);            
            
            % ��֤����˹켣���ɵĹؽڽǶ�
%             vrep.simxPauseCommunication(clientID,1);
%             for i = 1 : 3
%                 vrep.simxSetJointTargetPosition (clientID, eval(['handle_hexa_joint',num2str(i),'_',num2str(0)]),theta_d(i,tt),vrep.simx_opmode_oneshot);                   
%             end
%             vrep.simxPauseCommunication(clientID,0);

        end
        % ��ֹ�ؽ��˶�����������ס���
        for i = 1 : 3
            vrep.simxSetJointTargetVelocity(clientID, eval(['handle_hexa_joint',num2str(i),'_',num2str(0)]),0,vrep.simx_opmode_oneshot);
            vrep.simxSetJointForce(clientID, eval(['handle_hexa_joint',num2str(i),'_',num2str(0)]),1, vrep.simx_opmode_oneshot);               
        end 
        vrep.simxSynchronousTrigger(clientID);
        vrep.simxGetPingTime(clientID);
        break;
    end
    
%% ��ͼ
    figure(11);
%     plot(t,theta_d(1,:),t,theta_c(1,:),t,torque(1,:));
    plot(t,theta_d(1,:),t,theta_c(1,:));
    xlabel('t');
    ylabel('�Ƕ�/rad');
%     legend('Theta1�����Ƕ�','Theta1ʵ�ʽǶ�','�������1');
    legend('Theta1�����Ƕ�','Theta1ʵ�ʽǶ�');
    set(gca,'XTick',1:200:length(t));
    figure(22);
%     plot(t,theta_d(2,:),t,theta_c(2,:),t,torque(2,:));
    plot(t,theta_d(2,:),t,theta_c(2,:));
    xlabel('t');
    ylabel('�Ƕ�/rad');
%     legend('Theta2�����Ƕ�','Theta2ʵ�ʽǶ�','�������2');
    legend('Theta2�����Ƕ�','Theta2ʵ�ʽǶ�');
    set(gca,'XTick',1:200:length(t));
    figure(33);
%     plot(t,theta_d(3,:),t,theta_c(3,:),t,torque(3,:));
    plot(t,theta_d(3,:),t,theta_c(3,:));
    xlabel('t');
    ylabel('�Ƕ�/rad');
%     legend('Theta3�����Ƕ�','Theta3ʵ�ʽǶ�','�������3');
    legend('Theta3�����Ƕ�','Theta3ʵ�ʽǶ�');
    set(gca,'XTick',1:200:length(t));
    save('train_torque.mat','torque');
    save('train_theta_c.mat','theta_c');
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