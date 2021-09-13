%% ����vrep
clear; clc;
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',3000,true,true,5000,5);

%% ��ȡCPG����������ɹؽڽǶ�
bool_flag = 0;
while (bool_flag == 0)
    gait_flag = input('***��ʾ*** �������Լ��㲽̬�˶���3/4/5 �㣿����');
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

%% ��˹켣������
global gait_cir3;
global gait_cir4;
global gait_cir5;

gait_cir3 = [1 0 1 0 1 0; 0 1 0 1 0 1];
gait_cir4 = [1 0 0 1 0 0; 0 0 1 0 0 1; 0 1 0 0 1 0];
gait_cir5 = [1 0 0 0 0 0; 0 0 0 0 0 1; 0 1 0 0 0 0; 0 0 0 0 1 0; 0 0 1 0 0 0; 0 0 0 1 0 0];


%% ��������д��ؽڽǶ�
pause_time = 0.01;% ͣ��ʱ������λ���룩
% �ؽڳ�ʼ�ǶȲ���
joint_offset(1) = 0;
joint_offset(2) = -pi/6;
joint_offset(3) = 2*pi/3;

if clientID>-1
    disp('Connected to remote API server��');
    % get handle for joints ����ȡ�ؽھ����
    for j = 0:5
        for i = 1:3
            hexa_jointI_J = ['hexa_joint',num2str(i),'_',num2str(j)];           
            % eval�����Ĺ����ǽ��ַ���ת��Ϊmatlab��ִ�����
            % handle_hexa_jointI_J = vrep.simxGetObjectHandle(clientID,hexa_jointI_J,vrep.simx_opmode_oneshot_wait);
            eval(['[~,handle_',hexa_jointI_J,'] = vrep.simxGetObjectHandle(clientID,hexa_jointI_J,vrep.simx_opmode_oneshot_wait);'])
        end
    end
    % Set the position of every joint �����ùؽڽ�λ�ơ�
    while vrep.simxGetConnectionId(clientID) ~= -1  % while v-rep connection is still active      
        % Moving! ���Խ��ɲ�̬�˶���
        for m = 1 : length(Output_jointJ_legI(:,1,1))
            vrep.simxPauseCommunication(clientID,1);
            for j = 1:6 
                for joint_i = 1:3
                    % �Ƕȵ�λ������rad
                    vrep.simxSetJointTargetPosition(clientID, eval(['handle_hexa_joint',num2str(joint_i),'_',num2str(j-1)]),...
                        Output_jointJ_legI(m,joint_i,j)+joint_offset(joint_i), vrep.simx_opmode_oneshot);
                end
            end
            vrep.simxPauseCommunication(clientID,0);
            pause(pause_time);
        end

        break;
    end
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











