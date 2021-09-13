%% ���������Ѳ�̬�ķ�ֵ������ת�������о����롿
% ���ؽڽǶ����ݷ��͸�������
close all;  clear;  clc;
disp('Program start');

%% MATLAB��V-rep��������
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%% ��̬������ʼ��
disp('ת�������о�')
A = deg2rad(30);
K = deg2rad(60);
W = deg2rad(90);
h = 4.70;
g = 0.98;
deltaA = deg2rad(0);
deltaA1 = deg2rad(0);
TimeStep = 0.05;                      % ����ÿ��ѭ��ʱ��
T = 0.0;                              % �������ʱ��
JointNum = 8;                        % ���ùؽڸ���
A1 = A*(1+deltaA*sign(sin(K*(j-1)+W*T)));
a = A1/(-2*sin(K/2));


% �������ݴ洢����
DataNum = 1;
DataT = zeros(1,10000);
Datas = zeros(6,10000);
SnakePostion = zeros(2,10000);

x1=-0.629;y1=-1.748;
x2=-0.812;y2=-1.991;
x3=-1.486;y3=-1.988;
x4=-1.713;y4=-1.913;
x5=-1.737;y5=-1.405;



%% ��������
% ���ӳɹ�
if (clientID>-1)
    disp('Connected to remote API server');
    % ��ȡ�ؽھ��
    for i = 1 : JointNum
        JointName = ['Joint',num2str(i)];
        eval(['[~,Handle',JointName,'] = vrep.simxGetObjectHandle(clientID,JointName,vrep.simx_opmode_oneshot_wait);'])
    end
    
    % ��ȡ�ؽڻ�������
    for i = 1 : (JointNum/2+1)
        DummyName = ['Dummy',num2str(i)];
        eval(['[~,Handle',DummyName,'] = vrep.simxGetObjectHandle(clientID,DummyName,vrep.simx_opmode_oneshot_wait);'])
    end
    
    % ��ȡ���˾��
    [~,HandleLink0] = vrep.simxGetObjectHandle(clientID,'Snake',vrep.simx_opmode_oneshot_wait);
    for i = 1 : (JointNum-1)
        LinkName = ['Link',num2str(i)];
        eval(['[~,Handle',LinkName,'] = vrep.simxGetObjectHandle(clientID,LinkName,vrep.simx_opmode_oneshot_wait);'])
    end
 
    
    % ��ȡ������
    [~,HandleVs] = vrep.simxGetObjectHandle(clientID,'Vs',vrep.simx_opmode_oneshot_wait);

    vrep.simxSynchronous(clientID,1); 
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    vrep.simxAddStatusbarMessage(clientID,'deltaA = deg2rad(0)',vrep.simx_opmode_oneshot);


    % Vrep��������״̬
    while(vrep.simxGetConnectionId(clientID) ~= -1)
 
        % ��ȡʵ�ʹؽڽ�
        RealJoints = zeros(1,JointNum); % �ؽڽ�
        for m = 1:JointNum
            [~,RealJoints(m)] = vrep.simxGetJointPosition(clientID, eval((['HandleJoint',num2str(m)])), vrep.simx_opmode_oneshot);
        end
        % ƽ���ؽڽǣ�ע�����ڸ����ؽڵĴ��ڣ�������Ҫ����2
        AveRealJoints = mean(RealJoints)*2;
        
        % ��ȡʵ�����˽ǣ����㷽���
        Dir = 0;
        for m = 1:(JointNum/2+1)
            [~,DummyDirection] = vrep.simxGetObjectOrientation(clientID, eval((['HandleDummy',num2str(m)])), -1, vrep.simx_opmode_oneshot);
            Dir = Dir + DummyDirection(3);
        end
        Dir = Dir/(JointNum/2+1);
        
        % ��ȡ����λ��
        LinkPosition = zeros(3,JointNum);
        for m = 1:JointNum
            [~,LinkPosition(:,m)] = vrep.simxGetObjectPosition(clientID,eval((['HandleLink',num2str(m-1)])),-1,vrep.simx_opmode_oneshot);
        end
        SnakePostion(1,DataNum) = mean(LinkPosition(1,:));
        SnakePostion(2,DataNum) = mean(LinkPosition(2,:));
 
         % ����ת��Ч��
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


        % �������Ѳ�̬�˿̸��ؽڽǶ�ֵ
        Joints = zeros(1,JointNum);
        Joints(2)=A*(1+deltaA1*sign(sin(K+W*T)))*(1-exp(-T))*sin(K+W*T)+g*a*cos(W*T+h*K);
        for j = 4:2:JointNum
            Joints(j) = A*(1+deltaA*sign(sin(K*(j-1)+W*T)))*(1-exp(-T))*sin(K*(j-1)+W*T);
        end

        % ���͸������������Ƕ�
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
    


    % ֹͣ���沢�Ͽ�����
    disp('������ֹͣ�����ڶϿ�����...');
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    vrep.simxGetPingTime(clientID);
    vrep.simxFinish(clientID);
else
    fprintf('Warning: Failed connecting to remote API server!\n');
end

%% �������
vrep.delete();
disp('Program end');
