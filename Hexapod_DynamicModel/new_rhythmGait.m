%% ��������˵Ľ����˶���̬(3/4/5�㲽̬)
%**************************************************************************
%*****************��Authors��AndyGao��Date��2020/11��**********************
% Ϊ��ģ�黯��ƣ����û�������ÿ�����������󶼻ָ�����ʼλ��
%**************************************************************************
clear;    clc;

%% �����˽ṹ����
global BR L1 L2 L3 L4 Alpha
% ����뾶R���Ȳ������˲���L�����̰뾶r ����λ��m��
L1 = 0.09;    L2 = 0.15;    L3 = 0.16;    L4 = 0.15;    BR = 0.18;
% ����֧���ֲ��� ����λ��rad��
Alpha = [pi/6, pi/2, 5*pi/6, -5*pi/6, -pi/2, -pi/6];

%% �����˵ĳ�ʼλ�Ͳ���
global Theta_0 beta_0 phi_0
Theta_0 = zeros(6,4);   % ���ؽڽǶȳ�ֵ ����λ��rad��
beta_0 = (pi/180)*(0);  % ���̵���Գ�ʼת�� ����λ��rad��
phi_0 = (pi/180)*(0);   % ����ƽ̨�ĳ�ʼ������ ����λ��rad��

%% �����˶�����
global liftH Xstride Ystride Zrotate
liftH = 0.10;           % �ڶ����̧�ȸ߶�(LiftH)����λ��m��
% ��������(���������ɷ���)����������(Stride)
Xstride = input('***��ʾ*** ����������������� X�� �������������Xstride����λ��m����');
Ystride = input('***��ʾ*** ����������������� Y�� �������������Ystride����λ��m����');
Zrotate = input('***��ʾ*** ����������������� Z�� ��������������Zrotate����λ��deg����') * (pi/180);

% �ڶ���-֧�����־����1���ڶ��ࣻ-1��֧���ࣩ
bool_flag = 0;
while (bool_flag == 0)
    gait_flag = input('***��ʾ*** ��ȷ�������˵Ĳ�̬ģʽ��3/4/5 �㣿����');
    if (gait_flag == 3)
        SwiSur = [1 -1 1 -1 1 -1
                  -1 1 -1 1 -1 1];
        bool_flag = 1;
    elseif (gait_flag == 4)
        SwiSur = [1 -1 -1 1 -1 -1
                  -1 1 -1 -1 1 -1
                  -1 -1 1 -1 -1 1];
        bool_flag = 1;
    elseif (gait_flag == 5)
        SwiSur = [1 -1 -1 -1 -1 -1
                  -1 1 -1 -1 -1 -1
                  -1 -1 1 -1 -1 -1
                  -1 -1 -1 1 -1 -1
                  -1 -1 -1 -1 1 -1
                  -1 -1 -1 -1 -1 1];
        bool_flag = 1;
    else
        bool_flag = 0;
    end
end
[row, ~] = size(SwiSur);
duty = gait_flag / 6;   % ռ�ձ�

%% �����ڶ������ڣ��˶��������á�
swigTend = 1;           % ���ڶ�����ʱ�������Զ��塿
swigNum = 100;          % ���ڶ����ڲ�ֵ��������Ķ�������vrep_main��Ҳ������Ӧ�޸ġ�
SwigTimeSeq = linspace(0,swigTend, swigNum); % ���ڶ����ڵ�ʱ������
% �����ڶ������ڵ�Sigmoid�Ͳ�ֵ������
const = 20;             % Sigmoid��ֵ����
Tg = swigTend / 2;      % Sigmoid��ֵ�еĸ���ʱ�����
Gamma = 1 ./ (1 + exp(-const .* (SwigTimeSeq - Tg)));	% Sigmoid����

%% �������������ڣ����(����ڻ���ϵ)��λ�ơ�
% �������洢����
StriRelFootPose = cell(6,1);	% �����������������ڻ���ϵ��λ�� [srFx;srFy;srFz]
StriTheta = cell(6,1);          % �����������ؽڽǶȾ��� [q1;q2;q3;q4]
StriMovRate = cell(6,1);        % ���������������������ڣ������˲ʱ�˶���������
StriRelBodyPose = zeros(4,row*swigNum); % ������������������ڻ���ϵ��λ�� [srBx;srBy;srBz;srByaw]

r = 0;
while (r < row)
    r = r + 1;
    % ��ǰ�ڶ������ڣ��Ȳ��İڶ�/֧��״̬
    if (mod(r,row) ~= 0)
        SwiSur_cur = SwiSur(mod(r,row), :);
    else
        SwiSur_cur = SwiSur(end, :);
    end
    
for L = 1 : 6
    alphaL = Alpha(L); % ��ǰ֧���ķ�λ��
    % �������ڻ���ϵ�ĳ�ʼλ��
    Fx_0 = BR*cos(alphaL) + (L1 + L2*cos(Theta_0(L,2)) + L3*sin(Theta_0(L,2)+Theta_0(L,3)) +...
           L4*sin(Theta_0(L,2)+Theta_0(L,3)+Theta_0(L,4))) * cos(alphaL + Theta_0(L,1));
    Fy_0 = BR*sin(alphaL) + (L1 + L2*cos(Theta_0(L,2)) + L3*sin(Theta_0(L,2)+Theta_0(L,3)) +...
           L4*sin(Theta_0(L,2)+Theta_0(L,3)+Theta_0(L,4))) * sin(alphaL + Theta_0(L,1));
    Fz_0 = L2*sin(Theta_0(L,2)) - L3*cos(Theta_0(L,2)+Theta_0(L,3)) - L4*cos(Theta_0(L,2)+Theta_0(L,3)+Theta_0(L,4));
    
    % �����˲ʱ�˶�����������ֵ
    StriMovRate{L}(1,1) = 0;
    % �Ȳ�״̬�ķ��ű�ǩ���ڶ���Ϊ1��֧����Ϊ-1
    sgnL = SwiSur_cur(L);
    % ���ڶ������ڣ��ڶ���/֧������˶������ܱ���������ֵ��
    swigMovRate = (1+sgnL)/2 * sgnL * duty + (1-sgnL)/2 * sgnL * (1 - duty); % ǰ�߶�Ӧ�ڶ��࣬���߶�Ӧ֧����
    
    for k = 1 : swigNum
        kk = k + (r - 1) * swigNum;
        %--------------------------------------------------------------------------
        % �����������ڣ���˵ĵѿ���λ��
        %--------------------------------------------------------------------------
        StriMovRate{L}(1,kk) = StriMovRate{L}(1,max(1,(r-1)*swigNum)) + swigMovRate * Gamma(k); % �����˲ʱ�˶���������
        % �������ڻ���ϵ��˲ʱλ������
        StriRelFootPose{L}(1,kk) = Fx_0 * cos(Zrotate*StriMovRate{L}(1,kk)) - Fy_0 * sin(Zrotate*StriMovRate{L}(1,kk)) + Xstride*StriMovRate{L}(1,kk);
        StriRelFootPose{L}(2,kk) = Fx_0 * sin(Zrotate*StriMovRate{L}(1,kk)) + Fy_0 * cos(Zrotate*StriMovRate{L}(1,kk)) + Ystride*StriMovRate{L}(1,kk);
        StriRelFootPose{L}(3,kk) = Fz_0 + (1+sgnL)/2 * liftH * (-0.5 * cos(2*pi/swigTend * SwigTimeSeq(k)) + 0.5);
        
        %--------------------------------------------------------------------------
        % �����������ڣ��˶�ѧ���
        %--------------------------------------------------------------------------
        Fx_cur = StriRelFootPose{L}(1,kk); % ��ǰʱ�̵���˺�����
        Fy_cur = StriRelFootPose{L}(2,kk); % ��ǰʱ�̵����������
        Fz_cur = StriRelFootPose{L}(3,kk); % ��ǰʱ�̵����������
        phi_i  = phi_0;                 % ����ƽ̨��˲ʱ������
        beta_i = beta_0;                % ���̵�ת��
        % ��ǰʱ�̵Ĺؽڱ���
        theta1 = atan2(Fy_cur - BR*sin(alphaL), Fx_cur - BR*cos(alphaL)) - alphaL;
        tempA = Fz_cur + L4*cos(beta_i-sign(alphaL)*phi_i);
        tempB = (Fy_cur - BR*sin(alphaL))/sin(alphaL+theta1) - L1 - L4*sin(beta_i-sign(alphaL)*phi_i);
        tempC = L2;
        tempD = -L3;
        theta3 = asin((tempA^2+tempB^2-tempC^2-tempD^2) / (-2*tempC*tempD));
        theta2 = atan((-2*tempC*tempD*cos(theta3)) / (tempA^2+tempB^2+tempC^2-tempD^2)) + atan2(tempA , tempB);
        theta4 = beta_i - sign(alphaL)*phi_i - theta2 - theta3;
        % �洢�ؽڱ���
        Theta_ik = [theta1; theta2; theta3; theta4];
        StriTheta{L} = [StriTheta{L}, Theta_ik]; % �������ڸ��ؽڽǶ�ֵ
        
        %--------------------------------------------------------------------------
        % �����������ڣ����ĵ����λ��
        %--------------------------------------------------------------------------
        StriRelBodyPose(4,kk) = StriRelBodyPose(4,max(1,(r-1)*swigNum)) + Zrotate / 2 * Gamma(k);
        StriRelBodyPose(1:3,kk) = StriRelBodyPose(1:3,max(1,(r-1)*swigNum)) + rotz(Zrotate / 2 * Gamma(k)) * ( [Xstride;Ystride;0] / 2 * Gamma(k) );
        
    end
end
end

%% �ಽ�����������˶�
% �����˶���������
rhyNum = 1;                    % ���ڽ����˶��Ĳ����������Զ��塿
RhyRelFootPose = cell(6,1);     % �����˶���������λ�� % [rrFx;rrFy;rrFz]
RhyAbsFootPose = cell(6,1);     % �����˶�����˾���λ�� % [raFx;raFy;raFz]
RhyTheta = cell(6,1);           % �����˶��Ĺؽ�λ�� [q1;q2;q3;q4]
RhyRelBodyPose = [];            % �����˶��Ļ�������ڻ���ϵ��λ�� [rrBx;rrBy;rrBz;rrByaw]
RhyAbsBodyPose = [];            % �����˶��Ļ������������ϵ�ľ���λ�� [raBx;raBy;raBz;raByaw]
RhyTimeSeq = linspace(0,row*swigTend*rhyNum, row*swigNum*rhyNum); % �����˶��ķ���ʱ��

% �����˶������λ��
for n = 1 : rhyNum
    for L = 1 : 6
        RhyRelFootPose{L} = [RhyRelFootPose{L}, StriRelFootPose{L}];	% [rrFx;rrFy;rrFz]
        RhyTheta{L} = [RhyTheta{L}, StriTheta{L}];                      % [q1;q2;q3;q4]
    end
    RhyRelBodyPose = [RhyRelBodyPose, StriRelBodyPose];                 % [rrBx;rrBy;rrBz;rrByaw]
end

%% ��ͼ
for L = 1 : 6
    figure(L);    clf;
    hold on;    box on;
    plot(RhyTimeSeq, RhyTheta{L}(1,:), 'k:','LineWidth',2);
    plot(RhyTimeSeq, RhyTheta{L}(2,:), 'r:','LineWidth',2);
    plot(RhyTimeSeq, RhyTheta{L}(3,:), 'g:','LineWidth',2);
    plot(RhyTimeSeq, RhyTheta{L}(4,:), 'b:','LineWidth',2);
    plot(RhyTimeSeq, RhyRelFootPose{L}(1,:), '-r','LineWidth',3);
    plot(RhyTimeSeq, RhyRelFootPose{L}(2,:), '-g','LineWidth',3);
    plot(RhyTimeSeq, RhyRelFootPose{L}(3,:), '-b','LineWidth',3);
end
% %% �洢���ؽڽǶ�ֵ
% JointAngles = RhyTheta;
% save(['Datas/Gait',num2str(gait_flag),'_JointAngles'], 'JointAngles');
% %% �洢����λ�ñ仯ֵ
% FootPose = RhyRelFootPose;
% save(['Datas/Gait',num2str(gait_flag),'_FootPose'], 'FootPose');

disp('***** ִ����ϣ�*****');
