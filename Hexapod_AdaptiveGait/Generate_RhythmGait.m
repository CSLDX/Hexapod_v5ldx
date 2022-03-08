%% ��������˵Ľ����˶���̬(3/4/5�㲽̬)
%**************************************************************************
%******************��Authors��LDX��Date��2021/7��**********************
% Ϊ��ģ�黯��ƣ����û�������ÿ�����������󶼻ָ�����ʼλ��
% ����˹켣�����ĺ�����ɢ������ɢ��ɵ�������ɢ�㰴��̬��λ��滮��ɢ��
%**************************************************************************
clear;    clc;
%**************************************************************************
% �������˵Ľṹ��λ�β�����
%**************************************************************************
global BR Alpha LL1 LL2 LL3 Theta1 Theta2 Theta3
% ����뾶R����λ��m��
BR = 0.0800602;
LL1 = 0.0514982;
LL2 = 0.0723751;
LL3 = 0.1165047;
Theta1 = pi/180*0;
Theta2 = -pi/180*30;
Theta3 = pi/180*120;
% ����֧���ֲ��� ����λ��rad��
Alpha = [0, pi/3, 2*pi/3, pi, -2*pi/3, -pi/3];
offset_x = 0;
offset_y = 0;
offset_z = 0;

ST = 0;
SW_UP = 1;
SW_DOWN = 2;

%% ���������ڣ���˵��˶��������á� ���沽��10ms
% �ڶ���-֧�����־����1���ڶ��ࣻ-1��֧���ࣩ
global stepHeight stepAmplitude Zrotate
stepHeight = input('***��ʾ*** ������ڶ��ȸ߶�stepHeight����λ��m����'); 
stepAmplitude = input('***��ʾ*** ��������������stepAmplitude����λ��m����');
Zrotate = input('***��ʾ*** ������ǰ������ǣ���λ��deg����') * (pi/180);
gait_flag = input('***��ʾ*** ��ȷ�������˵Ĳ�̬ģʽ��3/4/5 �㣿����');
% e.g: 0.03 0.03 any 3/4/5

switch gait_flag
    case {3},   SwiSur = [1 -1 1 -1 1 -1;  -1 1 -1 1 -1 1];
    case {4},   SwiSur = [1 -1 -1 1 -1 -1;  -1 1 -1 -1 1 -1;  -1 -1 1 -1 -1 1];
    case {5},   SwiSur = [1 -1 -1 -1 -1 -1;  -1 1 -1 -1 -1 -1;  -1 -1 1 -1 -1 -1;  -1 -1 -1 1 -1 -1;  -1 -1 -1 -1 1 -1;  -1 -1 -1 -1 -1 1];
    otherwise,  error('��������̬ģʽ���󣡣���');
end
[numGroup, ~] = size(SwiSur);   % ������
floorfactor = gait_flag / 6;    % ռ��ϵ��
dutyfactor = 1 - floorfactor;   % ռ�ձ�
ksw = round(floorfactor/dutyfactor);    % �����ڶ����ʱ��ϵ��

%% ��˹켣��������ɢ��
% igTend = 1;           % ���ڶ�����ʱ��
% swigNum = 15;          % ���ڶ����ڲ�ֵ����
% stigNum = ksw*swigNum;  % ��֧�����ڲ�ֵ����
% dt = igTend / swigNum;% ����ʱ����
% SwigTimeSeq = linspace(0,igTend, swigNum)./igTend; % ���ڶ����ڵ�ʱ������
% StigTimeSeq = linspace(0,igTend, stigNum)./igTend;
% 
% % �����ڶ������ڵ����Һ�����
% Swz = stepHeight*sin(SwigTimeSeq.*pi);
% Swx = SwigTimeSeq.*stepAmplitude*cos(Zrotate);
% Swy = SwigTimeSeq.*stepAmplitude*sin(Zrotate);
% % ����֧�������ڵ����Һ�����
% Stz = zeros(1, length(StigTimeSeq));
% Stx = fliplr(StigTimeSeq).*stepAmplitude*cos(Zrotate);
% Sty = fliplr(StigTimeSeq).*stepAmplitude*sin(Zrotate);

% new
swigNum_sw = 40;          % ���ڶ����ڲ�ֵ����
swigNum_up = 30;
swigNum_down = 30;

swigNum = swigNum_sw+swigNum_up+swigNum_down;
stigNum = ksw*swigNum;  % ��֧�����ڲ�ֵ����
% ���ڶ����ڵ�ʱ������
SwigTimeSeq_sw = linspace(0,1, swigNum_sw); 
SwigTimeSeq_up = linspace(0,1, swigNum_up);
SwigTimeSeq_down = linspace(0,1, swigNum_down);
SwigTimeSeq = linspace(0,1, swigNum);
% ��֧�����ڵ�ʱ������
%     StigTimeSeq = linspace(0,1, stigNum);
Gait_num = swigNum + stigNum;
% �����ڶ������ڵ����Һ�����  
Swx = [0*ones(1,swigNum_up), stepAmplitude*cos(Zrotate)*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*cos(Zrotate)*ones(1,swigNum_down)];
Swy = [0*ones(1,swigNum_up), stepAmplitude*sin(Zrotate)*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*sin(Zrotate)*ones(1,swigNum_down)];
Swz = stepHeight*(1-(0.5*(cos(SwigTimeSeq.*2*pi)+1)));
% ����֧�������ڵ����Һ�����
stx = [zeros(1,swigNum_down), stepAmplitude*cos(Zrotate)*1/ksw*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*cos(Zrotate)*1/ksw*ones(1,swigNum_up)];
sty = [zeros(1,swigNum_down), stepAmplitude*sin(Zrotate)*1/ksw*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*sin(Zrotate)*1/ksw*ones(1,swigNum_up)];
Stx = stx;
Sty = sty;
for i = 1:ksw-1
    Stx = [Stx, stx+stepAmplitude*cos(Zrotate)*i/ksw];
    Sty = [Sty, sty+stepAmplitude*sin(Zrotate)*i/ksw];
end
Stx = fliplr(Stx);
Sty = fliplr(Sty);
Stz = zeros(1, stigNum);


%% ����λ��滮��˹켣��ɢ��
leg_pos = cell(6,1);
for leg = 1:6
    % ��̬����ʱ������˹켣
    flag = mod(leg,numGroup);
    if(flag == 1)
        leg_z = [Swz,Stz];
        leg_y = [Swy,Sty];
        leg_x = [Swx,Stx];
        pos_state = [SW_UP*ones(1,sum(gradient(Swz)>=0)),SW_DOWN*ones(1,sum(gradient(Swz)<0)),ST*ones(1,length(Stz))];
    else
        if(flag == 0)
            leg_z = [Stz,Swz];
            leg_y = [Sty-stepAmplitude*sin(Zrotate),Swy-stepAmplitude*sin(Zrotate)];
            leg_x = [Stx-stepAmplitude*cos(Zrotate),Swx-stepAmplitude*cos(Zrotate)];
            pos_state = [ST*ones(1,length(Stz)),SW_UP*ones(1,sum(gradient(Swz)>=0)),SW_DOWN*ones(1,sum(gradient(Swz)<0))];
        else
            leg_z = [Stz(end-floor(length(Stz)*(mod(leg,numGroup)-1)/(numGroup-1)):end),Swz,Stz(1:end-floor(length(Stz)*(mod(leg,numGroup)-1)/(numGroup-1))-1)];
            leg_y = [Sty(end-floor(length(Sty)*(mod(leg,numGroup)-1)/(numGroup-1)):end)-stepAmplitude*sin(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1),...
                     Swy-stepAmplitude*sin(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1),...
                     Sty(1:end-floor(length(Sty)*(mod(leg,numGroup)-1)/(numGroup-1))-1)-stepAmplitude*sin(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1)];

            leg_x = [Stx(end-floor(length(Stx)*(mod(leg,numGroup)-1)/(numGroup-1)):end)-stepAmplitude*cos(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1),...
                     Swx-stepAmplitude*cos(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1),...
                     Stx(1:end-floor(length(Stx)*(mod(leg,numGroup)-1)/(numGroup-1))-1)-stepAmplitude*cos(Zrotate)*(mod(leg,numGroup)-1)/(numGroup-1)];
            pos_state = [ST*ones(1,length(Stz(end-floor(length(Stz)*(mod(leg,numGroup)-1)/(numGroup-1)):end))),SW_UP*ones(1,sum(gradient(Swz)>=0)),SW_DOWN*ones(1,sum(gradient(Swz)<0)),ST*ones(1,length(Stz(1:end-floor(length(Stz)*(mod(leg,numGroup)-1)/(numGroup-1))-1)))];
        end
    end
    % ��������legbase��ƫ������������vrepͨ�Ŷ�ȡ��
    offset_x = cos(Alpha(leg))*BR+cos(Alpha(leg)+Theta1)*(LL1+LL2*cos(Theta2)+LL3*cos(Theta3+Theta2));
    offset_y = sin(Alpha(leg))*BR+sin(Alpha(leg)+Theta1)*(LL1+LL2*cos(Theta2)+LL3*cos(Theta3+Theta2));
    offset_z = 0;
    % һ����̬���ڵ���˹켣λ��
    leg_pos{leg} = [offset_x+leg_x; offset_y+leg_y; offset_z+leg_z; pos_state];
end



%% **************************************************************************
% ����ͼ��
%**************************************************************************
figure(1);
for L = 1 : 2
    plot(1:swigNum+stigNum, leg_pos{L,1}(1,:));
    hold on;
end
xlabel('Seq');
ylabel('/m');


figure(2);
for L = 1:6
    plot3(leg_pos{L,1}(1,:), leg_pos{L,1}(2,:), leg_pos{L,1}(3,:));

    hold on;
end
xlabel('x/m');
ylabel('y/m');
zlabel('z/m')
legend('Leg1','Leg2','Leg3','Leg4','Leg5','Leg6');
% xlim([-0.7 0.7])
% ylim([-0.4 0.4])
% zlim([0 0.5])
% view([45,45])
% ���������0.2 0.5 20 3
%% �洢���ؽڽǶ�ֵ
% save(['Leg_Pos',num2str(gait_flag),'.mat'], 'leg_pos');
% disp('***** ִ����ϣ�*****');
