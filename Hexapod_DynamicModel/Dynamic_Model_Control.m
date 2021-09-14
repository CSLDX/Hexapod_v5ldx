%% PID Control Test
clear all; 
syms Theta0 Theta1 Theta2 Theta3 % �Ƕ�
syms W_Theta1 W_Theta2 W_Theta3 % ���ٶ�
syms A_Theta1 A_Theta2 A_Theta3 % �Ǽ��ٶ�
syms L0 L1 L2 L3 M1 M2 M3 G % ���˳��Ⱥ�����
%% ����
dt = 0.05;
t = 1:dt:100; % ����ʱ��
g = 9.8;
% �������
power = 16.32; % ������ʣ���λ��W   P = FV  (N/m)(deg/s) 
% max_velocity = 9999; % �����ٶ�
max_torque = 10; % ���Ť��
% ��������ṹ���� ���ȵ�λ��m ������λ��kg �Ƕȵ�λ��rad
LL1 = 0.0514982;
LL2 = 0.0723751;
LL3 = 0.1165047;
LL = [LL1,LL2,LL3];
ML1 = 0.05;
ML2 = 0.05;
ML3 = 0.05;
theta_c = [0;-pi/180*30;pi/180*120];
w_c = [0;0;0];
a_c = [0;0;0];
theta_d = [];
load('G_theta.mat');
load('M_theta.mat');
load('V_theta_wtheta.mat');
% ע����sin cos �Ƕȵ�λΪ��rad
G_theta = subs(G_theta,{L1,L2,L3,M1,M2,M3,G},{LL1,LL2,LL3,ML1,ML2,ML3,g});
% G_theta_c = subs(G_theta,{Theta1,Theta2,Theta3},{theta_c(1),theta_c(2),theta_c(3)});
M_theta = subs(M_theta,{L1,L2,L3,M1,M2,M3},{LL1,LL2,LL3,ML1,ML2,ML3});
% M_theta_c = subs(M_theta,{Theta1,Theta2,Theta3},{theta_c(1),theta_c(2),theta_c(3)});
V_theta_wtheta = subs(V_theta_wtheta,{L1,L2,L3,M1,M2,M3},{LL1,LL2,LL3,ML1,ML2,ML3});
% V_theta_wtheta_c = subs(V_theta_wtheta,{Theta1,Theta2,Theta3,W_Theta1,W_Theta2,W_Theta3},{theta_c(1),theta_c(2),theta_c(3),w_c(1),w_c(2),w_c(3)});
% λ����pid
Kp = [100,600,80000];
Kd = [16000,16000,20000];
% Kp = [5,30,20];
% Kd = [80,800,1000];

e = [0,0,0];
last_e = [0,0,0];
last_last_e = [0,0,0];
de = [0,0,0];
u = zeros(3,length(t)+1);
du = zeros(3,length(t));
torque = zeros(3,length(t));

%% Ԥ��켣����
x = 0.03*(sin(pi/180*10*t-pi/2)+1)+0.1142;
y = 0.03*(sin(pi/180*10*t-pi/2)+1);
% y = zeros(1,length(t));
z = sin(pi/180*10*t);
for i = 1:length(t)
    if z(i) < 0
        z(i) = 0.0703*(0-1);
    else
        z(i) = 0.02*z(i)-0.0703;
    end
end
% z = -0.0803*ones(1,length(t));
% figure(1);
% plot3(x,y,z);
% xlabel('X');ylabel('Y');zlabel('Z');

%% ���˶�ѧ���ؽڽǶ�
for i = 1:length(t)
    tempA = sqrt(x(i)^2 + y(i)^2 + z(i)^2 + LL1^2 - 2*LL1*sqrt(x(i)^2+y(i)^2));
    tempB = sqrt(x(i)^2+y(i)^2) - LL1;
    tempC = acos((tempA^2 + tempB^2 - z(i)^2)/(2*tempA*tempB));
    tempD = acos((tempA^2 + LL2^2 - LL3^2)/(2*tempA*LL2));
    theta_d(1,i) = atan(y(i)/x(i));
    theta_d(2,i) = -(tempD - tempC);
    theta_d(3,i) = -(acos((LL2^2 + LL3^2 - tempA^2)/(2*LL2*LL3)) - pi);   
    
end
figure(2);
plot(t,theta_d(1,:));
hold on;
figure(3);
plot(t,theta_d(2,:));
hold on;
figure(4);
plot(t,theta_d(3,:));
hold on;

%% PID 
for tt = 1:length(t)
    % λ����pid
    for j = 1:3
        e(j) = theta_d(j,tt) - theta_c(j,tt);
        de(j) = e(j) - last_e(j);
        u(j,tt) = Kp(j)*e(j) + Kd(j)*de(j);
        last_e(j) = e(j);        
    end  
    % ��������޷�   
    for j = 1:3 
        if abs(u(j,tt)) > max_torque
            torque(j,tt) = sign(u(j,tt))*max_torque;
        else
            torque(j,tt) = u(j,tt);
        end
    end
   
    
    % ������������������ˣ�ģ�ͣ��õ��������
    G_theta_c = vpa(subs(G_theta,{Theta1,Theta2,Theta3},{theta_c(1,tt),theta_c(2,tt),theta_c(3,tt)}));
    M_theta_c = vpa(subs(M_theta,{Theta1,Theta2,Theta3},{theta_c(1,tt),theta_c(2,tt),theta_c(3,tt)}));
    V_theta_wtheta_c = vpa(subs(V_theta_wtheta,{Theta1,Theta2,Theta3,W_Theta1,W_Theta2,W_Theta3},{theta_c(1,tt),theta_c(2,tt),theta_c(3,tt),w_c(1),w_c(2),w_c(3)}));
    
    a_c = M_theta_c*(torque(:,tt) - V_theta_wtheta_c - G_theta_c);
    w_c = w_c + a_c*dt;
    theta_c(:,tt) = theta_c(:,tt) + w_c*dt;
    theta_c(:,tt+1) = theta_c(:,tt);
    
end
figure(2);
plot(t,theta_c(1,1:length(t)));
xlabel('t');
ylabel('�Ƕ�/rad');
legend('Theta1�����Ƕ�','Theta1ʵ�ʽǶ�');
set(gca,'XTick',1:200:length(t));
figure(3);
plot(t,theta_c(2,1:length(t)));
xlabel('t');
ylabel('�Ƕ�/rad');
legend('Theta2�����Ƕ�','Theta2ʵ�ʽǶ�');
set(gca,'XTick',1:200:length(t));
figure(4);
plot(t,theta_c(3,1:length(t)));
xlabel('t');
ylabel('�Ƕ�/rad');
legend('Theta3�����Ƕ�','Theta3ʵ�ʽǶ�');
set(gca,'XTick',1:200:length(t));

save('Torque.mat','torque');
temp_theta_d = theta_d*180/pi; % �ɻ���ת���ɽǶ�
save('Theta_d.mat','theta_d');








