%% 计算CPG网络输出
% 相关fundction： Hopf_Network
clear;clc;
ts = 100;
global gait_state;
gait_state = 5;%步态模式选择
%不同步态的初值设置
if gait_state == 3
    Set_x = [0;-0.001;0;0.02;0;-0.002;0;0.003;0;-0.003;0;0.001]; 
elseif gait_state == 4
    Set_x = [0.0345,-0.9996,0.6983,0.7159,-0.7140,0.7001,0.0345,-0.9996,0.6983,0.7159,-0.7140,0.7001];  
elseif gait_state == 5
    Set_x = [0.0129,1.0002,0.9279,0.3724,-0.9877,0.1556,-0.6935,0.7206,0.3443,-0.9391,0.6224,0.7829];  
end
[t,x] = ode45(@Hopf_Network,[0 ts],Set_x);

%% 滤波
% % 中值滤波
% for i = 1:6
%     x_smooth_med(:,i) = medfilt1(x(:,2*i-1),10);
% end
% figure(3)
% plot(t,x_smooth_med(:,1),t,x_smooth_med(:,2),t,x_smooth_med(:,3),t,x_smooth_med(:,4),t,x_smooth_med(:,5),t,x_smooth_med(:,6));

% 低通滤波



%% 单腿映射函数，CPG网络输出生成关节角度，保存文件
k0 = 0.3;
k1 = -0.2;
k2 = 0;
k3 = 0.3;
j=1;
for i =1:6    
    if i<=3
        Output_jointJ_legI(:,1,i) = k0.*x(:,j);
    else
        Output_jointJ_legI(:,1,i) = -k0.*x(:,j);
    end
   
    for g = 1:length(t)
        if(x(g,j+1)>0)
            x1(g,j+1) = k1*x(g,j+1);
        else
            x1(g,j+1) = k2*x(g,j+1);
        end
    end
    Output_jointJ_legI(:,2,i) = x1(:,j+1);
    Output_jointJ_legI(:,3,i) = k3.*Output_jointJ_legI(:,2,i);    
    j = j+2;   
end
% if gait_state == 3
%     save('jointJ_legI_3.mat','Output_jointJ_legI');
% elseif gait_state == 4
%     save('jointJ_legI_4.mat','Output_jointJ_legI');
% elseif gait_state == 5
%     save('jointJ_legI_5.mat','Output_jointJ_legI');
% end

%% 振荡器输出到足端轨迹的映射
% % 足端轨迹设为椭圆方程
% H = 1; % 长轴，为摆动相抬腿高度
% b = 1; % 短轴，为支撑相
% for i = 1:6
%     dx = diff(x_smooth_med(:,i))./diff(t);
%     for j = 1:length(x_smooth_med(:,i))-1
%         if dx(j) >= 0 
%             y(j,i) = real(H*sqrt(b^2-x_smooth_med(j,i)^2)/b);    
%         else
%             y(j,i) = x_smooth_med(j,i)*0;
%         end
%     end
% end


%% 画图
% figure(1);
% plot(t,x(:,1),t,x(:,3),t,x(:,5),t,x(:,7),t,x(:,9),t,x(:,11));
figure(2);
% plot(t(1:end-1),y(:,1),t(1:end-1),y(:,2),t(1:end-1),y(:,3),t(1:end-1),y(:,4),t(1:end-1),y(:,5),t(1:end-1),y(:,6));
% plot(t(1:end-1),smooth_dx(:));
plot(t, Output_jointJ_legI(:,1,1),t, Output_jointJ_legI(:,2,1),t, Output_jointJ_legI(:,3,1),t, Output_jointJ_legI(:,1,2),t, Output_jointJ_legI(:,2,2),t, Output_jointJ_legI(:,3,2));
xlabel('t');
ylabel('output');






