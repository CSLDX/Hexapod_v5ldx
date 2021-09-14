clc;clear all;
syms Mj Lj G t
% 符号变量，不含t的替换变量
syms Theta0 Theta1 Theta2 Theta3 % 角度
syms W_Theta1 W_Theta2 W_Theta3 % 角速度
syms A_Theta1 A_Theta2 A_Theta3 % 角加速度
% 关于t的函数
syms Theta1t(t) Theta2t(t) Theta3t(t) % 角度
syms W_Theta1t(t) W_Theta2t(t) W_Theta3t(t) % 角速度
% 常量定义
syms L0 L1 L2 L3 M1 M2 M3 % 连杆长度和质量

T0_1 = [cos(Theta0+Theta1) -sin(Theta0+Theta1) 0 L0*cos(Theta0);
        sin(Theta0+Theta1) cos(Theta0+Theta1)  0 L0*sin(Theta0);
        0                  0                   1 0;
        0                  0                   0 1
       ];
T1_2 = [cos(Theta2) -sin(Theta2) 0 L1;
        0           0           -1 0;
        sin(Theta2) cos(Theta2)  0 0;
        0           0            0 1
       ];
T2_3 = [cos(Theta3) -sin(Theta3) 0 L2;
        sin(Theta3) cos(Theta3)  0 0;
        0           0            1 0;
        0           0            0 1
       ];
T3_4 = [1 0 0 L3;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1
       ];
Ij = [(Mj*Lj^2)/3 0 0 Mj*Lj/2;
      0           0 0 0;
      0           0 0 0;
      Mj*Lj/2     0 0 Mj
     ];

I1 = subs(Ij,{Mj,Lj},{M1,L1});
I2 = subs(Ij,{Mj,Lj},{M2,L2});
I3 = subs(Ij,{Mj,Lj},{M3,L3});
% D-H建模，齐次变换矩阵
T0_2 = simplify(T0_1*T1_2);
T0_3 = simplify(T0_1*T1_2*T2_3);
T0_4 = simplify(T0_1*T1_2*T2_3*T3_4);
%% 动能计算
% 连杆1的动能k1
k1 = 0;
for g = 1:1:1
    for k = 1:1:1
        theta_g = ['Theta',num2str(g)];
        theta_k = ['Theta',num2str(k)];
        W_theta_g = ['W_Theta',num2str(g)];
        W_theta_k = ['W_Theta',num2str(k)];
        eval(['k1 = k1 + diff(T0_1,',theta_g,')*I1*diff(transpose(T0_1),',theta_k,')*',W_theta_g,'*',W_theta_k,';']); 
    end
end
k1 = 1/2*trace(k1);
k1 = simplify(k1);
% 连杆2的动能k2
k2 = 0;
for g = 1:1:2
    for k = 1:1:2
        theta_g = ['Theta',num2str(g)];
        theta_k = ['Theta',num2str(k)];
        W_theta_g = ['W_Theta',num2str(g)];
        W_theta_k = ['W_Theta',num2str(k)];
        eval(['k2 = k2 + diff(T0_2,',theta_g,')*I2*diff(transpose(T0_2),',theta_k,')*',W_theta_g,'*',W_theta_k,';']); 
    end
end
k2 = 1/2*trace(k2);
k2 = simplify(k2);
% 连杆3的动能k3
k3 = 0;
for g = 1:1:3
    for k = 1:1:3
        theta_g = ['Theta',num2str(g)];
        theta_k = ['Theta',num2str(k)];
        W_theta_g = ['W_Theta',num2str(g)];
        W_theta_k = ['W_Theta',num2str(k)];
        eval(['k3 = k3 + diff(T0_3,',theta_g,')*I3*diff(transpose(T0_3),',theta_k,')*',W_theta_g,'*',W_theta_k,';']); 
    end
end
k3 = 1/2*trace(k3);
k3 = simplify(k3);

%% 势能计算 G=9.8N/kg
% 连杆1的势能p1
p1 = 0;
% 连杆2的势能p2
p2 = 1/2*M2*G*L2*sin(Theta2);
% 连杆3的势能p3
p3 = 1/2*M3*G*L3*sin(Theta2+Theta3)+M3*G*L2*sin(Theta2);

%% Lagrange 方程
L = (k1+k2+k3)-(p1+p2+p3);

% 偏导数
dLdTheta1 = diff(L,Theta1);
dLdTheta2 = diff(L,Theta2);
dLdTheta3 = diff(L,Theta3);

dLdW_Theta1 = diff(L,W_Theta1);
dLdW_Theta2 = diff(L,W_Theta2);
dLdW_Theta3 = diff(L,W_Theta3);

dLdW_Theta1t = subs(dLdW_Theta1,{Theta1,Theta2,Theta3,W_Theta1,W_Theta2,W_Theta3},{Theta1t(t),Theta2t(t),Theta3t(t),W_Theta1t(t),W_Theta2t(t),W_Theta3t(t)});
dLdW_Theta2t = subs(dLdW_Theta2,{Theta1,Theta2,Theta3,W_Theta1,W_Theta2,W_Theta3},{Theta1t(t),Theta2t(t),Theta3t(t),W_Theta1t(t),W_Theta2t(t),W_Theta3t(t)});
dLdW_Theta3t = subs(dLdW_Theta3,{Theta1,Theta2,Theta3,W_Theta1,W_Theta2,W_Theta3},{Theta1t(t),Theta2t(t),Theta3t(t),W_Theta1t(t),W_Theta2t(t),W_Theta3t(t)});

dLdW_Theta1dt = diff(dLdW_Theta1t,t);
dLdW_Theta2dt = diff(dLdW_Theta2t,t);
dLdW_Theta3dt = diff(dLdW_Theta3t,t);

F1 = simplify(dLdW_Theta1dt - dLdTheta1);
F2 = simplify(dLdW_Theta2dt - dLdTheta2);
F3 = simplify(dLdW_Theta3dt - dLdTheta3);

F1 = subs(F1,{diff(W_Theta1t(t),t),diff(W_Theta2t(t),t),diff(W_Theta3t(t),t),diff(Theta1t(t),t),diff(Theta2t(t),t),diff(Theta3t(t),t)},{A_Theta1,A_Theta2,A_Theta3,W_Theta1,W_Theta2,W_Theta3});
F1 = subs(F1,{Theta1t(t),Theta2t(t),Theta3t(t),W_Theta1t(t),W_Theta2t(t),W_Theta3t(t)},{Theta1,Theta2,Theta3,W_Theta1,W_Theta2,W_Theta3});

F2 = subs(F2,{diff(W_Theta1t(t),t),diff(W_Theta2t(t),t),diff(W_Theta3t(t),t),diff(Theta1t(t),t),diff(Theta2t(t),t),diff(Theta3t(t),t)},{A_Theta1,A_Theta2,A_Theta3,W_Theta1,W_Theta2,W_Theta3});
F2 = subs(F2,{Theta1t(t),Theta2t(t),Theta3t(t),W_Theta1t(t),W_Theta2t(t),W_Theta3t(t)},{Theta1,Theta2,Theta3,W_Theta1,W_Theta2,W_Theta3});

F3 = subs(F3,{diff(W_Theta1t(t),t),diff(W_Theta2t(t),t),diff(W_Theta3t(t),t),diff(Theta1t(t),t),diff(Theta2t(t),t),diff(Theta3t(t),t)},{A_Theta1,A_Theta2,A_Theta3,W_Theta1,W_Theta2,W_Theta3});
F3 = subs(F3,{Theta1t(t),Theta2t(t),Theta3t(t),W_Theta1t(t),W_Theta2t(t),W_Theta3t(t)},{Theta1,Theta2,Theta3,W_Theta1,W_Theta2,W_Theta3});

%% 整理成一般形式，矩阵表达
% 上面一部分到这一部分的整理，注释掉的是人工整理结果已验证正确，后面用多项式提取系数的方法，减少工作量
% M1_1 = L1^2*(M1/3+M2+M3) + L2^2*cos(Theta2)^2*(M2/3+M3) + L3^2*cos(Theta2+Theta3)^2*M3/3 + L1*L2*cos(Theta2)*(M2+2*M3) + ...
%        L2*L3*M3/2*(cos(2*Theta2+Theta3)+cos(Theta3)) + L1*L3*cos(Theta2+Theta3)*M3;    
% M2_2 = L2^2*(M2/3+M3) + L3^2*M3/3 + L2*L3*M3*cos(Theta3);
% M2_3 = L3^2*M3/3 + L2*L3*cos(Theta3)*M3/2;
% M3_2 = L3^2*M3/3 + L2*L3*cos(Theta3)*M3/2;
% M3_3 = L3^2*M3/3;
% 
% M_theta = [M1_1 0 0;
%            0 M2_2 M2_3;
%            0 M3_2 M3_3
%           ];
% V1_1 = W_Theta1*W_Theta2*(L2^2*(-M2/3*sin(2*Theta2)-M3*sin(2*Theta2))-L3^2*M3/3*sin(2*Theta2+2*Theta3)-...
%        L1*L3*M3*sin(Theta2+Theta3)-L2*L3*M3*sin(2*Theta2+Theta3)-L1*L2*M2*sin(Theta2)-2*L1*L2*M3*sin(Theta2)) - ...
%        W_Theta1*W_Theta3*(L3^2*M3/3*sin(2*Theta2+2*Theta3)+L1*L3*M3*sin(Theta2+Theta3)+L2*L3*M3/2*(sin(Theta3+2*Theta2)+sin(Theta3)));
%    
% V2_1 = W_Theta1^2*(L2^2*M2/6*sin(2*Theta2)+L3^2*M3/6*sin(2*Theta2+2*Theta3)+L2^2*M3/2*sin(2*Theta2)+...
%        L1*L3*M3/2*sin(Theta2+Theta3)+L1*L2*M2/2*sin(Theta2)+L1*L2*M3*sin(Theta2)+L2*L3*M3/2*sin(2*Theta2+Theta3))-...
%        L2*L3*M3*sin(Theta3)*W_Theta2*W_Theta3-L2*L3*M3/2*sin(Theta3)*W_Theta3^2;
%    
% V3_1 = W_Theta1^2*(L2*L3*M3/4*(sin(Theta3)+sin(2*Theta2+Theta3))+L3^2*M3/6*sin(2*Theta2+2*Theta3)+L1*L3*M3/2*sin(Theta2+Theta3))+...
%        L2*L3*M3/2*sin(Theta3)*W_Theta2^2;
% 
% V_theta_wtheta = [V1_1;V2_1;V3_1];
% 
% G_theta = [0;...
%            L3*M3/2*G*cos(Theta2+Theta3)+L2*M2/2*G*cos(Theta2)+L2*M3*G*cos(Theta2);...
%            L3*M3/2*G*cos(Theta2+Theta3)
%           ];
% A_theta = [A_Theta1;A_Theta2;A_Theta3];  
% F = M_theta * A_theta + V_theta_wtheta + G_theta;

%% 提取系数的方法整理，整理成一般形式，矩阵表达
M1_1 = Get_Formula(F1,A_Theta1);
M1_2 = Get_Formula(F1,A_Theta2);
M1_3 = Get_Formula(F1,A_Theta3);

M2_1 = Get_Formula(F2,A_Theta1);
M2_2 = Get_Formula(F2,A_Theta2);
M2_3 = Get_Formula(F2,A_Theta3);

M3_1 = Get_Formula(F3,A_Theta1);
M3_2 = Get_Formula(F3,A_Theta2);
M3_3 = Get_Formula(F3,A_Theta3);

M_theta = [M1_1 M1_2 M1_3;
           M2_1 M2_2 M2_3;
           M3_1 M3_2 M3_3
          ];

G1_1 = Get_Formula(F1,G)*G;
G2_1 = Get_Formula(F2,G)*G;
G3_1 = Get_Formula(F3,G)*G;

G_theta = [G1_1;
           G2_1;
           G3_1;
          ];

V1_1 = F1 - (M1_1*A_Theta1+M1_2*A_Theta2+M1_3*A_Theta3) - G1_1;
V2_1 = F2 - (M2_1*A_Theta1+M2_2*A_Theta2+M2_3*A_Theta3) - G2_1;
V3_1 = F3 - (M3_1*A_Theta1+M3_2*A_Theta2+M3_3*A_Theta3) - G3_1;


V_theta_wtheta = [V1_1;
                  V2_1;
                  V3_1
                 ];
A_theta = [A_Theta1;A_Theta2;A_Theta3];
F = M_theta * A_theta + V_theta_wtheta + G_theta;

% 验证推算的动力学方程与转换成矩阵形式后是否一致，作差的结果全为零则一致
% ans1 = simplify(F(1)-F1)
% ans2 = simplify(F(2)-F2)
% ans3 = simplify(F(3)-F3)

% 取含某项的系数的函数
function X = Get_Formula(f,x)
    temp = coeffs(f,x);
    n = length(temp);
    if n<=1
        X = 0;
    else
        for i = 2:n
            X = temp(i);
        end      
    end
end