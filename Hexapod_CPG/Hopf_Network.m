% 腿间的环形耦合网络的Hopf振荡器
% tips:
% 腿间耦合项的产生稳定相位差所需的时间与收敛系数和初值参数选取有关
% 初值参数的选取可能会导致耦合项无效的问题，尤其体现在三角步态上
function OUTPUT=Hopf_Network(t,X)
global gait_state; %步态模式选择


a=1000;      %收敛系数，越大收敛速度越快
u=1;         %振荡器幅值的平方

A=100;       %较大的正数，决定频率切换速度
Wsw=12*pi;    %摆动相频率
C=0.3;       %腿间耦合强度

if gait_state == 3
    B=1/2;       %占地系数，摆动相和支撑相 1/2  2/3  5/6
    %三角步态相位差
    phase = {pi,-pi,pi,-pi,pi,-pi,...
            pi,-pi,pi,-pi,pi,-pi,...
            0,0,0,0,0,0}; 
elseif gait_state == 4
    B=2/3;       %占地系数，摆动相和支撑相 1/2  2/3  5/6
    %四足步态相位差
    phase = {4/3*pi,-2/3*pi,-2/3*pi,4/3*pi,-2/3*pi,-2/3*pi,...
            2/3*pi,2/3*pi,-4/3*pi,2/3*pi,2/3*pi,-4/3*pi,...
            0,0,0,0,0,0};
elseif gait_state == 5
    B=5/6;       %占地系数，摆动相和支撑相 1/2  2/3  5/6
    %五足步态相位差
    phase = {2/3*pi,2/3*pi,1/3*pi,-2/3*pi,-2/3*pi,-1/3*pi,...
            1/3*pi,2/3*pi,2/3*pi,-1/3*pi,-2/3*pi,-2/3*pi,...
            0,0,0,0,0,0}; 
end

% 振荡器频率
W1=(((1-B)/B)*Wsw)/(exp(-1*A*X(2))+1)+(Wsw/(exp(A*X(2))+1));
W2=(((1-B)/B)*Wsw)/(exp(-1*A*X(4))+1)+(Wsw/(exp(A*X(4))+1));
W3=(((1-B)/B)*Wsw)/(exp(-1*A*X(6))+1)+(Wsw/(exp(A*X(6))+1));
W4=(((1-B)/B)*Wsw)/(exp(-1*A*X(8))+1)+(Wsw/(exp(A*X(8))+1));
W5=(((1-B)/B)*Wsw)/(exp(-1*A*X(10))+1)+(Wsw/(exp(A*X(10))+1));
W6=(((1-B)/B)*Wsw)/(exp(-1*A*X(12))+1)+(Wsw/(exp(A*X(12))+1));
    
r1=X(1)^2+X(2)^2;
r2=X(3)^2+X(4)^2;
r3=X(5)^2+X(6)^2;
r4=X(7)^2+X(8)^2;
r5=X(9)^2+X(10)^2;
r6=X(11)^2+X(12)^2;


% 腿间的相位差    
[th12,th23,th34,th45,th56,th61,...
 th16,th65,th54,th43,th32,th21,...
 th11,th22,th33,th44,th55,th66]=phase{:};

% 环形耦合网络
 G=[cos(th11),cos(th21),0,0,0,cos(th61);
    cos(th12),cos(th22),cos(th32),0,0,0;
    0,cos(th23),cos(th33),cos(th43),0,0;
    0,0,cos(th34),cos(th44),cos(th54),0;
    0,0,0,cos(th45),cos(th55),cos(th65);
    cos(th16),0,0,0,cos(th56),cos(th66)];

 M=[sin(th11),sin(th21),0,0,0,sin(th61);
    sin(th12),sin(th22),sin(th32),0,0,0;
    0,sin(th23),sin(th33),sin(th43),0,0;
    0,0,sin(th34),sin(th44),sin(th54),0;
    0,0,0,sin(th45),sin(th55),sin(th65);
    sin(th16),0,0,0,sin(th56),sin(th66)];

Gy=G*[X(2);X(4);X(6);X(8);X(10);X(12)];
Mx=M*[X(1);X(3);X(5);X(7);X(9);X(11)];


FF = [a*(u-r1)*X(1)-W1*X(2);%腿1 x(1) x(2)
      a*(u-r1)*X(2)+W1*X(1)+C*(Gy(1)-Mx(1));
      a*(u-r2)*X(3)-W2*X(4);%腿2 x(3) x(4)
      a*(u-r2)*X(4)+W2*X(3)+C*(Gy(2)-Mx(2));
      a*(u-r3)*X(5)-W3*X(6);%腿3 x(5) x(6)
      a*(u-r3)*X(6)+W3*X(5)+C*(Gy(3)-Mx(3));
      a*(u-r4)*X(7)-W4*X(8);%腿4 x(7) x(8)
      a*(u-r4)*X(8)+W4*X(7)+C*(Gy(4)-Mx(4));
      a*(u-r5)*X(9)-W5*X(10);%腿5 x(9) x(10)
      a*(u-r5)*X(10)+W5*X(9)+C*(Gy(5)-Mx(5));
      a*(u-r6)*X(11)-W6*X(12);%腿6 x(11) x(12)
      a*(u-r6)*X(12)+W6*X(11)+C*(Gy(6)-Mx(6))];

OUTPUT=FF;

end