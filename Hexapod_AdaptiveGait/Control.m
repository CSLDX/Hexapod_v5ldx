function C = Control
C.Body_walk_control = @Body_walk_control;

end

function [stepHeight,stepAmplitude,stepRotate,error_dis,error_rot] = Body_walk_control(dX,dY,dBody_rotate,rX,rY,rBody_rotate,lastX,lastY,lastBody_rotate,Amplitude_threshold,stepheight)
    %******************************************************************************
    % PID ������ ��λ��m   deg
    %******************************************************************************
    rBody_rotate = rad2deg(rBody_rotate);
    lastBody_rotate = rad2deg(lastBody_rotate);
    % ��ʼ�����
    stepRotate = [0 0 0 0 0 0];
    stepHeight = stepheight*ones(1,6);
    stepAmplitude = [0 0 0 0 0 0];
    rot1 = [0 0 0 0 0 0];
    rot2 = [0 0 0 0 0 0];
    Mini_rot = 0.3;
%     Amplitude_threshold = 0.055;
    % ����
    % �Ƕ�pid����
    Kp1 = 0.002; % 0.001
    Kd1 = 0.002; % 0.001
    % ����pid����
    Kp2 = 0.4;
    Kd2 = 0.5;
    % ���
    error_x = dX - rX;
    error_y = dY - rY;
    error_dis = sqrt(error_x^2+error_y^2);
    % �Ƕȱ�ʾ��Χ�ǡ�-180�� - 180�㡿���������180��ʱ����ȥ360�㣬���С��-180��ʱ������360��
    error_rot = dBody_rotate - rBody_rotate;
    if error_rot > 180
        error_rot = error_rot - 360;
    elseif error_rot < -180
        error_rot = error_rot + 360;
    end
        
    last_error_x = dX - lastX;
    last_error_y = dY - lastY;
    last_error_dis = sqrt(last_error_x^2+last_error_y^2);
    % �Ƕȱ�ʾ��Χ�ǡ�-180�� - 180�㡿���������180��ʱ����ȥ360�㣬���С��-180��ʱ������360��
    last_error_rot = dBody_rotate - lastBody_rotate;
    if last_error_rot > 180
        last_error_rot = last_error_rot - 360;
    elseif last_error_rot < -180
        last_error_rot = last_error_rot + 360;
    end
    % ���㷽���
    if abs(error_rot) > Mini_rot
        if error_rot > 0
            rot1 = [180 180 180 0 0 0];
        else
            rot1 = [0 0 0 180 180 180];
        end
    end
    if error_x == 0
        rot2(:) = sign(error_y)*90 - rBody_rotate;
    elseif error_y == 0
        if error_x > 0 
            rot2(:) = 0 - rBody_rotate;
        else
            rot2(:) = 180 - rBody_rotate;
        end
    else 
        rot2(:) = rad2deg(atan2(error_y,error_x)) - rBody_rotate;
    end
    s1 = sind(rot1);
    c1 = cosd(rot1);
    s2 = sind(rot2);
    c2 = cosd(rot2);
    
    % ���㲽��
    % ������̬С��һ����ֵ���ٵ�����������Ϊ0
    if abs(error_rot) > Mini_rot
        amp1 = Kp1*error_rot + Kd1*(error_rot - last_error_rot);
    else
        amp1 = 0;
    end
    amp2 = Kp2*error_dis + Kd2*(error_dis - last_error_dis);
    
    ampx = abs(amp1).*c1 + abs(amp2).*c2;
    ampy = abs(amp1).*s1 + abs(amp2).*s2;
    
    % ���,�޷�
    for i = 1:6 
        if sqrt(ampx(i)^2+ampy(i)^2) > Amplitude_threshold
            stepAmplitude(i) = Amplitude_threshold;
        else
            stepAmplitude(i) = sqrt(ampx(i)^2+ampy(i)^2);
        end
        if ampx(i) == 0
            stepRotate(i) = sign(ampy(i))*90;
        elseif ampy(i) == 0
            if ampx(i) > 0 
                stepRotate(i) = 0;
            else
                stepRotate(i) = 180;
            end
        else 
            stepRotate(i) = rad2deg(atan2(ampy(i),ampx(i)));
        end
        
    end

end