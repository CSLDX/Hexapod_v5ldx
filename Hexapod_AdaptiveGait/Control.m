function C = Control
C.Body_walk_control = @Body_walk_control;
C.Body_balance_control = @Body_balance_control;
end

function [stepHeight,stepAmplitude,stepRotate,error_dis,error_rot] = Body_walk_control(dX,dY,dBody_rotate,rX,rY,rBody_rotate,lastX,lastY,lastBody_rotate,Amplitude_threshold,stepheight)
    %******************************************************************************
    % PID 控制器 单位：m   deg
    %******************************************************************************
    rBody_rotate = rad2deg(rBody_rotate);
    lastBody_rotate = rad2deg(lastBody_rotate);
    % 初始化输出
    stepRotate = [0 0 0 0 0 0];
    stepHeight = stepheight*ones(1,6);
    stepAmplitude = [0 0 0 0 0 0];
    rot1 = [0 0 0 0 0 0];
    rot2 = [0 0 0 0 0 0];
    Mini_rot = 0.3;
%     Amplitude_threshold = 0.055;
    % 参数
    % 角度pid参数
    Kp1 = 0.002; % 0.001
    Kd1 = 0.002; % 0.001
    % 距离pid参数
    Kp2 = 0.4;
    Kd2 = 0.5;
    % 误差
    error_x = dX - rX;
    error_y = dY - rY;
    error_dis = sqrt(error_x^2+error_y^2);
    % 角度表示范围是【-180° - 180°】因此误差大于180°时，减去360°，误差小于-180°时，加上360°
    error_rot = dBody_rotate - rBody_rotate;
    if error_rot > 180
        error_rot = error_rot - 360;
    elseif error_rot < -180
        error_rot = error_rot + 360;
    end
        
    last_error_x = dX - lastX;
    last_error_y = dY - lastY;
    last_error_dis = sqrt(last_error_x^2+last_error_y^2);
    % 角度表示范围是【-180° - 180°】因此误差大于180°时，减去360°，误差小于-180°时，加上360°
    last_error_rot = dBody_rotate - lastBody_rotate;
    if last_error_rot > 180
        last_error_rot = last_error_rot - 360;
    elseif last_error_rot < -180
        last_error_rot = last_error_rot + 360;
    end
    % 计算方向角
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
    
    % 计算步幅
    % 机身姿态小于一定阈值则不再调整，步幅设为0
    if abs(error_rot) > Mini_rot
        amp1 = Kp1*error_rot + Kd1*(error_rot - last_error_rot);
    else
        amp1 = 0;
    end
    amp2 = Kp2*error_dis + Kd2*(error_dis - last_error_dis);
    
    ampx = abs(amp1).*c1 + abs(amp2).*c2;
    ampy = abs(amp1).*s1 + abs(amp2).*s2;
    
    % 输出,限幅
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

function [body_rotatex_exp,body_rotatey_exp,body_height_exp,body_pos_exp,f] = Body_balance_control(footTip_pos,stepHeight, body_height, body_angles) 
    MT = MakeTrack;
    Turn_z = [0,0,-body_angles(3)];
    Turn_AbsoluteOrientation = MT.Trans_Matirx([0 0 0],Turn_z);
    
    body_pos_exp = [0 0];
    
    % 根据足端位置计算机身调整位姿的角度，计算斜率，转换成角度
    % 左侧
    k_xz1 = (footTip_pos(1,3) - footTip_pos(2,3))/(footTip_pos(1,1) - footTip_pos(2,1));
    k_xz2 = (footTip_pos(2,3) - footTip_pos(3,3))/(footTip_pos(2,1) - footTip_pos(3,1));
    % 右侧
    k_xz3 = (footTip_pos(6,3) - footTip_pos(5,3))/(footTip_pos(6,1) - footTip_pos(5,1));
    k_xz4 = (footTip_pos(5,3) - footTip_pos(4,3))/(footTip_pos(5,1) - footTip_pos(4,1));
    k_xz_front = mean([k_xz1 k_xz3]);
    k_xz_back = mean([k_xz2 k_xz4]);
    
    k_yz1 = (footTip_pos(6,3) - footTip_pos(1,3))/(footTip_pos(6,2) - footTip_pos(1,2));
    k_yz2 = (footTip_pos(5,3) - footTip_pos(2,3))/(footTip_pos(5,2) - footTip_pos(2,2));
    k_yz3 = (footTip_pos(4,3) - footTip_pos(3,3))/(footTip_pos(4,2) - footTip_pos(3,2));
    
    height_m = mean([footTip_pos(1,3) footTip_pos(2,3) footTip_pos(3,3) footTip_pos(4,3) footTip_pos(5,3) footTip_pos(6,3)]);
    height_s = min([footTip_pos(1,3) footTip_pos(2,3) footTip_pos(3,3) footTip_pos(4,3) footTip_pos(5,3) footTip_pos(6,3)]);
    height_std = std([footTip_pos(1,3) footTip_pos(2,3) footTip_pos(3,3) footTip_pos(4,3) footTip_pos(5,3) footTip_pos(6,3)]);

    Legpos_x = mean([footTip_pos(1,1) footTip_pos(2,1) footTip_pos(3,1) footTip_pos(4,1) footTip_pos(5,1) footTip_pos(6,1)]); % 足端xy平面上沿x轴的位置，机身应该处于中心
    Legpos_y = mean([footTip_pos(1,2) footTip_pos(2,2) footTip_pos(3,2) footTip_pos(4,2) footTip_pos(5,2) footTip_pos(6,2)]); % 足端xy平面上沿y轴的位置，机身应该处于中心

    
    
    % 依照规则赋值机身期望姿态角度
    Alpha = mean(atan([k_yz1 k_yz2 k_yz3]));
    Beta = mean(atan([k_xz_front k_xz_back]));
    
    body_rotatex = Alpha;
    body_rotatey = Beta;
    
    body_angles_impro = Turn_AbsoluteOrientation*[body_angles';1];
    body_rotate_impro = Turn_AbsoluteOrientation*[body_rotatex;body_rotatey;0;1];
    % 地形自适应规则
    if abs(rad2deg(body_rotate_impro(2)-body_angles_impro(2)))>3 % 俯仰角姿态调整
        body_rotatey_exp = (body_rotate_impro(2) + body_angles_impro(2));
        body_pos_exp(1) = 0;
        f1 = 1;
    else 
        body_rotatey_exp = (0 + body_angles_impro(2));
        body_pos_exp(1) = -(0+Legpos_x);
        f1 = 2;
    end
    if abs(rad2deg(body_rotate_impro(1)+body_angles_impro(1)))>3 % 翻滚角姿态调整
        body_rotatex_exp = -(body_rotate_impro(1) - body_angles_impro(1));
        body_pos_exp(2) = 0;
        f2 = 3;
    else 
        body_rotatex_exp = -(0 - body_angles_impro(1));
        body_pos_exp(2) = -(0+Legpos_y);
        f2 = 4;
    end
    

    % 身体姿态的转换矩阵
    % 期望机身坐标系相对当前机身坐标系的偏移和旋转
    % vrep读取姿态角度中，如果是相对于绝对坐标，就以绝对坐标系来判定翻滚、俯仰和偏航
    % 如果要完成类似于陀螺仪的姿态读取，在读取绝对坐标后要进行转换，相当于把绝对坐标转换成要读取的坐标系同向
    
    if height_std < 0.01 % 方差小于一定阈值
        body_heightz = height_s;
    else
        body_heightz = height_m;
    end
    
    body_height_exp = (body_height - body_heightz);
    
    f = [rad2deg(body_rotate_impro(1)+body_angles_impro(1)) rad2deg(body_rotate_impro(2)-body_angles_impro(2)) Legpos_x Legpos_y height_std f1 f2];
    
    
    
end


