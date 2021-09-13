function [gait_num, leg_pos, flag] = walk_path(rX,rY,rBody_rotate,lastX,lastY,lastBody_rotate,flag)
    C = Control;
    MT = MakeTrack;
    Amplitude_threshold = 0.05;
    stepheight = 0.035;
    switch flag
        case {1} 
            tar_x = 0.57139;
            tar_y = -0.94721;
            tar_rot = -180;
        case {2}
            tar_x = 0.076;
            tar_y = -0.87721;
            tar_rot = -90;
        case {3}
            tar_x = -2.8086;
            tar_y = -0.87721;
            tar_rot = -90;
        case {4}
            tar_x = -2.8086;
            tar_y = -1.0882;
            tar_rot = 0;
        case {5}
            tar_x = -2.8086;
            tar_y = -2.0412;
            tar_rot = 0;
        case {6}
            tar_x = -1.0956;
            tar_y = -2.0412;
            stepheight = 0.05;
            Amplitude_threshold = 0.054;
            tar_rot = 0;
        case {7}
            tar_x = -1.0956;
            tar_y = -2.0412;
            stepheight = 0.02;
            Amplitude_threshold = 0.04;
            tar_rot = -170;
        otherwise
            tar_x = -1;
            tar_y = -2.0412;
            stepheight = 0.03;
            Amplitude_threshold = 0.04;
            tar_rot = 0;
            flag = 0;
%             disp('done')
    end
%     switch flag
%         case {1} 
%             tar_x = -1.0956;
%             tar_y = -1.9672;
%             tar_rot = 0;
%         otherwise
%             tar_x = -1.0956;
%             tar_y = -1.9672;
%             tar_rot = 0;
%             flag = 0;
% %             disp('done')  
%     end
    [stepHeight,stepAmplitude,stepRotate,error_dis,error_rot] = C.Body_walk_control(tar_x, tar_y, tar_rot,rX,rY,rBody_rotate,lastX,lastY,lastBody_rotate,Amplitude_threshold,stepheight);
    [gait_num, leg_pos] = MT.Three_leg_gait(stepHeight,stepAmplitude,stepRotate);
    if error_dis < 0.5 && error_rot < 10 && flag ~= 0
        flag = flag + 1;
    end
end