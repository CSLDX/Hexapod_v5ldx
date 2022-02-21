function MT = MakeTrack
MT.Generate_track = @Generate_track;
MT.Generate_track2 = @Generate_track2;
MT.Generate_track3 = @Generate_track3;
MT.Output_leg_track = @Output_leg_track;
% MT.Output_track = @Output_track;
MT.Three_leg_gait = @Three_leg_gait;
MT.Four_leg_gait = @Four_leg_gait;
MT.Five_leg_gait = @Five_leg_gait;
MT.Adaptive_gait = @Adaptive_gait;
MT.New_adaptive_gait = @New_adaptive_gait;
MT.Trans_Matirx = @Trans_Matirx;
MT.Body2Body_trans = @Trans_Matirx;
MT.Leg2Body_trans = @Trans_Matirx;
end

%% 根据设置的节律步态生成具体的按照时间排列的足端轨迹离散坐标
% 输入：Generate_track类似
% 输出：具体足端的轨迹离散坐标、插值点数
% function [Gait_num, Leg_pos] = Output_track(stepHeight,stepAmplitude,stepRotate,Gait_flag)
%     % 标志量常量
%     ST = 0;
%     SW_UP = 1;
%     SW_DOWN = 2;
%     % 轨迹生成
%     [Gait_num, Swpos, Stpos] = Generate_track(stepHeight,stepAmplitude,stepRotate,Gait_flag);
%     % SwOrder = [腿1的摆动次序 腿2的摆动次序 腿3的摆动次序 腿4的摆动次序 腿5的摆动次序 腿6的摆动次序];
%     switch Gait_flag
%         case {3},   SwOrder = [1 2 1 2 1 2];
%         case {4},   SwOrder = [1 2 3 1 2 3];
%         case {5},   SwOrder = [1 2 3 4 5 6];
%         otherwise,  error('！！！步态模式错误！！！');
%     end
%     stepRotate = deg2rad(stepRotate);
%     numGroup = 6/(6-Gait_flag); % 动作组数
%     floorfactor = Gait_flag / 6; % 占地系数
%     Leg_pos = cell(6,1);
%    
%     
%     for leg = 1:6
%         % 步态，延时各腿足端轨迹
%         flag = mod(SwOrder(leg),numGroup);
%         if(flag == 1)
%             leg_z = [Swpos(3,:),Stpos(3,:)];
%             leg_y = [Swpos(2,:),Stpos(2,:)];
%             leg_x = [Swpos(1,:),Stpos(1,:)];
%             pos_state = [SW_UP*ones(1,sum(gradient(Swpos(3,:))>=0)),SW_DOWN*ones(1,sum(gradient(Swpos(3,:))<0)),ST*ones(1,length(Stpos(3,:)))];
%         else
%             if(flag == 0)
%                 leg_z = [Stpos(3,:),Swpos(3,:)];
%                 leg_y = [Stpos(2,:)-stepAmplitude*sin(stepRotate),Swpos(2,:)-stepAmplitude*sin(stepRotate)];
%                 leg_x = [Stpos(1,:)-stepAmplitude*cos(stepRotate),Swpos(1,:)-stepAmplitude*cos(stepRotate)];
%                 pos_state = [ST*ones(1,length(Stpos(3,:))),SW_UP*ones(1,sum(gradient(Swpos(3,:))>=0)),SW_DOWN*ones(1,sum(gradient(Swpos(3,:))<0))];
%             else
%                 leg_z = [Stpos(3,end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1)):end),Swpos(3,:),Stpos(3,1:end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1))-1)];
%                 leg_y = [Stpos(2,end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1)):end)-stepAmplitude*sin(stepRotate)*(flag-1)/(numGroup-1),...
%                          Swpos(2,:)-stepAmplitude*sin(stepRotate)*(flag-1)/(numGroup-1),...
%                          Stpos(2,1:end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1))-1)-stepAmplitude*sin(stepRotate)*(flag-1)/(numGroup-1)];
% 
%                 leg_x = [Stpos(1,end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1)):end)-stepAmplitude*cos(stepRotate)*(flag-1)/(numGroup-1),...
%                          Swpos(1,:)-stepAmplitude*cos(stepRotate)*(flag-1)/(numGroup-1),...
%                          Stpos(1,1:end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1))-1)-stepAmplitude*cos(stepRotate)*(flag-1)/(numGroup-1)];
%                 pos_state = [ST*ones(1,length(Stpos(3,end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1)):end))),...
%                              SW_UP*ones(1,sum(gradient(Swpos(3,:))>=0)),SW_DOWN*ones(1,sum(gradient(Swpos(3,:))<0)),...
%                              ST*ones(1,length(Stpos(3,1:end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1))-1)))];
%             end
%         end
%         % 一个步态周期的足端轨迹位置
%         Leg_pos{leg} = [leg_x; leg_y; leg_z; pos_state];
%     end
% end

%% 三足步态
% stepHeight,stepAmplitude,stepRotate为1*6的向量，代表每个腿
function [Gait_num, Leg_pos] = Three_leg_gait(stepHeight,stepAmplitude,stepRotate,track_type)
    Leg_pos = cell(6,1);
    for i = 1:6
        [Gait_num, leg_pos] = Output_leg_track(stepHeight(i),stepAmplitude(i),stepRotate(i),3,i,track_type);
        Leg_pos{i} = leg_pos;
    end
end
%% 四足步态
function [Gait_num, Leg_pos] = Four_leg_gait(stepHeight,stepAmplitude,stepRotate,track_type)
    Leg_pos = cell(6,1);
    for i = 1:6
        [Gait_num, leg_pos] = Output_leg_track(stepHeight(i),stepAmplitude(i),stepRotate(i),4,i,track_type);
        Leg_pos{i} = leg_pos;
    end
end
%% 五足步态
function [Gait_num, Leg_pos] = Five_leg_gait(stepHeight,stepAmplitude,stepRotate,track_type)
    Leg_pos = cell(6,1);
    for i = 1:6
        [Gait_num, leg_pos] = Output_leg_track(stepHeight(i),stepAmplitude(i),stepRotate(i),5,i,track_type);
        Leg_pos{i} = leg_pos;
    end
end
%% 自适应步态，修改足端轨迹
function [Gait_num, Leg_pos, pos_offset, ready2next, next_gait_num, Is_torch] = Adaptive_gait(Gait_num, Leg_pos, Is_collision, gait_num, last_pos_offset)
    % 标志量常量
    ST = 0;
    SW_UP = 1;
    SW_DOWN = 2;
    pos_offset = zeros(3,6);
    threshold = 0.001; % 1mm的阈值
    ready2next = 1; % 如果为1表示下落阶段的足端都已经触地，如果为0表示还有足端未落地
    Is_torch = ones(1,6);
    next_gait_num = 0;
    for i = 1:6
        % 足端提前遇到地面的情况
        % 修改当前步态周期的轨迹点
        if Is_collision(i) == 1  && Leg_pos{i,1}(4,gait_num) == SW_DOWN && Leg_pos{i,1}(3,gait_num) > threshold 
            pos_offset(3,i) = Leg_pos{i,1}(3,gait_num);
            pos_offset(2,i) = Leg_pos{i,1}(2,gait_num);
            pos_offset(1,i) = Leg_pos{i,1}(1,gait_num);
            % 修改当前步态周期，当前腿的轨迹点z坐标
            Leg_pos{i,1}(3,gait_num:end) = pos_offset(3,i);            
            % 执行下一步的条件
            ready2next = ready2next & 1;
        else
            ready2next = ready2next & 0;
        end
    end
    
    if ready2next == 1
        % 找到下一个要抬起的脚
        for t = gait_num:Gait_num
            for i = 1:6
                if Leg_pos{i,1}(4,t) == SW_UP
                    if t ~= Gait_num
                        next_gait_num = t;
                    end
                    break;
                end
            end
        end
    else
        % 判断落地的足端如果没有触地，执行新的下落周期
        for i = 1:6
            if Is_collision(i) == 0  && Leg_pos{i,1}(4,gait_num) == ST && gait_num > 1 && Leg_pos{i,1}(4,gait_num-1) == SW_DOWN 
                Is_torch(i) = 0;
            elseif Is_collision(i) == 0  && Leg_pos{i,1}(4,gait_num) == ST && gait_num == 1  
                Is_torch(i) = 0;
            elseif Is_collision(i) == 0  && Leg_pos{i,1}(4,gait_num) == SW_DOWN && gait_num == Gait_num  
                Is_torch(i) = 0;
            end
        end
    end
    
end


%% *********************************************************************************************************
% 内部的功能函数，外部不调用
%*********************************************************************************************************
%% 生成具体足端节律轨迹
function [Gait_num, Leg_pos] = Output_leg_track(stepHeight,stepAmplitude,stepRotate,Gait_flag,Leg,track_type)
    % 标志量常量  注意：不同设计的足端轨迹标注的上升段和下降段不一样！！
    % Generate_track函数生成的轨迹分为支撑相{ST}和摆动相，摆动相中z轴上升的标注为上升段{SW_UP}，z轴下降的标注为下降段{SW_DOWN}
    % Generate_track2函数生成的轨迹分为支撑相{ST}和摆动相，摆动相中z轴上升，xy轴不动（即垂直上升）的标注为上升段{SW_UP}，同理垂直下降的才标注为下降段{SW_DOWN},其余为摆动段{SW_SW}
    ST = 0;
    SW_UP = 1;
    SW_DOWN = 2;
    SW_SW = 3;
    % 轨迹生成
%     track_type = 2;
    switch track_type
        case {1}, [Gait_num, Swpos, Stpos] = Generate_track(stepHeight,stepAmplitude,stepRotate,Gait_flag);
        case {2}, [Gait_num, Swpos, Stpos] = Generate_track2(stepHeight,stepAmplitude,stepRotate,Gait_flag);
    end
    % SwOrder = [腿1的摆动次序 腿2的摆动次序 腿3的摆动次序 腿4的摆动次序 腿5的摆动次序 腿6的摆动次序];
    switch Gait_flag
        case {3},   SwOrder = [1 2 1 2 1 2];
        case {4},   SwOrder = [1 2 3 1 2 3];
        case {5},   SwOrder = [1 2 3 4 5 6];
        otherwise,  error('！！！步态模式错误！！！');
    end
    stepRotate = deg2rad(stepRotate);
    numGroup = 6/(6-Gait_flag); % 动作组数
    floorfactor = Gait_flag / 6; % 占地系数

    % 步态，延时足端轨迹
    flag = mod(SwOrder(Leg),numGroup);
    if(flag == 1)
        leg_z = [Swpos(3,:),Stpos(3,:)];
        leg_y = [Swpos(2,:),Stpos(2,:)];
        leg_x = [Swpos(1,:),Stpos(1,:)];
        switch track_type
            case {1}, pos_state = [SW_UP*ones(1,sum(gradient(Swpos(3,:))>=0)),SW_DOWN*ones(1,sum(gradient(Swpos(3,:))<0)),ST*ones(1,length(Stpos(3,:)))];
            case {2}, pos_state = [SW_UP*ones(1,sum(gradient(Swpos(3,:))>=0&gradient(Swpos(2,:))==0)),SW_SW*ones(1,sum(gradient(Swpos(2,:))~=0)),SW_DOWN*ones(1,sum(gradient(Swpos(3,:))<0&gradient(Swpos(2,:))==0)),ST*ones(1,length(Stpos(3,:)))];
        end
        
    else
        if(flag == 0)
            leg_z = [Stpos(3,:),Swpos(3,:)];
            leg_y = [Stpos(2,:)-stepAmplitude*sin(stepRotate),Swpos(2,:)-stepAmplitude*sin(stepRotate)];
            leg_x = [Stpos(1,:)-stepAmplitude*cos(stepRotate),Swpos(1,:)-stepAmplitude*cos(stepRotate)];  
            switch track_type
                case {1}, pos_state = [ST*ones(1,length(Stpos(3,:))),SW_UP*ones(1,sum(gradient(Swpos(3,:))>=0)),SW_DOWN*ones(1,sum(gradient(Swpos(3,:))<0))];
                case {2}, pos_state = [ST*ones(1,length(Stpos(3,:))),SW_UP*ones(1,sum(gradient(Swpos(3,:))>=0&gradient(Swpos(2,:))==0)),SW_SW*ones(1,sum(gradient(Swpos(2,:))~=0)),SW_DOWN*ones(1,sum(gradient(Swpos(3,:))<0&gradient(Swpos(2,:))==0))];
            end
        else
            leg_z = [Stpos(3,end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1)):end),Swpos(3,:),Stpos(3,1:end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1))-1)];
            leg_y = [Stpos(2,end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1)):end)-stepAmplitude*sin(stepRotate)*(flag-1)/(numGroup-1),...
                     Swpos(2,:)-stepAmplitude*sin(stepRotate)*(flag-1)/(numGroup-1),...
                     Stpos(2,1:end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1))-1)-stepAmplitude*sin(stepRotate)*(flag-1)/(numGroup-1)];
            leg_x = [Stpos(1,end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1)):end)-stepAmplitude*cos(stepRotate)*(flag-1)/(numGroup-1),...
                     Swpos(1,:)-stepAmplitude*cos(stepRotate)*(flag-1)/(numGroup-1),...
                     Stpos(1,1:end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1))-1)-stepAmplitude*cos(stepRotate)*(flag-1)/(numGroup-1)];
            switch track_type
                case {1}, pos_state = [ST*ones(1,length(Stpos(3,end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1)):end))),...
                                     SW_UP*ones(1,sum(gradient(Swpos(3,:))>=0&gradient(Swpos(2,:))==0)),SW_SW*ones(1,sum(gradient(Swpos(2,:))~=0)),SW_DOWN*ones(1,sum(gradient(Swpos(3,:))<0&gradient(Swpos(2,:))==0)),...
                                     ST*ones(1,length(Stpos(3,1:end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1))-1)))];
                case {2}, pos_state = [ST*ones(1,length(Stpos(3,end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1)):end))),...
                                     SW_UP*ones(1,sum(gradient(Swpos(3,:))>=0&gradient(Swpos(2,:))==0)),SW_SW*ones(1,sum(gradient(Swpos(2,:))~=0)),SW_DOWN*ones(1,sum(gradient(Swpos(3,:))<0&gradient(Swpos(2,:))==0)),...
                                     ST*ones(1,length(Stpos(3,1:end-floor(Gait_num*floorfactor*(flag-1)/(numGroup-1))-1)))];
            end
        end
    end
    % 一个步态周期的足端轨迹位置
    Leg_pos = [leg_x; leg_y; leg_z; pos_state];

end

%% 生成足端轨迹
% 输入：步幅、步高、行走方向、节律步态
% 输出：足端轨迹的离散坐标，支撑相和摆动相（x,y,z）、插值点数
% 内置：生成足端轨迹的函数（这里用的是正弦函数）
function [Gait_num, Swpos, Stpos] = Generate_track(stepHeight,stepAmplitude,stepRotate,Gait_flag)
    switch Gait_flag
        case {3}%,   disp('三足步态');
        case {4}%,   disp('四足步态');
        case {5}%,   disp('五足步态');
        otherwise,  error('！！！步态模式错误！！！');
    end
    stepRotate = deg2rad(stepRotate);
    floorfactor = Gait_flag / 6;    % 占地系数
    dutyfactor = 1 - floorfactor;   % 占空比
    ksw = round(floorfactor/dutyfactor);    % 放作摆动相的时间系数
    
    % 足端轨迹函数，离散化
    swigNum = 6;          % 单摆动周期插值点数
    stigNum = ksw*swigNum;  % 单支撑周期插值点数
    SwigTimeSeq = linspace(0,1, swigNum); % 单摆动周期的时间序列
    StigTimeSeq = linspace(0,1, stigNum);
    Gait_num = swigNum + stigNum;
    % 【单摆动周期内的正弦函数】  
    Swpos(1,:) = stepAmplitude*cos(stepRotate)*0.5*(1-cos(SwigTimeSeq.*pi));
    Swpos(2,:) = stepAmplitude*sin(stepRotate)*0.5*(1-cos(SwigTimeSeq.*pi));
    Swpos(3,:) = stepHeight*sin(SwigTimeSeq.*pi);
    % 【单支撑周期内的正弦函数】
    Stpos(1,:) = stepAmplitude*cos(stepRotate)*0.5*(1-cos(fliplr(StigTimeSeq).*pi));
    Stpos(2,:) = stepAmplitude*sin(stepRotate)*0.5*(1-cos(fliplr(StigTimeSeq).*pi));
    Stpos(3,:) = zeros(1, length(StigTimeSeq));


end
% 新的足端轨迹设计，分为上升段、滑动段、下落段，上升和下落只有z轴变换，下落段的轨迹点间隔比上升段密
function [Gait_num, Swpos, Stpos] = Generate_track2(stepHeight,stepAmplitude,stepRotate,Gait_flag)
    switch Gait_flag
        case {3}%,   disp('三足步态');
        case {4}%,   disp('四足步态');
        case {5}%,   disp('五足步态');
        otherwise,  error('！！！步态模式错误！！！');
    end
    stepRotate = deg2rad(stepRotate);
    floorfactor = Gait_flag / 6;    % 占地系数
    dutyfactor = 1 - floorfactor;   % 占空比
    ksw = round(floorfactor/dutyfactor);    % 放作摆动相的时间系数

    % 足端轨迹函数，离散化  4 6 10  //  4 2 2
    swigNum_sw = 4;          % 单摆动周期插值点数
    swigNum_up = 6;
    swigNum_down = 10;
    
    swigNum = swigNum_sw+swigNum_up+swigNum_down;
    stigNum = ksw*swigNum;  % 单支撑周期插值点数
    % 单摆动周期的时间序列
    SwigTimeSeq_sw = linspace(0,1, swigNum_sw); 
    SwigTimeSeq_up = linspace(0,1, swigNum_up);
    SwigTimeSeq_down = linspace(0,1, swigNum_down);
    % 单支撑周期的时间序列
%     StigTimeSeq = linspace(0,1, stigNum);
    Gait_num = swigNum + stigNum;
    % 【单摆动周期内的正弦函数】  
    Swpos(1,:) = [0*ones(1,swigNum_up), stepAmplitude*cos(stepRotate)*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*cos(stepRotate)*ones(1,swigNum_down)];
    Swpos(2,:) = [0*ones(1,swigNum_up), stepAmplitude*sin(stepRotate)*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*sin(stepRotate)*ones(1,swigNum_down)];
    Swpos(3,:) = [2/3*stepHeight*SwigTimeSeq_up, 2/3*stepHeight + 1/3*stepHeight*sin(SwigTimeSeq_sw.*pi), 2/3*stepHeight*fliplr(SwigTimeSeq_down)];
    % 【单支撑周期内的正弦函数】
    for i = 1:ksw
        if i == 1
            Stx = [stepAmplitude*cos(stepRotate)*(i-1)/ksw*ones(1,swigNum_down), stepAmplitude*cos(stepRotate)*i/ksw*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*cos(stepRotate)*i/ksw*ones(1,swigNum_up)];
            Sty = [stepAmplitude*sin(stepRotate)*(i-1)/ksw*ones(1,swigNum_down), stepAmplitude*sin(stepRotate)*i/ksw*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*sin(stepRotate)*i/ksw*ones(1,swigNum_up)];
        else
            Stx = [Stx, [stepAmplitude*cos(stepRotate)*(i-1)/ksw*ones(1,swigNum_down), stepAmplitude*cos(stepRotate)*i/ksw*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*cos(stepRotate)*i/ksw*ones(1,swigNum_up)]];
            Sty = [Sty, [stepAmplitude*sin(stepRotate)*(i-1)/ksw*ones(1,swigNum_down), stepAmplitude*sin(stepRotate)*i/ksw*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*sin(stepRotate)*i/ksw*ones(1,swigNum_up)]];
        end
    end
    Stpos(1,:) = fliplr(Stx);
    Stpos(2,:) = fliplr(Sty);
    Stpos(3,:) = zeros(1, stigNum);
end

% 相对的
% 输入的是期望机身坐标系相对当前机身坐标系的偏移和旋转
% （当前机身坐标系下坐标）= T*（期望机身坐标系下坐标）
% 输入的是腿相对与机身的位置偏移和旋转，
% （机身坐标系下坐标）= T*（腿坐标系下坐标）
function [T] = Trans_Matirx(offset,rot)
    trans_x = [1 0           0            offset(1);
               0 cos(rot(1)) -sin(rot(1)) 0;
               0 sin(rot(1)) cos(rot(1))  0;
               0 0           0            1
              ];
    trans_y = [cos(rot(2))  0 sin(rot(2))  0;
               0            1 0            offset(2);
               -sin(rot(2)) 0 cos(rot(2))  0;
               0            0 0            1
              ];
    trans_z = [cos(rot(3)) -sin(rot(3)) 0 0;
               sin(rot(3)) cos(rot(3))  0 0;
               0           0            1 offset(3);
               0           0            0 1
              ];
    % 得到的是机身坐标系下腿部构成的平面在y-z和x-z平面上的倾角，按照姿态角旋转的先后顺序不同，在俯仰和翻滚上回有不同的误差
    T = (trans_x*trans_y*trans_z+trans_y*trans_x*trans_z)/2;
    
end




