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

%% �������õĽ��ɲ�̬���ɾ���İ���ʱ�����е���˹켣��ɢ����
% ���룺Generate_track����
% �����������˵Ĺ켣��ɢ���ꡢ��ֵ����
% function [Gait_num, Leg_pos] = Output_track(stepHeight,stepAmplitude,stepRotate,Gait_flag)
%     % ��־������
%     ST = 0;
%     SW_UP = 1;
%     SW_DOWN = 2;
%     % �켣����
%     [Gait_num, Swpos, Stpos] = Generate_track(stepHeight,stepAmplitude,stepRotate,Gait_flag);
%     % SwOrder = [��1�İڶ����� ��2�İڶ����� ��3�İڶ����� ��4�İڶ����� ��5�İڶ����� ��6�İڶ�����];
%     switch Gait_flag
%         case {3},   SwOrder = [1 2 1 2 1 2];
%         case {4},   SwOrder = [1 2 3 1 2 3];
%         case {5},   SwOrder = [1 2 3 4 5 6];
%         otherwise,  error('��������̬ģʽ���󣡣���');
%     end
%     stepRotate = deg2rad(stepRotate);
%     numGroup = 6/(6-Gait_flag); % ��������
%     floorfactor = Gait_flag / 6; % ռ��ϵ��
%     Leg_pos = cell(6,1);
%    
%     
%     for leg = 1:6
%         % ��̬����ʱ������˹켣
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
%         % һ����̬���ڵ���˹켣λ��
%         Leg_pos{leg} = [leg_x; leg_y; leg_z; pos_state];
%     end
% end

%% ���㲽̬
% stepHeight,stepAmplitude,stepRotateΪ1*6������������ÿ����
function [Gait_num, Leg_pos] = Three_leg_gait(stepHeight,stepAmplitude,stepRotate,track_type)
    Leg_pos = cell(6,1);
    for i = 1:6
        [Gait_num, leg_pos] = Output_leg_track(stepHeight(i),stepAmplitude(i),stepRotate(i),3,i,track_type);
        Leg_pos{i} = leg_pos;
    end
end
%% ���㲽̬
function [Gait_num, Leg_pos] = Four_leg_gait(stepHeight,stepAmplitude,stepRotate,track_type)
    Leg_pos = cell(6,1);
    for i = 1:6
        [Gait_num, leg_pos] = Output_leg_track(stepHeight(i),stepAmplitude(i),stepRotate(i),4,i,track_type);
        Leg_pos{i} = leg_pos;
    end
end
%% ���㲽̬
function [Gait_num, Leg_pos] = Five_leg_gait(stepHeight,stepAmplitude,stepRotate,track_type)
    Leg_pos = cell(6,1);
    for i = 1:6
        [Gait_num, leg_pos] = Output_leg_track(stepHeight(i),stepAmplitude(i),stepRotate(i),5,i,track_type);
        Leg_pos{i} = leg_pos;
    end
end
%% ����Ӧ��̬���޸���˹켣
function [Gait_num, Leg_pos, pos_offset, ready2next, next_gait_num, Is_torch] = Adaptive_gait(Gait_num, Leg_pos, Is_collision, gait_num, last_pos_offset)
    % ��־������
    ST = 0;
    SW_UP = 1;
    SW_DOWN = 2;
    pos_offset = zeros(3,6);
    threshold = 0.001; % 1mm����ֵ
    ready2next = 1; % ���Ϊ1��ʾ����׶ε���˶��Ѿ����أ����Ϊ0��ʾ�������δ���
    Is_torch = ones(1,6);
    next_gait_num = 0;
    for i = 1:6
        % �����ǰ������������
        % �޸ĵ�ǰ��̬���ڵĹ켣��
        if Is_collision(i) == 1  && Leg_pos{i,1}(4,gait_num) == SW_DOWN && Leg_pos{i,1}(3,gait_num) > threshold 
            pos_offset(3,i) = Leg_pos{i,1}(3,gait_num);
            pos_offset(2,i) = Leg_pos{i,1}(2,gait_num);
            pos_offset(1,i) = Leg_pos{i,1}(1,gait_num);
            % �޸ĵ�ǰ��̬���ڣ���ǰ�ȵĹ켣��z����
            Leg_pos{i,1}(3,gait_num:end) = pos_offset(3,i);            
            % ִ����һ��������
            ready2next = ready2next & 1;
        else
            ready2next = ready2next & 0;
        end
    end
    
    if ready2next == 1
        % �ҵ���һ��Ҫ̧��Ľ�
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
        % �ж���ص�������û�д��أ�ִ���µ���������
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
% �ڲ��Ĺ��ܺ������ⲿ������
%*********************************************************************************************************
%% ���ɾ�����˽��ɹ켣
function [Gait_num, Leg_pos] = Output_leg_track(stepHeight,stepAmplitude,stepRotate,Gait_flag,Leg,track_type)
    % ��־������  ע�⣺��ͬ��Ƶ���˹켣��ע�������κ��½��β�һ������
    % Generate_track�������ɵĹ켣��Ϊ֧����{ST}�Ͱڶ��࣬�ڶ�����z�������ı�עΪ������{SW_UP}��z���½��ı�עΪ�½���{SW_DOWN}
    % Generate_track2�������ɵĹ켣��Ϊ֧����{ST}�Ͱڶ��࣬�ڶ�����z��������xy�᲻��������ֱ�������ı�עΪ������{SW_UP}��ͬ��ֱ�½��Ĳű�עΪ�½���{SW_DOWN},����Ϊ�ڶ���{SW_SW}
    ST = 0;
    SW_UP = 1;
    SW_DOWN = 2;
    SW_SW = 3;
    % �켣����
%     track_type = 2;
    switch track_type
        case {1}, [Gait_num, Swpos, Stpos] = Generate_track(stepHeight,stepAmplitude,stepRotate,Gait_flag);
        case {2}, [Gait_num, Swpos, Stpos] = Generate_track2(stepHeight,stepAmplitude,stepRotate,Gait_flag);
    end
    % SwOrder = [��1�İڶ����� ��2�İڶ����� ��3�İڶ����� ��4�İڶ����� ��5�İڶ����� ��6�İڶ�����];
    switch Gait_flag
        case {3},   SwOrder = [1 2 1 2 1 2];
        case {4},   SwOrder = [1 2 3 1 2 3];
        case {5},   SwOrder = [1 2 3 4 5 6];
        otherwise,  error('��������̬ģʽ���󣡣���');
    end
    stepRotate = deg2rad(stepRotate);
    numGroup = 6/(6-Gait_flag); % ��������
    floorfactor = Gait_flag / 6; % ռ��ϵ��

    % ��̬����ʱ��˹켣
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
    % һ����̬���ڵ���˹켣λ��
    Leg_pos = [leg_x; leg_y; leg_z; pos_state];

end

%% ������˹켣
% ���룺���������ߡ����߷��򡢽��ɲ�̬
% �������˹켣����ɢ���֧꣬����Ͱڶ��ࣨx,y,z������ֵ����
% ���ã�������˹켣�ĺ����������õ������Һ�����
function [Gait_num, Swpos, Stpos] = Generate_track(stepHeight,stepAmplitude,stepRotate,Gait_flag)
    switch Gait_flag
        case {3}%,   disp('���㲽̬');
        case {4}%,   disp('���㲽̬');
        case {5}%,   disp('���㲽̬');
        otherwise,  error('��������̬ģʽ���󣡣���');
    end
    stepRotate = deg2rad(stepRotate);
    floorfactor = Gait_flag / 6;    % ռ��ϵ��
    dutyfactor = 1 - floorfactor;   % ռ�ձ�
    ksw = round(floorfactor/dutyfactor);    % �����ڶ����ʱ��ϵ��
    
    % ��˹켣��������ɢ��
    swigNum = 6;          % ���ڶ����ڲ�ֵ����
    stigNum = ksw*swigNum;  % ��֧�����ڲ�ֵ����
    SwigTimeSeq = linspace(0,1, swigNum); % ���ڶ����ڵ�ʱ������
    StigTimeSeq = linspace(0,1, stigNum);
    Gait_num = swigNum + stigNum;
    % �����ڶ������ڵ����Һ�����  
    Swpos(1,:) = stepAmplitude*cos(stepRotate)*0.5*(1-cos(SwigTimeSeq.*pi));
    Swpos(2,:) = stepAmplitude*sin(stepRotate)*0.5*(1-cos(SwigTimeSeq.*pi));
    Swpos(3,:) = stepHeight*sin(SwigTimeSeq.*pi);
    % ����֧�������ڵ����Һ�����
    Stpos(1,:) = stepAmplitude*cos(stepRotate)*0.5*(1-cos(fliplr(StigTimeSeq).*pi));
    Stpos(2,:) = stepAmplitude*sin(stepRotate)*0.5*(1-cos(fliplr(StigTimeSeq).*pi));
    Stpos(3,:) = zeros(1, length(StigTimeSeq));


end
% �µ���˹켣��ƣ���Ϊ�����Ρ������Ρ�����Σ�����������ֻ��z��任������εĹ켣��������������
function [Gait_num, Swpos, Stpos] = Generate_track2(stepHeight,stepAmplitude,stepRotate,Gait_flag)
    switch Gait_flag
        case {3}%,   disp('���㲽̬');
        case {4}%,   disp('���㲽̬');
        case {5}%,   disp('���㲽̬');
        otherwise,  error('��������̬ģʽ���󣡣���');
    end
    stepRotate = deg2rad(stepRotate);
    floorfactor = Gait_flag / 6;    % ռ��ϵ��
    dutyfactor = 1 - floorfactor;   % ռ�ձ�
    ksw = round(floorfactor/dutyfactor);    % �����ڶ����ʱ��ϵ��

    % ��˹켣��������ɢ��  4 6 10  //  4 2 2
    swigNum_sw = 4;          % ���ڶ����ڲ�ֵ����
    swigNum_up = 6;
    swigNum_down = 10;
    
    swigNum = swigNum_sw+swigNum_up+swigNum_down;
    stigNum = ksw*swigNum;  % ��֧�����ڲ�ֵ����
    % ���ڶ����ڵ�ʱ������
    SwigTimeSeq_sw = linspace(0,1, swigNum_sw); 
    SwigTimeSeq_up = linspace(0,1, swigNum_up);
    SwigTimeSeq_down = linspace(0,1, swigNum_down);
    % ��֧�����ڵ�ʱ������
%     StigTimeSeq = linspace(0,1, stigNum);
    Gait_num = swigNum + stigNum;
    % �����ڶ������ڵ����Һ�����  
    Swpos(1,:) = [0*ones(1,swigNum_up), stepAmplitude*cos(stepRotate)*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*cos(stepRotate)*ones(1,swigNum_down)];
    Swpos(2,:) = [0*ones(1,swigNum_up), stepAmplitude*sin(stepRotate)*0.5*(1-cos(SwigTimeSeq_sw.*pi)), stepAmplitude*sin(stepRotate)*ones(1,swigNum_down)];
    Swpos(3,:) = [2/3*stepHeight*SwigTimeSeq_up, 2/3*stepHeight + 1/3*stepHeight*sin(SwigTimeSeq_sw.*pi), 2/3*stepHeight*fliplr(SwigTimeSeq_down)];
    % ����֧�������ڵ����Һ�����
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

% ��Ե�
% �������������������ϵ��Ե�ǰ��������ϵ��ƫ�ƺ���ת
% ����ǰ��������ϵ�����꣩= T*��������������ϵ�����꣩
% ������������������λ��ƫ�ƺ���ת��
% ����������ϵ�����꣩= T*��������ϵ�����꣩
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
    % �õ����ǻ�������ϵ���Ȳ����ɵ�ƽ����y-z��x-zƽ���ϵ���ǣ�������̬����ת���Ⱥ�˳��ͬ���ڸ����ͷ����ϻ��в�ͬ�����
    T = (trans_x*trans_y*trans_z+trans_y*trans_x*trans_z)/2;
    
end




