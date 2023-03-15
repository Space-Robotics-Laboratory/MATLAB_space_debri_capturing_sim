% ロボット制御用クラス
% 
% 2023.3 akiyoshi uchida
% 
classdef Controller
    properties 
        phase
        pathway
        desVel
        tau
        controlMode
        pathUpFlag
    end
    methods
        % constructor 
        function obj = Controller(robo, startTime, controlMode)
            obj.pathway = Pathway(robo, startTime);
            obj.controlMode = controlMode;
            obj.tau = zeros(8, 1);
            obj.phase = 0;
            obj.desVel = zeros(6, 1);
            obj.pathUpFlag = false;
        end


        %%% コントロール関数
        function obj = control(obj, robo, targ, roboExtEst, time, param)
            switch obj.controlMode
                % 直接捕獲するケース
                case 1
                    obj.pathUpFlag = time == 0;
                    obj = obj.directCapture(robo, targ, roboExtEst, time, param);
                % 複数回接触によって減衰させてから捕獲するケース
                case 2
            end
        end

        %%% 以下，制御で用いる関数
        % 並進するターゲットを直接捕獲する制御
        function obj = directCapture(obj, robo, targ, roboExtEst, time, param)
            % フラグがたった時刻のみでpathwayを更新
            if obj.pathUpFlag
                goalPathway = obj.pathway.directCapture(targ, time, param);     % 目標位置時刻計算
                obj.pathway = obj.pathway.overWrite(robo, goalPathway, time);   % pathway更新
            end
            obj.desVel = obj.pathway.vel(time, 1);
            if obj.phase == -1
                obj.tau = calc_TauByVel(robo, zeros(6,1), roboExtEst, [true, true]);
            else
                obj.tau = calc_TauByVel(robo, obj.desVel, roboExtEst, [false, false]);
            end
            obj.phase = obj.pathway.phase(time, param);
        end

        % 並進するターゲットに移動している方向の手を当てる制御
        function obj = contactDampen(obj, robo, targ, roboExtEst, time, param)
            % フラグがたった時刻のみでpathwayを更新
            
        end
    end
end