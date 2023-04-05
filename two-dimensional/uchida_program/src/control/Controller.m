% ロボット制御用クラス
% 
% 目標速度，目標関節トルク，目標位置等を保持し，ロボットに目標関節トルクを与える．
% 目標位置の計算に関しては，幾何的で煩雑な式となることからPathwayクラスを要素として持ち，その内部で行う．
% 
% 2023.3 akiyoshi uchida
% 
classdef Controller
    properties 
        % 制御用設定パラメータ
        gain            % 制御ゲイン
        waitT           % contactDompen内で接触後待機する時間

        % 制御目標パラメータ
        pathway
        desVel
        tau

        % 状態パラメータ
        controlMode
        phase
        phaseStarting
        pathUpdateFlag
    end
    methods
        % constructor 
        function obj = Controller(robo, startTime, controlMode)
            obj.pathway = Pathway(robo, startTime);     % 目標手先位置・時間
            obj.controlMode = controlMode;              % 制御モード
            obj.tau = zeros(8, 1);                      % 関節入力トルク
            obj.phase = 0;                              % 目標手先位置(pathway)追従において，どの段階にいるかを示す
            obj.desVel = zeros(6, 1);                   % 目標手先速度
            obj.pathUpdateFlag = false;                 % 目標手先位置を更新するためのフラグ

            % パラメータ設定
            obj.gain.Ck = [0, 0, 0]';%[.01, .01, .01]'; % 目標位置に関するPD制御比例ゲイン
            obj.gain.Cd = [0, 0, 0]';%[.1, .1, .1]';    % 目標位置に関するPD制御微分ゲイン
            obj.gain.Cf = [0.04, 0.04, 0.0]';%[.1, .1, 0.1]';                % 手先外力に対する目標速度ゲイン（インピーダンス制御）
            obj.waitT = .1;
        end


        %%% コントロール関数
        % mainの制御はここでおこなっている
        function obj = control(obj, robo, targ, roboExtEst, time, state, param)
            switch obj.controlMode
                % 直接捕獲するケース
                case 1
                    % 初期時刻にフラグをたてる
                    obj.pathUpdateFlag = time == 0 ;
                    obj = obj.directCapture(robo, targ, roboExtEst, time, param);

                % 複数回接触によって減衰させてから捕獲するケース
                case 2
                    % 角速度がしきい値より小さくなったら直接捕獲に移行
                    if abs(targ.SV.w0(3) ) < 0 && abs(targ.SV.R0(1)) < .2
                        obj.pathUpdateFlag = true;
                        obj = obj.directCapture(robo, targ, roboExtEst, time, param);
                        return
                    end
                    % 初期時刻及び接触後待機時間経過後にフラグ立て
                    flagAfterContact = equal_time(time, state.time.lastContact + obj.waitT, param.DivTime) && obj.phase == -1;
                    obj.pathUpdateFlag = time == 0 || flagAfterContact; %time == state.time.lastContact + obj.waitT;;
                    obj =  obj.contactDampen(robo, targ, roboExtEst, state, time, param);
            end
        end





        %%% 以下，制御で用いる関数
        % 関節トルクを計算する

        %% 並進するターゲットを直接捕獲する制御
        function obj = directCapture(obj, robo, targ, roboExtEst, time, param)
            % フラグがたった時刻のみでpathwayを更新
            % 直接捕獲のための目標位置を設定する
            if obj.pathUpdateFlag
                obj.pathUpdateFlag = false;
                goalPathway = obj.pathway.directCapture(robo, targ, time, param);     % 目標位置時刻計算
                obj.pathway = obj.pathway.overWriteGoal(robo, goalPathway, time);   % pathway更新
            end
            obj.desVel = obj.pathway.vel(time, robo, obj.gain, 2);
            if obj.phase == -1 
                % 目標時間の後，手先を相対停止
                obj.tau = calc_TauByVel(robo, zeros(6,1), roboExtEst, [true, true]);
            else
                obj.tau = calc_TauByVel(robo, obj.desVel, roboExtEst, [false, false]);
            end
            % 目標手先位置追従のどのフェーズにいるか計算
            obj.phase = obj.pathway.phase(time, param);
        end

        %% 並進するターゲットに移動している方向の手を当てる制御
        function obj = contactDampen(obj, robo, targ, roboExtEst, state, time, param)
            % フラグがたった時刻のみでpathwayを更新
            if obj.pathUpdateFlag
                obj.pathUpdateFlag = false;
                goalPathway = obj.pathway.contactDampen(robo, targ, time, param);
                obj.pathway = obj.pathway.overWriteGoal(robo, goalPathway, time);   % pathway更新
            end
            if any(state.newContact)
                % 接触時，初めの手先力に比例した速度で離れることによってインピーダンス制御を行う
                % 目標位置追従が完了していることを想定．このネスト内でphase~=-1の場合，想定していない接触．
                fource = [roboExtEst(1:2, 2); roboExtEst(3, 2); roboExtEst(1:2, 3); roboExtEst(6, 3)]; % 6*1
                obj.desVel = repmat(obj.gain.Cf, [2,1]) .* fource;
            elseif obj.phase ~= -1
                % 目標位置追従の途中
                % pahse==-1の時，目標手先位置はないので目標速度の更新はない
                obj.desVel = obj.pathway.vel(time, robo, obj.gain, 1);
            elseif obj.phase == -1 && time > state.time.lastContact + obj.waitT*.8
                % 手先を逃して接触力を低減したのち，次にフラグが立つ前に速度０を代入
                obj.desVel = zeros(6,1);
            elseif obj.phase == -1 && time <= state.time.lastContact + obj.waitT*.8
                % 接触直後，フォローリリース的な動き
%                 obj.desVel = zeros(6,1);
            else
                error('Not Considered state')
            end
            obj.tau = calc_TauByVel(robo, obj.desVel, roboExtEst, [false, false]);
            obj.phase = obj.pathway.phase(time, param);
        end
    end
end