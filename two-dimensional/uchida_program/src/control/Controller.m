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
        pathway         % Pathway class: 目標位置を扱うクラス
        desVel          % 目標速度 6*1
        tau             % 目標関節トルク 8*1 (ただし，受動関節はのちにバネモデルによって上書きされる)

        % 状態パラメータ
        controlMode     % 制御モード
        phase           % int  pathwayのどの段階にいるかの指標．現在向かっている目標が保存された目標位置の何番目の経由地かを示す．目標地点がない場合-1を持つ．
        phaseStarting   % bool pathwayのphaseの切り替えを示す．経由地点を初めて達成した時刻でtrue．
        pathUpdateFlag  % bool
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
            obj.waitT = .10;
        end


        %%% コントロール関数
        % 目標位置更新のフラグたてを行う．モードによって制御が異なる．
        function obj = control(obj, robo, targ, roboFTsensor, time, state, param)
            switch obj.controlMode
                % 直接捕獲するケース
                case 1
                    % 初期時刻にフラグをたてる
                    obj.pathUpdateFlag = time == 0 ;
                    obj = obj.directCapture(robo, targ, roboFTsensor, time, param);

                % 複数回接触によって減衰させてから捕獲するケース
                case 2
                    % 角速度がしきい値より小さくなったら直接捕獲に移行
                    if equal_time(state.time.comeTargetSlow + obj.waitT, time, param.DivTime)
                        obj.pathUpdateFlag = true;
                        obj = obj.directCapture(robo, targ, roboFTsensor, time, param);
                        obj.controlMode = 1;        % 速度制御の手法が変わる場合に注意．                 
                        return
                    end
                    % 初期時刻及び接触後待機時間経過後にフラグ立て
                    flagAfterContact = equal_time(time, state.time.lastContact + obj.waitT, param.DivTime) && obj.phase == -1;
                    obj.pathUpdateFlag = time == 0 || flagAfterContact;
                    obj =  obj.contactDampen(robo, targ, roboFTsensor, state, time, param);
            end
        end





        %%% 以下，制御で用いる関数
        % 関節トルクを計算する

        %% 並進するターゲットを直接捕獲する制御
        % obj.tauを更新する
        function obj = directCapture(obj, robo, targ, roboFTsensor, time, param)
            % フラグがたった時刻のみでpathwayを更新
            % 直接捕獲のための目標位置を設定する
            if obj.pathUpdateFlag
                obj.pathUpdateFlag = false;
                goalPathway = obj.pathway.directCapture(targ, time, param);     % 目標位置時刻計算
                obj.pathway = obj.pathway.overWriteGoal(robo, goalPathway, time);   % pathway更新
            end
            obj.desVel = obj.pathway.vel(time, robo, obj.gain, 1);
            if obj.phase == -1 
                % 目標時間の後，手先を相対停止
                obj.tau = calc_TauByVel(robo, zeros(6,1), roboFTsensor, [true, true]);
            else
                obj.tau = calc_TauByVel(robo, obj.desVel, roboFTsensor, [false, false]);
            end
            % 目標手先位置追従のどのフェーズにいるか計算
            obj.phase = obj.pathway.phase(time, param);
        end

        %% 並進するターゲットに移動している方向の手を当てる制御
        % obj.tauを更新する
        function obj = contactDampen(obj, robo, targ, roboFTsensor, state, time, param)
            % フラグがたった時刻のみでpathwayを更新
            if obj.pathUpdateFlag
                obj.pathUpdateFlag = false;
                goalPathway = obj.pathway.contactDampen(robo, targ, time, param);
                obj.pathway = obj.pathway.overWriteGoal(robo, goalPathway, time);   % pathway更新
            end
            if any(state.newContact) 
                % 接触時，初めの手先力に比例した速度で離れることによってインピーダンス制御を行う
                % 目標位置追従が完了していることを想定．このネスト内でphase~=-1の場合，想定していない接触．
                fource = [roboFTsensor(1:2, 1); roboFTsensor(6, 1); roboFTsensor(1:2, 2); roboFTsensor(6, 2)]; % 6*1
                obj.desVel = repmat(obj.gain.Cf, [2,1]) .* fource;
            elseif obj.phase ~= -1
                % 目標位置追従の途中
                % pahse==-1の時，目標手先位置はないので目標速度の更新はない
                obj.desVel = obj.pathway.vel(time, robo, obj.gain, 2);
            elseif obj.phase == -1 && time <= state.time.lastContact + obj.waitT*.8
                % 接触直後，フォローリリース的な動き
%                 obj.desVel = zeros(6,1);
%                 obj.tau = calc_TauByVel(robo, obj.desVel, roboExtEst, [true, true]);
%                 obj.phase = obj.pathway.phase(time, param);
%                 return
            elseif obj.phase == -1 && time > state.time.lastContact + obj.waitT*.8
                % 手先を逃して接触力を低減したのち，次にフラグが立つ前に速度０を代入
                obj.desVel = zeros(6,1);
            else
                error('Not Considered state')
            end
            obj.tau = calc_TauByVel(robo, obj.desVel, roboFTsensor, [false, false]);
            obj.phase = obj.pathway.phase(time, param);
        end
    end
end