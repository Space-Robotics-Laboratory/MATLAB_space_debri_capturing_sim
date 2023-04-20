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
        impDuration     % 接触後のインピーダンス制御持続時間
        switchingDelay  % 複数回接触から直接捕獲へと切り替える待ち時間
        mi              % imp 仮装質量
        di              % imp ダンパ特性
        ki              % imp バネ特性
        dp              % 'str_fb'におけるfeedback微分ゲイン
        kp              % 'str_fb'におけるfeedback比例ゲイン

        % 制御目標パラメータ
        pathway         % Pathway class: 目標位置を扱うクラス
        desEEvel        % 目標速度 6*1
        desEEacc        % 目標加速度 6*1
        deltPosLike     % 目標位置からの変位 6*1 (by integrate)
        velInBaseFram   % 目標速度の表現フレーム
        velocityMode    % 目標位置から目標速度を計算するモード
        tau             % 目標関節トルク 8*1 (ただし，受動関節はのちにバネモデルによって上書きされる)

        % 制御状態パラメータ
        controllerMode  % 制御モード
        impedanceMode   % 反力低減モード
        controlState    % string 制御状態．目標位置追従や反力低減を区別する. sub機能内での判別には用いない（追うのが大変）
        phase           % int  pathwayのどの段階にいるかの指標．現在向かっている目標が保存された目標位置の何番目の経由地かを示す．目標地点がない場合-1を持つ
        phaseStarting   % bool pathwayのphaseの切り替えを示す．経由地点を初めて達成した時刻でtrue．
        flagPathUpdate  % bool pathwayをupdateするためのフラグ
        controlArm      % bool 1*2 制御の対象となる腕

        % シミュレーションパラメータ
        dt              % 刻み時間
    end
    methods
        % constructor 
        function obj = Controller(robo, startTime, controlMode, param)
            obj.pathway = Pathway(robo, startTime);     % 目標手先位置・時間
            obj.controllerMode = controlMode;           % 制御モード
            obj.tau = zeros(8, 1);                      % 関節入力トルク
            obj.phase = 0;                              % 目標手先位置(pathway)追従において，どの段階にいるかを示す
            obj.desEEvel = zeros(6, 1);                 % 目標手先速度
            obj.desEEacc = zeros(6, 1);                 % 目標手先加速度
            obj.deltPosLike = zeros(6, 1);              % 目標位置からの変位
            obj.velInBaseFram = [false, false];         % 手先速度がベースで表現されているかどうか
            obj.velocityMode = 'str_tru';               % 目標位置から目標速度を計算するモード 
            obj.flagPathUpdate = false;                 % 目標手先位置を更新するためのフラグ
            obj.controlState = 'follow';                % 関節速度が０
            obj.impedanceMode = 'addmitance';           % インピーダンス制御のモード
            obj.controlArm = [false, false];            % 積極的に制御する対象となるアーム

            % パラメータ設定
            obj.gain.Ck = param.control.kp;             % 目標位置に関するPD制御比例ゲイン
            obj.gain.Cd = param.control.dp;             % 目標位置に関するPD制御微分ゲイン
            obj.mi = repmat(param.control.mi, [2,1]);
            obj.di = repmat(param.control.di, [2,1]);
            obj.ki = repmat(param.control.ki, [2,1]);
            obj.dp = param.control.dp;
            obj.kp = param.control.kp;
            obj.impDuration = param.control.impedanceDuration;
            obj.switchingDelay = param.control.swichingDelay2Direct;
        end


        %%% コントロール関数
        % 目標位置更新のフラグたてを行う．モードによって制御が異なる．
        function obj = control(obj, robo, targ, roboFTsensor, time, state, param)
            switch obj.controllerMode
                % 直接捕獲するケース
                case 'DIRECT'
                    % 初期時刻にフラグをたてる
                    obj.flagPathUpdate = time == 0 ;
                    obj = obj.directCapture(robo, targ, roboFTsensor, time, param);
                    return

                % 複数回接触によって減衰させてから捕獲するケース
                case 'MULTIPLE'
                    % 角速度がしきい値より小さくなったら直接捕獲に移行
                    if time > state.time.comeTargetSlow + obj.impDuration
                        % 少し時間をおいてから直接捕獲のフラグを立てる
                        obj.flagPathUpdate = equal_time(time, state.time.comeTargetSlow+obj.switchingDelay, param.DivTime);    
                        obj = obj.directCapture(robo, targ, roboFTsensor, time, param);
                        return
                    end
                    % 初期時刻及び接触後待機時間経過後にフラグ立て
                    flagAfterContact = equal_time(time, state.time.lastContact + obj.impDuration, param.DivTime) && obj.phase == -1;
                    obj.flagPathUpdate = time == 0 || flagAfterContact;
                    obj =  obj.contactDampen(robo, targ, roboFTsensor, state, time, param);
                    return 
                
                % 手先半力低減制御を確かめるためのテスト
                % ターゲット回転なし，左側に並進を想定
                case 'TEST'
                    % 初期時刻にフラグをたてる
                    obj.flagPathUpdate = time == 0 ;
                    if obj.flagPathUpdate
                        obj.flagPathUpdate = false;
                        obj.velInBaseFram = [false, false];
                        goalPathway = obj.pathway.testImpedanceNoRotate(targ, time);     % 目標位置時刻計算
                        obj.pathway = obj.pathway.overWriteGoal(robo, goalPathway, time);   % pathway更新
                    end
                    % インピーダンス割り込み処理
                    obj = obj.impedance(time, robo, roboFTsensor, state, param);
                    if obj.phase ~= -1 && strcmp(obj.controlState, 'follow')
                        obj = obj.followPathway(time, robo, roboFTsensor);
                    end
                    obj.phase = obj.pathway.phase(time, param);
                    return 
            end
            error('No such control mode')
        end




        %%% 状況ごとの細かい制御
        %% 並進するターゲットを直接捕獲する制御
        function obj = directCapture(obj, robo, targ, roboFTsensor, time, param)
            % フラグがたった時刻のみでpathwayを更新
            % 直接捕獲のための目標位置を設定する
            if obj.flagPathUpdate
                obj.flagPathUpdate = false;
                obj.velInBaseFram = [false, false];
                obj.pathway = obj.pathway.directCapture(robo, targ, time, param);     % 目標位置時刻計算
            end

            if obj.phase == -1 % 目標手先位置を持たない
                % 目標時間の後，手先を相対停止
                obj = obj.stopEndEffector(robo, roboFTsensor);
            else
                % 両手でpathway(目標位置)を追従
                obj.controlArm = [true, true];
                obj = obj.followPathway(time, robo, roboFTsensor);
            end
            % 目標手先位置追従のどのフェーズにいるか計算
            obj.phase = obj.pathway.phase(time, param);
        end

        %% 並進するターゲットに移動している方向の手を当てる制御
        % obj.tauを更新する
        function obj = contactDampen(obj, robo, targ, roboFTsensor, state, time, param)
            % フラグがたった時刻のみでpathwayを更新
            if obj.flagPathUpdate
                obj.flagPathUpdate = false;
                obj.velInBaseFram = [false, false];
                obj.pathway = obj.pathway.contactDampen(robo, targ, time, param);
            end
            % インピーダンス割り込み処理
            obj = obj.impedance(time, robo, roboFTsensor, state, param);
            if obj.phase ~= -1 && strcmp(obj.controlState, 'follow')
                % 片手でpathway(目標位置)を追従
                obj.controlArm = [targ.SV.v0(1)<0, targ.SV.v0(1)>=0];
                obj = obj.followPathway(time, robo, roboFTsensor);
            end
            obj.phase = obj.pathway.phase(time, param);
        end


        %%% 以下sub機能
        %% 手先をベースに対して固定する
        function obj = stopEndEffector(obj, robo, roboFTsensor)
            obj.desEEvel = zeros(6, 1);
            obj.velInBaseFram = [true, true];
            obj.controlArm = [false, false];
            obj.controlState = 'freeze';
            obj.tau = calc_TauByVel(robo, obj.desEEvel, roboFTsensor, obj.velInBaseFram);
        end

        %% pathwayを追従する
        function obj = followPathway(obj, time, robo, roboFTsensor)
            obj.velInBaseFram = [false, false];
            obj.controlState = 'follow';
            obj.desEEvel = obj.pathway.vel(time, robo, obj);
            obj.tau = calc_TauByVel(robo, obj.desEEvel, roboFTsensor, obj.velInBaseFram);
        end
        
        %% モードに応じてインピーダンス制御をして手先半力を低減する
        function obj = impedance(obj, time, robo, roboFTsensor, state, param)
            switch obj.impedanceMode
                % 初めの手先力に比例して速度制御
                case 'proportional' 
                    obj.velInBaseFram = [false, false];
                    % 新たに接触した瞬間の力に比例した速度を計算
                    if any(state.isContact)
                        force = [roboFTsensor(1:2, 1); roboFTsensor(6, 1); roboFTsensor(1:2, 2); roboFTsensor(6, 2)]; % 6*1
                        obj.desEEvel = obj.di .* force;
                        obj.tau = calc_TauByVel(robo, obj.desEEvel, roboFTsensor, obj.velInBaseFram);
                        obj.controlState = 'imp';
                    % 規定時間経過まで計算した速度を維持
                    elseif time >= state.time.lastContact && time < state.time.lastContact + obj.impDuration * .8
                        obj.tau = calc_TauByVel(robo, obj.desEEvel, roboFTsensor, obj.velInBaseFram);
                        obj.controlState = 'imp';
                    elseif time >= state.time.lastContact + obj.impDuration * .8 && time < state.time.lastContact + obj.impDuration
                        obj = obj.stopEndEffector(robo, roboFTsensor);
                        obj.controlState = 'imp';
                    else
                        obj.controlState = 'follow';
                    end
                    return
                case 'addmitance'
                    obj.velInBaseFram = [false, false];
                    if time >= state.time.lastContact && time < state.time.lastContact + obj.impDuration * .8
                        force = [roboFTsensor(1:2, 1); roboFTsensor(6, 1); roboFTsensor(1:2, 2); roboFTsensor(6, 2)]; % 6*1
                        % バネマスダンパモデルによって目標加速度計算
                        obj.desEEacc = force ./ obj.mi - obj.desEEvel .* obj.di ./ obj.mi - obj.deltPosLike .* obj.ki ./ obj.mi;
                        obj.desEEvel = obj.desEEvel + obj.desEEacc * param.DivTime; % 直接制御に用いるのはこの値
                        obj.deltPosLike = obj.deltPosLike + obj.desEEvel * param.DivTime;
                        obj.tau = calc_TauByVel(robo, obj.desEEvel, roboFTsensor, obj.velInBaseFram);
                        obj.controlState ='imp';
                    elseif time >= state.time.lastContact + obj.impDuration * .8 && time < state.time.lastContact + obj.impDuration
                        obj = obj.stopEndEffector(robo, roboFTsensor);
                    else
                        obj.controlState = 'follow';
%                         obj.deltPosLike = zeros(6, 1);
                    end
                    return
            end
            error('No such impedance mode')
        end
    end
end