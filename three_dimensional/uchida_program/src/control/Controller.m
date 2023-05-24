% ロボット制御用クラス
% 
% 目標速度，目標関節トルク，目標位置等を保持し，ロボットに目標関節トルクを与える．
% 目標位置の計算に関しては，幾何的で煩雑な式となることからPathwayクラスを要素として持ち，その内部で行う．
%
% 
% 2023.3 akiyoshi uchida
% 
classdef Controller
    properties 
        % 制御用設定パラメータ
        impDuration     % 接触後のインピーダンス制御持続時間
        switchingDelay  % 複数回接触から直接捕獲へと切り替える待ち時間
        mi              % imp 仮装質量
        di              % imp ダンパ特性
        ki              % imp バネ特性
        controlMode     % 制御モード
        impedanceMode   % 反力低減モード

        % 制御目標パラメータ
        pathway_L       % Pathway class: 目標位置を扱うクラス
        pathway_R       % Pathway class: 目標位置を扱うクラス
        desBaseVel      % ベース目標速度 3*1 [vx,vy,wz]'
        desEEVel        % 目標速度 6*1
        desEEAcc        % 目標加速度 6*1
        deltPosLike     % 目標位置からの変位 6*1 (by integrate)
        vel_in_baseFram % 目標速度の表現フレーム
        velocityMode    % 目標位置から目標速度を計算するモード
        tau             % 目標関節トルク 8*1 (ただし，受動関節はのちにバネモデルによって上書きされる)

        % 制御状態パラメータ
        controlState    % string 制御状態．目標位置追従や反力低減を区別する. sub機能内での判別には用いない（追うのが大変）
        phase           % int  pathwayのどの段階にいるかの指標．現在向かっている目標が保存された目標位置の何番目の経由地かを示す．目標地点がない場合-1を持つ
        phaseStarting   % bool pathwayのphaseの切り替えを示す．経由地点を初めて達成した時刻でtrue．
        flagPathUpdate  % bool pathwayをupdateするためのフラグ
        controlPart     % bool 1*3 [base, leftArm, rigthArm] 制御の対象となる物体
        vel_in_control  % bool 6*3 [base 6*1, leftArm 6*1, eightArm 6*1] 積極的に制御する対象となる速度

        % シミュレーションパラメータ
        dt              % 刻み時間
    end
    methods
        % constructor 
        function obj = Controller(robo, startTime, param)
            obj.pathway_L = Pathway(robo, startTime, 'L');      % 目標手先位置・時間
            obj.pathway_R = Pathway(robo, startTime, 'R');      % 目標手先位置・時間
            obj.controlMode = param.control.controlMode;        % 制御モード
            obj.tau = zeros(12, 1);                             % 関節入力トルク
            obj.phase = 0;                                      % 目標手先位置(pathway)追従において，どの段階にいるかを示す
            obj.desBaseVel = zeros(6, 1);                       % 目標ベース速度
            obj.desEEVel = zeros(12, 1);                        % 目標手先速度
            obj.desEEAcc = zeros(12, 1);                        % 目標手先加速度
            obj.deltPosLike = zeros(12, 1);                     % 目標位置からの変位
            obj.vel_in_baseFram = [false, false];               % 手先速度がベースで表現されているかどうか
            obj.velocityMode = param.control.velocityMode;      % 目標位置から目標速度を計算するモード 
            obj.flagPathUpdate = false;                         % 目標手先位置を更新するためのフラグ
            obj.controlState = 'follow';                        % 関節速度が０
            obj.impedanceMode = param.control.impedanceMode;    % インピーダンス制御のモード
            obj.controlPart = [false, false, false];            % 積極的に制御する対象となる物体
            obj.vel_in_control = false(6, 3);                   % 積極的に制御する対象となる速度

            % パラメータ設定
            obj.mi = repmat(param.control.mi, [2,1]);
            obj.di = repmat(param.control.di, [2,1]);
            obj.ki = repmat(param.control.ki, [2,1]);
            obj.impDuration = param.control.impedanceDuration;
            obj.switchingDelay = param.control.swichingDelay2Direct;
        end


        %%% コントロール関数
        % 目標位置更新のフラグたてを行う．モードによって制御が異なる．
        function obj = control(obj, robo, targ, roboFTsensor, time, state, param)
            switch obj.controlMode
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
                    flagAfterContact = equal_time(time, state.time.lastContact + obj.impDuration, param.DivTime);% && obj.phase == -1;
                    obj.flagPathUpdate = time == 0 || flagAfterContact;
                    obj =  obj.contactDampen(robo, targ, roboFTsensor, state, time, param);
                    return 
                
                % 手先半力低減制御を確かめるためのテスト
                % ターゲット回転なし，左側に並進を想定
                case 'TEST1'
                    flagAfterContact = equal_time(time, state.time.lastContact + obj.impDuration, param.DivTime);
                    obj.flagPathUpdate = time == 0 || flagAfterContact;
                    if obj.flagPathUpdate
                        newPath_L = [-.08*sqrt(2)-.08, 0.35, 0, 0, 0, -pi/2, time+.2;
                                     -.08*sqrt(2)-.03 , 0.35, 0, pi/3, 0, -pi/2, time+.4]';
                        newPath_R = [ .08*sqrt(2)+.08, 0.35, 0, 0, 0,  pi/2, time+.2;
                                      .08*sqrt(2)+.03 , 0.35, 0, 0, 0,  pi/2, time+.4]';
                        obj.pathway_L = obj.pathway_L.overWriteGoal(robo, newPath_L, time);
                        obj.pathway_R = obj.pathway_R.overWriteGoal(robo, newPath_R, time);
                    end
                    if obj.phase == -1
                        obj = obj.stopEndEffector(robo, roboFTsensor);
                        return
                    end
                    % obj = obj.impedanceInterrupt(time, robo, roboFTsensor, state, param);
                    if strcmp(obj.controlState, 'follow')
                        obj.desBaseVel = zeros(6, 1);
                        obj.desEEVel(1:6 , 1) = obj.pathway_L.vel(time, robo, param);
                        obj.desEEVel(7:12, 1) = obj.pathway_R.vel(time, robo, param);
                        obj.vel_in_control = [false(6,1), true(6,1), true(6,1)];
                        obj = obj.achieveVelocity(robo, roboFTsensor);
                    end
                    obj.phase = obj.pathway_L.phase(time, param);
                    return 

                case 'TEST2'
                    obj.flagPathUpdate = time == 0 ;
                    if obj.flagPathUpdate
                        newPath_L = [0, 0, 0, 0, 0, -pi/2, 2]';
                        newPath_R = [0.24, 0.46, 0.08*sqrt(2)+.036, 0, 0,  pi/2, 1;
                                     0, 0.35, 0.08*sqrt(2)+.036, 0, 0,  pi/2, 1.3;
                                     0, 0.35, 0.08*sqrt(2)+.036, pi/6, 0,  pi/2, 1.5;
                                     0, 0.35, 0.08*sqrt(2)+.033, pi/6, 0,  pi/2, 1.7]';
                        obj.pathway_L = obj.pathway_L.keepPosition(robo, time, 10);
                        obj.pathway_R = obj.pathway_R.addEnd(newPath_R);
                    end
                    obj.desBaseVel = zeros(6, 1);
                    obj.desEEVel(1:6 , 1) = obj.pathway_L.vel(time, robo, param);
                    obj.desEEVel(7:12, 1) = obj.pathway_R.vel(time, robo, param);
                    obj.vel_in_control = [false(6,1), true(6,1), true(6,1)];
                    obj = obj.impedanceInterrupt(time, robo, roboFTsensor, state, param);
                    if strcmp(obj.controlState, 'follow')
                        obj = obj.achieveVelocity(robo, roboFTsensor);
                    end
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
                obj.vel_in_baseFram = [false, false];
                obj.pathway_L = obj.pathway_L.directCapture(robo, targ, time, param);     % 目標位置時刻計算
            end

            if obj.phase == -1 % 目標手先位置を持たない
                % 目標時間の後，手先を相対停止
                obj = obj.stopEndEffector(robo, roboFTsensor);
            else
                % 両手でpathway(目標位置)を追従
                obj.vel_in_control = [false(6,1), true(6,1), true(6,1)];
                obj = obj.followPathway_2hands(time, robo, roboFTsensor);
            end
            % 目標手先位置追従のどのフェーズにいるか計算
            obj.phase = obj.pathway_L.phase(time, param, 1);
        end

        %% 並進するターゲットに移動している方向の手を当てる制御
        % obj.tauを更新する
        function obj = contactDampen(obj, robo, targ, roboFTsensor, state, time, param)
            % フラグがたった時刻のみでpathwayを更新
            if obj.flagPathUpdate
                obj.flagPathUpdate = false;
                obj.vel_in_baseFram = [false, false];
                obj.pathway_L = obj.pathway_L.contactDampen(robo, targ, time, param);
            end
            % インピーダンス割り込み処理
            obj = obj.impedanceInterrupt(time, robo, roboFTsensor, state, param);
            if obj.phase ~= -1 && strcmp(obj.controlState, 'follow')
                % 両手でpathway(目標位置)を追従
                obj.controlPart = [true, targ.SV.v0(1)<0, targ.SV.v0(1)>=0];
                obj.desEEVel = obj.pathway_L.vel(time, robo, obj);
                obj.desBaseVel = [0; targ.SV.v0(2); 0];
                obj = obj.achieveVelocity(robo, roboFTsensor);
                % obj = obj.followPathway_2hands(time, robo, roboFTsensor);
            end
            obj.phase = obj.pathway_L.phase(time, param, 1);
        end

        %%% 以下sub機能
        %% 手先をベースに対して固定する
        function obj = stopEndEffector(obj, robo, roboFTsensor)
            obj.desEEVel = zeros(12, 1);
            obj.vel_in_baseFram = [true, true];
            obj.vel_in_control = [false(6,1), false(6,1), false(6,1)];
            obj.controlState = 'freeze';
            obj.tau = calc_tau_from_ee_vel(robo, obj.desEEVel, roboFTsensor, obj.vel_in_baseFram);
        end

        %% pathwayを，両手で同時に追従する．
        function obj = followPathway_2hands(obj, time, robo, roboFTsensor, param)
            obj.vel_in_baseFram = [false, false];
            obj.controlState = 'follow';
            obj.desEEVel(1:6,  :) = obj.pathway_L.vel(time, robo, param);
            obj.desEEVel(7:12, :) = obj.pathway_R.vel(time, robo, param);
            obj.tau = calc_tau_from_ee_vel(robo, obj.desEEVel, roboFTsensor, obj.vel_in_baseFram);
        end

        %% 目標速度にarmのみでなくbaseもとり，任意の速度を入力として制御する
        function obj = achieveVelocity(obj, robo, roboFTsensor)
            obj.controlState = 'follow';
            allPartVel = [obj.desBaseVel; obj.desEEVel];
            obj.tau = calc_tau_from_ee_base_vel(robo, allPartVel, roboFTsensor, obj.vel_in_control);
        end
        
        %% モードに応じてインピーダンス制御をして手先半力を低減する
        function obj = impedanceInterrupt(obj, time, robo, roboFTsensor, state, param)
            switch obj.impedanceMode
                case 'addmitance'
                    % 接触時に制御対象を接触アームに設定
                    if any(state.isContact)
                        obj.vel_in_baseFram = [false, false];
                        % インピーダンス制御に新たに入った時に初期化
                        if ~strcmp(obj.controlState, 'imp')
                            obj.desEEVel = zeros(12, 1);
                        end
                        part = [false, any(state.isContact(1:3)), any(state.isContact(4:6))]; % 1*3
                        obj.vel_in_control = repmat(part, [6, 1]);
                    end

                    % 接触直後に衝撃吸収
                    if time >= state.time.lastContact && time < state.time.lastContact + obj.impDuration * .8
                        force = reshape(roboFTsensor, [12, 1]); % 6*1
                        % バネマスダンパモデルによって目標加速度計算
                        obj.desEEAcc = force ./ obj.mi - obj.desEEVel .* obj.di ./ obj.mi - obj.deltPosLike .* obj.ki ./ obj.mi;
                        obj.desEEVel = obj.desEEVel + obj.desEEAcc * param.DivTime; % 直接制御に用いるのはこの値
                        obj.deltPosLike = obj.deltPosLike + obj.desEEVel * param.DivTime;
                        obj.tau = calc_tau_from_ee_vel(robo, obj.desEEVel, roboFTsensor, obj.vel_in_baseFram);
                        obj.controlState ='imp';

                    % 規定時間経過後にアームストップ
                    elseif time >= state.time.lastContact + obj.impDuration * .8 && time < state.time.lastContact + obj.impDuration
                        obj = obj.stopEndEffector(robo, roboFTsensor);
                    else

                    % インピーダンス制御終了
                        obj.controlState = 'follow';
                    end
                    return
            end
            error('No such impedance mode')
        end
    end
end