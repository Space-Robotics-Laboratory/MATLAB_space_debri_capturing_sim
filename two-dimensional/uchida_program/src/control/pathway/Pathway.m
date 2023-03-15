% ロボット手先目標軌道を扱うクラス
%
% ロボット手先目標軌跡([pathwayLeft, pathwayRight]) 4*n*2初期化
% pathwayは[x(t), y(t), theta(t), t]'の形で，時刻tにおける座標を示す．
%
% pathwayは目標位置，その時の絶対時刻を合わせた行列．列0には初期位置，初期時刻を代入する.
% pathWay(1:3, n-1) -> pathWay(1:3, n) の移動を，pathWay(4, n-1) -> pathWay(4,
% n)秒で達成する.これをn-1回繰り返すことにより，経由地点を経た軌道を達成する．
% 
% ターゲットに対する処理に応じた目標位置を計算する関数をこのクラス内部で定義する
% 
% 2023.3 akiyoshi uchida
% 制御パラメータが散逸しているため，改善したい
% 

classdef Pathway
    properties
        pathway
        timeParam
    end
    methods
        %%%%% 基本関数 %%%%%
        % 初期化
        function obj = Pathway(robo, startTime)
            obj = obj.reset(robo, startTime);

            % 制御パラメータ
            obj.timeParam.move2targ = 1;    % directCapで，初めにターゲットに接触するまでの最小時間
            obj.timeParam.cap = .1;         % directCapで，接近後ターゲットを把持するのに要する時間
            obj.timeParam.contDmp = .5;     % contDampeで，ターゲットに接触するもでの時間
        end

        % pathwayをリセットする関数
        function obj = reset(obj, robo, time)
            % ロボット手先目標軌跡([pathwayLeft, pathwayRight]) 4*n*2初期化
            % pathwayは[x(t), y(t), theta(t), t]'の形で，時刻tにおける座標を示す．
            obj.pathway(:, 1, 1) = [robo.POS_e_L(1:2); robo.SV.QeL(3); time];   % pathwayLeft
            obj.pathway(:, 1, 2) = [robo.POS_e_R(1:2); robo.SV.QeR(3); time];   % pathwayRight
        end

        % pathwayを上書きする関数
        function obj = overWrite(obj, robo, newPathway, time)
            obj = obj.reset(robo, time);
            obj.pathway = [obj.pathway, newPathway];
        end

        % pathwayを上書きせずに追加する関数
        function obj = addEnd(obj, newPathway)
            obj.pathway = [obj.pathway, newPathway];
        end

        % pathwayのどのフェーズにいるか計算
        function [phase, phaseStarting, phaseEnding] = phase(obj, time, param)
            [~, n, ~] = size(obj.pathway);
            index = (obj.pathway(4, :, 1) <= time) & (obj.pathway(4, [2:n, 1], 1) > time);  % 時刻をもとに経路のフェーズを判定
            % 時刻(pathway(4,:)が順番通りでない場合，エラーを排出する
            if sum(index) >= 2                                                           
                error("you have to set pathway in order of time")
            end
            
            % pathwayの初めの時刻より前であれば（通常負の時刻），phaseは０
            if time < obj.pathway(4, 1, 1)
                phase = 0;
                phaseStarting = (time - param.MinusTime) < param.DivTime * 1.01 && ...
                                (time - param.MinusTime) >= 0;
                phaseEnding = (obj.pathway(4, 1, 1) - time) <= param.DivTime * 1.01 && ...
                              (obj.pathway(4, 1, 1) - time) > 0;
                return
            
            % pathwayの最後の項より後であれば，phaseは-1
            elseif time >= obj.pathway(4, n, 1)
                phase = -1;
                phaseStarting = (time - obj.pathway(4, n, 1)) < param.DivTime * 1.01 && ...
                                (time - obj.pathway(4, n, 1)) >= 0;
                phaseEnding = true;
                return
            end
            IND = 1:n;
            phase = IND(index);
            phaseStarting = (time - obj.pathway(4, phase, 1)) < param.DivTime * 1.01 && ... 
                            (time - obj.pathway(4, phase, 1)) >= 0; % おそらく余分
            phaseEnding = (obj.pathway(4, phase + 1, 1) - time) <= param.DivTime * 1.01 && ...
                          (obj.pathway(4, phase + 1, 1) - time) > 0; % おそらく余分
        end

        % pathwayから速度計算
        function desiredVel = vel(obj, time, mode)
            desiredVel(1:3, 1) = calc_Vel(obj.pathway(:, :, 1), time, mode);
            desiredVel(4:6, 1) = calc_Vel(obj.pathway(:, :, 2), time, mode);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%% pathway計算関数 %%%%%
        %%% 呼び出された時刻から並進するターゲットに非接触を保ち，その後把持する位置と時刻を設定
        % ターゲットの頂点が如何なる角度においても手先に接触しない(ちょうど触れる)
        function goalPathway = directCapture(obj, targ, time, param)
            % ターゲット情報代入
            targPos = targ.SV.R0(1:2, :);
            targV = targ.SV.v0(1:2, :);
            targW = targ.SV.w0(3);
            targWidth = targ.width;
            targOri = targ.SV.Q0(3);


            % ターゲット頂点がロボットエンドエフェクタの間に入る角度
            alpha = asin(param.LdH * sin(param.LdGamma) / (targWidth/sqrt(2) + param.LdD * .5));
            desTargOri = pi * .25 - sign(targW) * alpha;
            
            % 待機時間計算
            deltTime = rem(desTargOri - targOri, pi * .5) / targW;
            while(deltTime < obj.timeParam.move2targ) % 時間が小さすぎる場合，回転対称性から時刻を伸ばす
                deltTime = deltTime + pi * .5 / abs(targW);
            end

            %%% 目標手先位置代入
            % ターゲット頂点に接触しないギリギリの時刻・位置を追従するフェーズ
            targGoalPos = targPos + targV * deltTime;                                           % 接触時点のターゲット重心位置 2*1
            armSign = [-1, +1];                                                                 % 左手なら負，右手なら正
            goalPathway = zeros(4, 1, 2);                                                       % 目標手先位置初期化 4*1*2
            dX_close = sqrt( (targWidth/sqrt(2) + param.LdD*.5)^2 ...
                          -(param.LdH * sin(param.LdGamma))^2   ) ;                             % 手先先端球がターゲットにギリギリ触れない，ターゲット中心から手先までの距離
            dX_capture = (targWidth + param.LdD)/sqrt(2) - param.LdH * sin(param.LdGamma);      % 手先先端球がターゲットに触れる時の，ターゲット中心から手先までの距離


            contactPosVecClose = [dX_close; 0]  .* armSign ;                                    % 接触アーム目標位置の，ターゲット重心に対する相対位置ベクトル 2*2
            contactPosVecCapt  = [dX_capture; 0] .* armSign;
            goalPathway(:, 1, 1) = [targGoalPos + contactPosVecClose(:,1);-pi/2; deltTime + time]; % 左アーム目標位置代入
            goalPathway(:, 2, 1) = [targGoalPos + contactPosVecCapt(:,1) ;-pi/2; deltTime + time + obj.timeParam.cap];
            goalPathway(:, 1, 2) = [targGoalPos + contactPosVecClose(:,2); pi/2; deltTime + time]; % 右アーム目標位置代入
            goalPathway(:, 2, 2) = [targGoalPos + contactPosVecCapt(:,2) ; pi/2; deltTime + time + obj.timeParam.cap];
        end

        %%% ターゲットが並進してきている方向の手先を接触させることによりターゲット角速度を減衰させる
        function goalPathway = contactDampen(obj, targ, time, param)
            contactPos = .8;
            contactAng = 0;
            evadeAng = deg2rad(0);

            % ターゲット情報代入
            targPos = targ.SV.R0(1:2, :);
            targV = targ.SV.v0(1:2, :);
            targW = targ.SV.w0(3);
            targWidth = targ.width;
            targOri = targ.SV.Q0(3);
            gamma = param.LdGamma;

            % ターゲット頂点がロボットエンドエフェクタの間から出る角度
            beta = asin(targWidth * .5 * contactPos / (targWidth/sqrt(2) + param.LdD * .5));
            desTargOri = pi * .25 + sign(targW) * beta;
            
            % 待機時間計算
            deltTime = rem(desTargOri - targOri, pi * .5) / targW;
            while(deltTime < obj.timeParam.contDamp) % 時間が小さすぎる場合，回転対称性から時刻を伸ばす
                deltTime = deltTime + pi * .5 / abs(targW);
            end

            %%% 目標手先位置代入
            % ターゲット頂点に接触しないギリギリの時刻・位置を追従するフェーズ
            targGoalPos = targPos + targV * deltTime;                                           % 接触時点のターゲット重心位置 2*1
            armSign = [-1, +1];                                                                 % 左手なら負，右手なら正
            goalPathway = zeros(4, 1, 2);                                                       % 目標手先位置初期化 4*1*2
            dX_close = (targWidth/sqrt(2) + param.LdD * .5) * cos(beta) ...
                        + param.LdH * sin(gamma) * sin(evadeAng);                             % 手先先端球がターゲットにギリギリ触れない，ターゲット中心から手先までの距離
            dY_close = param.LdH * sin(gamma) * cos(evadeAng);
            dX_cont = (targWidth + param.LdD)/sqrt(2) - param.LdH * sin(gamma);      % 手先先端球がターゲットに触れる時の，ターゲット中心から手先までの距離


            contactPosVecClose = [dX_close; 0]  .* armSign ;                                    % 接触アーム目標位置の，ターゲット重心に対する相対位置ベクトル 2*2
            contactPosVecCont  = [dX_cont; 0] .* armSign;
            goalPathway(:, 1, 1) = [targGoalPos + contactPosVecClose(:,1);-pi/2; deltTime + time]; % 左アーム目標位置代入
            goalPathway(:, 2, 1) = [targGoalPos + contactPosVecCont(:,1) ;-pi/2; deltTime + time + obj.timeParam.cap];
            goalPathway(:, 1, 2) = [targGoalPos + contactPosVecClose(:,2); pi/2; deltTime + time]; % 右アーム目標位置代入
            goalPathway(:, 2, 2) = [targGoalPos + contactPosVecCont(:,2) ; pi/2; deltTime + time + obj.timeParam.cap];        
        end

    end
end