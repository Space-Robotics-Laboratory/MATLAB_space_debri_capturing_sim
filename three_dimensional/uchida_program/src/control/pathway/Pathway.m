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
% ターゲットのベースに対する相対速度で制御する方法に変更すべき？
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
            obj.timeParam.cap = .15;         % directCapで，接近後ターゲットを把持するのに要する時間
            obj.timeParam.contDmp = .5;     % contDampeで，ターゲットに接触するまでの時間
            obj.timeParam.maxWait = 1;      % directCapでの最大待ち時間
        end

        % pathwayをリセットする関数
        function obj = reset(obj, robo, time)
            % ロボット手先目標軌跡([pathwayLeft, pathwayRight]) 4*n*2初期化
            % pathwayは[x(t), y(t), theta(t), t]'の形で，時刻tにおける座標を示す．
            obj.pathway = zeros(4, 1, 2);
            obj.pathway(:, 1, 1) = [robo.POS_e_L(1:2); robo.SV.QeL(3); time];   % pathwayLeft
            obj.pathway(:, 1, 2) = [robo.POS_e_R(1:2); robo.SV.QeR(3); time];   % pathwayRight
        end

        % pathwayを上書きする関数
        function obj = overWriteGoal(obj, robo, newPathway, time)
            tempObj = obj.reset(robo, time);
            obj.pathway = [tempObj.pathway, newPathway];
        end

        % pathwayを上書きせずに追加する関数
        function obj = addEnd(obj, newPathway)
            obj.pathway = [obj.pathway, newPathway];
        end

        % pathwayのどのフェーズにいるか計算
        % フェーズの立ち上がりと立ち下がりをdtimeをもとに計算
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

        % 現在時刻から，次の目標位置を返す関数
        function nextPathway = goingTo(obj, time, param)
            phase = obj.phase(time, param);
            index = phase + 1;
            if index == 0
                nextPathway = nan(4,2);
                return 
            end
            nextPathway = obj.pathway(:, index, :);
        end

        % pathwayから速度計算
        function desiredVel = vel(obj, time, robo, gain, mode)
            desiredVel(1:3, 1) = calc_Vel(obj.pathway(:, :, 1), time, robo, gain, true, mode);    % left
            desiredVel(4:6, 1) = calc_Vel(obj.pathway(:, :, 2), time, robo, gain, false, mode);   % right
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%% pathway計算関数 %%%%%
        %%% 呼び出された時刻から並進するターゲットに非接触を保ち，その後把持するためのpathwayメソッドを返す．
        % ターゲットの頂点が如何なる角度においても手先に接触しない(ちょうど触れる)位置を経由することで意図しない接触を避ける
        % 基本targOri = pi/4 で捕獲するが，角速度が小さすぎる場合はその限りではない．
        % captureDeltAng : 捕獲時のターゲットの45度姿勢からの角度変化．-pi/4~pi/4
        function pathway = directCapture(obj, targ, time, param)
            % ターゲット相対情報代入
            targPos = targ.SV.R0(1:2, :);
            targV = targ.SV.v0(1:2, :);
            targW = targ.SV.w0(3);
            targWidth = targ.width;
            targOri = targ.SV.Q0(3);
            captureDeltAng = 0;

            % ターゲット頂点がロボットエンドエフェクタの間に入る角度
            alpha = asin(param.LdH * sin(param.LdGamma) / (targWidth/sqrt(2) + param.LdD * .5));

            % 捕獲時のターゲット角度，待機時間決定
            % 角速度０の場合は任意の時間．その他はターゲットの頂点が手先刺股の間に入るまで待機
            if targW == 0
                captureDeltAng =  rem(targOri - pi * .25, pi * .5);
                deltTime = obj.timeParam.move2targ;
            elseif abs(targW) <= .3
                deltTime = obj.timeParam.maxWait;
                desTargOri = targOri + targW * deltTime;
                captureDeltAng = rem(desTargOri - pi * .25, pi * .5);
            else
                % 頂点が刺股に入る角度
                desTargOri = pi * .25 - sign2(targW) * alpha;
                % 待機時間計算
                deltTime = rem(desTargOri - targOri, pi * .5) / targW;
                while(deltTime < obj.timeParam.move2targ) % 時間が小さすぎる場合，回転対称性から時刻を伸ばす
                    deltTime = deltTime + pi * .5 / abs(targW);
                end
            end
            
            captureDeltAngMat = vec2dc([0;0;captureDeltAng]);

            %%% 目標手先位置代入
            % ターゲット頂点に接触しないギリギリの時刻・位置を追従するフェーズ
            targGoalPos = targPos + targV * deltTime;                                           % 接触時点のターゲット重心位置 2*1
            armSign = [-1, +1];                                                                 % 左手なら負，右手なら正
            pathway = zeros(4, 1, 2);                                                       % 目標手先位置初期化 4*1*2
            dX_close = 1.1*sqrt( (targWidth/sqrt(2) + param.LdD*.5)^2 ...              
                          -(param.LdH * sin(param.LdGamma))^2   ) ;                             % 手先先端球がターゲットにギリギリ触れない，ターゲット中心から手先までの距離.許容誤差.03
            dX_capture = 1.03 * (targWidth + param.LdD)/sqrt(2) - param.LdH * sin(param.LdGamma);     % 手先先端球がターゲットに触れる時の，ターゲット中心から手先までの距離.サバよみ
            
            contactPosVecClose = captureDeltAngMat(1:2,1:2) * [dX_close; 0]  .* armSign;     % 接触アーム目標位置の，ターゲット重心に対する相対位置ベクトル 2*2
            contactPosVecCapt  = captureDeltAngMat(1:2,1:2) * [dX_capture; 0] .* armSign;
            pathway(:, 1, 1) = [targGoalPos + contactPosVecClose(:,1);-pi/2+captureDeltAng; deltTime + time]; % 左アーム目標位置代入
            pathway(:, 2, 1) = [targGoalPos + contactPosVecCapt(:,1) ;-pi/2+captureDeltAng; deltTime + time + obj.timeParam.cap];
            pathway(:, 1, 2) = [targGoalPos + contactPosVecClose(:,2); pi/2+captureDeltAng; deltTime + time]; % 右アーム目標位置代入
            pathway(:, 2, 2) = [targGoalPos + contactPosVecCapt(:,2) ; pi/2+captureDeltAng; deltTime + time + obj.timeParam.cap];
        end

        %%% ターゲットが並進してきている方向の手先を接触させることによりターゲット角速度を減衰させる
        % pathwayメソッドをリターン
        function pathway = contactDampen(obj, robo, targ, time, param)
            contPosRate = 0.8;              % 接触する位置の辺に対する割合．１で頂点．０で中心
            contAng = deg2rad(0);              % 接触する時のターゲット角度．おおむね30ど以内にする
            evadeAng = deg2rad(20);          % 接触する時のエンドエフェクター角度．おおむね10ど以内にする

            % ターゲット情報代入
            targPos = targ.SV.R0(1:2, :);
            targV = targ.SV.v0(1:2, :);
            targW = targ.SV.w0(3);
            targWidth = targ.width;
            targOri = targ.SV.Q0(3);
            gamma = param.LdGamma;
            r = param.LdD * .5;

            % 接触パターン設定
            % 回転の向き，並進方向から判別
            armSign = sign2( targV(1) );        % 左手接触なら負，右手接触なら正
            rotSign = sign2(targW);
            contSign = -armSign * rotSign;      % 回転を抑制するための接触位置は，接触速度の方向と回転方向によって決まる
            contArm = [targV(1)<0, targV(1)>=0];% sign2に合わせる

            % 回転行列作成
            contAngMat = vec2dc([0;0;contAng*rotSign]);             % 3*3
            evadeAngMat = vec2dc([0; 0; rotSign*evadeAng]);        % 3*3

            % 接触時点のエンドエフェクターのターゲット重心への相対位置ベクトル計算
            targ2ContPos0 = targWidth * .5 * [armSign; contSign * contPosRate];         % 2*1
            targ2ContPos = contAngMat(1:2, 1:2) * targ2ContPos0;                        % 2*1
            contPos2tip = contAngMat(1:2, 1:2) * [armSign*r; 0];                        % 2*1
            tip2EE = evadeAngMat(1:2, 1:2) * [0;  param.LdH * sin(gamma) * contSign];   % 2*1
            targ2Tip = targ2ContPos + contPos2tip;                                      % 2*1
            targ2EECont  = targ2Tip + tip2EE;                                           % 2*1
            
            % 接触点以前で接触しないための条件設定
            % ターゲット頂点がロボットエンドエフェクタ（接触する方）と接触する角度
            alpha = asin(targ2Tip(2) / (targWidth/sqrt(2)+r)) * armSign;          % 対角線の水平面との成す角．-pi/2 ~ pi/2
            waitOri = pi * .25 + alpha ;

            % 待機位置までの所要時間計算
            dtApproach = rem(waitOri - targOri, pi * .5) / targW;
            while(dtApproach < obj.timeParam.contDmp) % 時間が小さすぎる場合，回転対称性から時刻を伸ばす
                dtApproach = dtApproach + pi * .5 / abs(targW);
            end

            % 待機位置計算
            % わずかに余裕を持たせる
            targ2EEWait(1,1) = (targWidth/sqrt(2)+r) * cos(alpha) * armSign * 1.05 + tip2EE(1);    % 2*1
            targ2EEWait(2,1) = targ2EECont(2);                                              % 2*1

  
            % 接触に要する時間
            % わずかに早く目標位置を達成するように設定
            dtCont = (contAng + pi*.25 - alpha) / abs(targW) * .8;

            %%% 目標手先位置代入
            targWaitPos = targPos + targV * dtApproach;                     % 非接触時点のターゲット重心位置 2*1
            targGoalPos = targWaitPos + targV * dtCont;                     % 接触時のターゲット重心位置 2*1

            % 非接触手先位置計算
            targ2EEFree(1,1) = (targWidth/sqrt(2) + r) * 1.5 * -armSign;            % 2*1
            targ2EEFree(2,1) = robo.SV.R0(2)+.25-targWaitPos(2);                    % 2*1

            % 計算結果をpwathwayに代入．接触しないアームは現状の位置を保持
            tempObj = obj.reset(robo, time);
            pathway = repmat(tempObj.pathway, [1, 2]);
            effAngInit = pi * .5 * armSign;
            desEffAngGoal = evadeAng * rotSign + effAngInit;                   % 左手か右手かによって初期状態の手先角度が異なる
            
            pathway(1:3, 1, contArm) = [targWaitPos + targ2EEWait;desEffAngGoal];   % 非接触待機位置代入
            pathway(1:3, 2, contArm) = [targGoalPos + targ2EECont ;desEffAngGoal];  % 接触時位置代入  
            pathway(1:2, 1, ~contArm)= targWaitPos + targ2EEFree;  % 非接触側の手先はターゲットに触れない位置にする
            pathway(1:2, 2, ~contArm)= targWaitPos + targ2EEFree;  % 非接触側の手先はターゲットに触れない位置にする
            pathway(4, 1, :) = dtApproach + time;                                   % 時刻設定
            pathway(4, 2, :) = dtApproach + time + dtCont;                          % 時刻設定
            
        end

        % 反力低減性検証用テスト
        % 左手接触で，ターゲットに回転がない場合
        function pathway = testImpedanceNoRotate(obj, target, time)
            % ターゲット情報代入
            targPos = target.SV.R0(1:2, :);
            targV = target.SV.v0(1:2, :);
            targWidth = target.width;

            dtCont = obj.timeParam.move2targ*1.1;
            dtMove = obj.timeParam.move2targ;
            targGoalPos = targPos + targV * dtCont;
            targ2cont = [targWidth * .5; 0];
            
            pathway(1:3, 1, 1) = [targGoalPos - targ2cont;-pi * .5];        % 非接触待機位置代入
            pathway(1:3, 1, 2) = [[.2 ; .35 ]; pi * .5];                    % 非接触側の手先はターゲットに触れない位置にする
            pathway(4, 1, :) = dtMove + time;                               % 時刻設定
        end
        
        %%% 設定した時間現在位置を保持するpathway設定
        function pathway = keepPosition(obj, robo, currentTime, deltTime)
            tempObj = obj.reset(robo, currentTime);
            p1 = tempObj.pathway;
            pathway(:, 1, :) = p1;
            p1(4, :, :) = p1(4, :, :) + deltTime;
            pathway(:, 2, :) = p1;
        end
    end
end