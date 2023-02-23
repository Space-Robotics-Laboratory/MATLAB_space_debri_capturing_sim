% ロボットコントローラー．制御はこの関数内でおこなっている．
% 関節トルクを計算することによって制御．
%
% 2023.2 uchida akiyoshi
%

function [turques, pathway] = control_JointTau(robo, target, prePathway, roboExt, state, param, time)

%%% 捕獲成功時
if state.isCapture
    % 捕獲後，ベースに対して停止させる
    turques = calc_TauByVel(robo, zeros(6,1), roboExt, [true, true]);  % [ture, true]は左右どちらの速度もロボットに対する表現であるという意味
    pathway = prePathway; % 使わない
    return
end

% 捕獲前は位置制御に基づく分解速度制御によって関節トルクを管理する
controlMode = 1;

switch controlMode
    
    %%% 直接捕獲するモード
    case 1
        % フェーズ計算
        [phase, ~, phaseEnding] = calc_PathwayPhase(prePathway, time, param);
        
        % ターゲットに接近するフェーズ
        if time == 0
            % start phase 1
            pathway = set_Pathway2NearTarget(prePathway, target, param, time, 1); % ターゲットに近づける 4*2*2
        
        % 接近が終了したら，ターゲット角がある値になるまで待機 -> pi/4付近
        elseif phase == 1 && phaseEnding
            % start phase 2
            if target.SV.w0(3) >= 0     % 回転方向によって待機角度を変える
                alpha = pi * .25 * .9;
            else
                alpha = pi * .25 * 1.1;
            end
            pathway = set_Pathway2WaitTarget(prePathway, target, alpha); % 4*3*2
        
        % 待機終了後，捕獲開始
        elseif phase == 2 && phaseEnding
            % start phase 3
            pathway = set_Pathway2Target(prePathway, target, param, time, 0);  % ターゲット頂点把持の位置を設定 % 4*4*2
        else
            pathway = prePathway; % 4*4*2
        end
        
        % 手先速度計算
        desiredVel(1:3, :) = calc_Vel(pathway(:, :, 1), time, 2);       % 目標左手先速度計算
        desiredVel(4:6, :) = calc_Vel(pathway(:, :, 2), time, 2);       % 目標右手先速度計算
        turques = calc_TauByVel(robo, desiredVel, roboExt, [false, false]);

    %%% 手先を交互に接触させ，角運動量を減衰させてから捕獲するモード
    case 2
end
