% 推定されたターゲット状態から目標手先位置をPathway[x(te), y(te), theta(te), te]'で計算する関数
% ターゲット頂点をターゲット対角線が水平になる時刻で挟む挙動を示す.
% 呼び出された時刻から目標位置までの経路を設定するため，呼び出す時刻に注意．
%
% 2023.2 akiyoshi uchida
%
% input : targetSV
% output: pathway [ pathwayL, pathwayR ] 4*n*2
%
% pathwayが満たされる前に次のが計算されるバグあり．

function pathway = set_Pathway2Target(prePathway, target, param, time, minDeltTime)
% 推定されたターゲット状態
targPos = target.SV.R0(1:2, :);
targOri = target.SV.Q0(3);
targV = target.SV.v0(1:2, :);
targW = target.SV.w0(3);
targWidth = target.width;

% 目標位置までに要する時間
deltaT = (pi * .25 - rem( targOri, pi * .25 )) / abs(targW);                % ターゲット姿勢がpi/4の整数倍となり，対角線が水平になるまでの時間
while(deltaT < minDeltTime) % 速度が大きくなりすぎることを防ぐため，時間があまりに短いと伸ばす
    deltaT = deltaT + pi * .5 / abs(targW);
end

% 目標手先位置
targGoalPos = targPos + targV * deltaT;                                     % 接触時点のターゲット重心位置 2*1
armSign = [-1, +1];                                                         % 左手なら負，右手なら正
goalPathway = zeros(4, 1, 2);                                               % 目標手先位置初期化 4*1*2
deltaX = (targWidth + param.LdD)/sqrt(2) - param.LdH * sin(param.LdGamma); % 手先先端球がターゲットに触れる時の，ターゲット中心から手先までの距離

contactPosVec = [deltaX; 0]  .* armSign * .9;                               % 接触アーム目標位置の，ターゲット重心に対する相対位置ベクトル 2*2
goalPathway(:, 1, 1) = [targGoalPos + contactPosVec(:,1); 0; deltaT + time];% 左アーム目標位置代入
goalPathway(:, 1, 2) = [targGoalPos + contactPosVec(:,2); 0; deltaT + time];% 右アーム目標位置代入
pathway = [prePathway, goalPathway];                                        % 現状のpathwayに新たな目標pathwayを追加
return
end