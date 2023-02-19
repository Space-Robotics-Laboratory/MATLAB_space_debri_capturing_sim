% 推定されたターゲット状態から目標手先位置をPathway[x(te), y(te), theta(te), te]'で計算する関数
%
% 2023.2 akiyoshi uchida
%
% input : targetSV
% output: pathway [ pathwayL, pathwayR ] 4*n*2
%
% pathwayが満たされる前に次のが計算されるバグあり．

function pathway = set_Pathway(prePathway, target, isContact, wasContact, time)

% 推定されたターゲット状態
targPos = target.SV.R0;
targOri = target.SV.Q0(3);
targV = target.SV.v0;
targW = target.SV.w0(3);
targWidth = target.width;

% contact flag 立ち上がり
newCotact = isContact & ~wasContact;

if time ~= 0 && all(~newCotact)
    pathway = prePathway;
    return
end

% 目標位置までに要する時間
deltaT = (pi * .25 - rem( targOri, pi * .25 )) / abs(targW);                % ターゲット姿勢がpi/4の整数倍となり，対角線が水平になるまでの時間
% while(deltaT <= 0.3)
%     deltaT = deltaT + pi * .5 / abs(targW);
% end

% 右回転なら左手接触，左回転なら右手接触
contactArm = [targW <= 0, targW > 0];

% 目標手先位置
targGoalPos = targPos + targV * deltaT;                                     % 接触時点のターゲット重心位置
armSign = [-1, +1];                                                         % 左手なら負，右手なら正
goalPathway = zeros(4, 1, 2);                                               % 目標手先位置初期化

contactVec = [targWidth; 0; 0] / sqrt(2) * armSign(contactArm)*.9;          % 接触アーム目標位置の，ターゲット重心に対する相対位置ベクトル
nonContVec = [.15 * armSign(~contactArm); -.05; 0];                         % 非接触アーム目標位置の，相対位置ベクトル
goalPathway(:, 1, contactArm) = [targGoalPos + contactVec; deltaT + time];  % 接触アーム目標位置代入
goalPathway(:, 1,~contactArm) = [targGoalPos + nonContVec; deltaT + time];  % 非接触アーム目標位置代入
pathway = [prePathway, goalPathway];                                        % 現状のpathwayに新たな目標pathwayを追加
return
end