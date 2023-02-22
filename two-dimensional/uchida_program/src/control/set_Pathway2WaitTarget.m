% 推定されたターゲット状態から目標手先位置をPathway[x(te), y(te), theta(te), te]'で計算する関数
% pathwayの最終位置から，ターゲット角がある角以下になるまで静止するpathwayを設定する．速度制御でも可能であるが，位置制御に統一するため導入．
% ターゲット角は対照性を考慮して-pi/4~pi/4
%
% 2023.2 akiyoshi uchida
%
% input : targetSV
% output: pathway [ pathwayL, pathwayR ] 4*n*2
%
% pathwayが満たされる前に次のが計算されるバグあり．

function pathway = set_Pathway2WaitTarget(prePathway, target, desTargOri)
targW = target.SV.w0(3);
targOri = target.SV.Q0(3);

% 待機時間計算
deltTime = (desTargOri - rem( targOri, pi * .25 )) / abs(targW);

goalPath = prePathway(:, end, :);
goalPath(4, 1, :) = goalPath(4, 1, :) + deltTime;

pathway = [prePathway, goalPath];