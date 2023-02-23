% 推定されたターゲット状態から目標手先位置をPathway[x(te), y(te), theta(te), te]'で計算する関数
% pathwayの最終位置から，ターゲット角がある角以下になるまで静止するpathwayを設定する．速度制御でも可能であるが，位置制御に統一するため導入．
% ターゲット角は対照性を考慮して0~pi/2
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
deltTime = rem(desTargOri - targOri, pi * .5) / targW ;
while(deltTime < 0) % 負の時刻の場合，回転対称性から生の時刻に治す
    deltTime = deltTime + pi * .5 / abs(targW);
end

goalPath = prePathway(:, end, :);
goalPath(4, 1, :) = goalPath(4, 1, :) + deltTime;

pathway = [prePathway, goalPath];