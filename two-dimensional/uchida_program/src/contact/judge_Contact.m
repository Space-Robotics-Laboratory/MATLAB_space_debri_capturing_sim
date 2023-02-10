% ターゲットのロボットの接触状況を判定する関数
% 不完全で編集中．現時点では使用しない
% ２次元でのみ機能する
%
% 2023.1 Akiyoshi Uchida
%
% input : Target class, DualArmRobo class
% output: isContact bool 1*4, [tip1L, tip2L, tip1R, tip2R]
%
% 

function isContact = judge_Contact(Target, DualArmRobo)
    tWidth = Target.width;
    tDepth = Target.depth;

    % ロボット端球位置計算，第三次元方向にアーム端球
    roboArmTipsPos_0= [DualArmRobo.POS_es_L, DualArmRobo.POS_es_R];
    roboArmTipsPos_1 = repmat(roboArmTipsPos_0, [1, 1, 4]);
    roboArmTipsPos = permute(roboArmTipsPos_1, [1, 3, 2]);

    % 先端級半径設定
    r = DualArmRobo.r;

    % ターゲット頂点計算
    targetTipsPos_0 = calc_SquareTips(Target.SV.R0(1:2), tWidth, tDepth, Target.SV.Q0(3));
    % 外積計算のために2dから3dに拡張
    targetTipsPos = [targetTipsPos_0; zeros(1,4)];          

    % ロボアーム端球からターゲット頂点までの位置ベクトル
    robo2TargetVec = roboArmTipsPos - targetTipsPos;        % dim1:vec3, dim2:targetTip, dim3:roboArmTip

    % ターゲット辺のベクトル計算
    targetSideVec_0 = targetTipsPos(:, [2, 3, 4, 1]) - targetTipsPos;       % 辺ベクトル計算
    targetSideVec_1 = targetSideVec_0 ./ [tDepth, tWidth, tDepth, tWidth];  % 正規化
    targetSideVec = repmat(targetSideVec_1, [1, 1, 4]);                     % 次元拡張
    
    % 外積計算により接触判定
    isContact_0 = cross(robo2TargetVec, targetSideVec, 1);                  % 点と辺の距離計算
    isContact_1 = (isContact_0(3, :, :) > -r);                              % 点と辺の距離と球半径を比較
    isContact_2 = all(isContact_1, 2);                                      % 全ての辺に対して条件を満たしていたら接触　
    isContact = permute(isContact_2, [1, 3, 2]);                            % 次元を調整し横ベクトルに変換　
end