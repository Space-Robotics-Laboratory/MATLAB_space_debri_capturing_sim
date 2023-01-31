% 双腕ロボ手先球と四角形ターゲットの接触力計算関数
% 
% 2023.1 uchida akiyoshi
% 
% input  :DualArmRobo class, Targer class, contactElast, contactDamp
% output :[EdgeWrench[LeftWrench 6*1, RightWrnech 6*1], TagrgetWrench 6*1, contactState 1*1*4] ; Wrench = [Fource 3*1; Torque 3*1]
%
% 接触している辺の判定は，1ステップ前の状態と比較して行う
% 多次元配列は，dim2:targetTip , dim3:roboArmTipとして定義
%

function [edgeWrench, targetWrench] = calc_ContactForce(DualArmRobo, Target, contactElast, contactDamp)
    tWidth = Target.width;          % ターゲット横
    tDepth = Target.depth;          % ターゲット縦

    % ロボット端球位置計算，第三次元方向にアーム端球
    roboArmTipsPos_0= [DualArmRobo.POS_es_L, DualArmRobo.POS_es_R]; % dim1:[xyz], dim2:roboArmTip
    roboArmTipsPos_1 = repmat(roboArmTipsPos_0, [1, 1, 4]);         % dim1:[xyz], dim2:roboArmTip, dim3:targetTip
    roboArmTipsPos = permute(roboArmTipsPos_1, [1, 3, 2]);          % dim1:[xyz], dim2:targetTip , dim3:roboArmTip

    % 先端級半径設定
    r = DualArmRobo.r;

    % ターゲット頂点計算
    targetTipsPos_0 = calc_SquareTips(Target.SV.R0(1:2), tWidth, tDepth, Target.SV.Q0(3));  % 頂点計算 2*4
    targetTipsPos_1 = [targetTipsPos_0; zeros(1,4)];                                        % 外積計算のためにvec2からvec3に拡張3*4
    targetTipsPos = repmat(targetTipsPos_1, [1, 1, 4]);                                     % 外積籍計算のために次元拡張 3*4*4

    % ロボアーム端球中心からターゲット頂点までの位置ベクトル
    robo2TargetVec = roboArmTipsPos - targetTipsPos;        % dim1:[xyz], dim2:targetTip, dim3:roboArmTip

    % ターゲット辺のベクトル計算
    targetSideVec_0 = targetTipsPos(:, [2, 3, 4, 1], :) - targetTipsPos;    % 辺ベクトル計算
    targetSideVec = targetSideVec_0 ./ [tDepth, tWidth, tDepth, tWidth];    % 辺ベクトル正規化
    
    % 外積計算により接触判定
    pDeltas_0 = cross(robo2TargetVec, targetSideVec, 1);                    % 端球中心と辺直線の距離計算 3*4*4
    pDeltas = pDeltas_0(3, :, :) + r;                                       % 端球の周と辺直線の距離計算 1*4*4
    sideContact= (pDeltas > 0);                                             % 端球の周の辺直線へのめり込み判定 1*4*4
    isContact = all(sideContact, 2);                                        % 全ての辺直線に対して条件を満たしていたら接触　1*1*4
                                                                            % isContact(:,:,i)はarmTip(i)の接触状況を示す.
    tipContact = (pDeltas_0(3, :, :) < 0) & sideContact;                    % 頂点付近であるか判定 1*4*4

    % 全ての先端球が接触していなければ，接触力は0
    % ターゲット頂点付近でない場合，後の計算をスキップ
    if ~any(isContact)            
        edgeWrench = zeros(6,2);
        targetWrench = zeros(6,1);
        return
    end
    
    % ターゲット頂点付近の場合，頂点とアームチップの距離を判定
    targetTipsPos_2 = zeros(3, 4, 4);
    roboArmTipsPos_2 = zeros(3, 4, 4);
    targetTipsPos_2(:, tipContact) = targetTipsPos(:, tipContact);
    roboArmTipsPos_2(:, tipContact) = roboArmTipsPos(:, tipContact);
    distance = vecnorm(targetTipsPos_2 - roboArmTipsPos_2);
    
    % 頂点付近非接触
    if all(distance > r, "all")
        edgeWrench = zeros(6,2);
        targetWrench = zeros(6,1);
        return
    end

    % 接触している球に関して力とトルクを計算  
    % 最もめり込み量が小さい辺を接触辺とする
    [~, I] = min(pDeltas, [], 2); 
    pDelta(:, :, ~isContact) = 0
    edgeWrench = zeros(6,2);
    targetWrench = zeros(6,1);     
    targetSideVel = 0;
    
end