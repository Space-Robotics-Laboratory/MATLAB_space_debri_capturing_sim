% 双腕ロボ手先球と四角形ターゲットの接触力計算関数
% 
% 2023.1 uchida akiyoshi
% 
% input  :DualArmRobo class, Targer class, contactElast, contactDamp
% output :[EdgeWrench[LeftWrench 6*1, RightWrnech 6*1], TagrgetWrench 6*1, contactState 1*1*4] ; Wrench = [Fource 3*1; Torque 3*1]
%
% 接触している辺の判定は，のめり込み量が最も小さい辺を接触辺とする
% 多次元配列は，dim1:xyz , dim2:targetTip , dim3:roboArmEndEfectorとして定義
%

function [edgeWrench, targetWrench] = calc_ContactForce(DualArmRobo, Target, contactElast, contactDamp)
    tWidth = Target.width;                      % ターゲット横
    tDepth = Target.depth;                      % ターゲット縦
    targetZ = repmat([0, 0, 1]', [1,4]);        % ターゲットz方向

    % ロボット端球位置計算，第三次元方向にアーム端球
    endEfecPos_0= [DualArmRobo.POS_es_L, DualArmRobo.POS_es_R]; % dim1:[xyz], dim2:roboArmTip
    endEfecPos_1 = repmat(endEfecPos_0, [1, 1, 4]);             % dim1:[xyz], dim2:roboArmTip, dim3:targetTip
    endEfecPos = permute(endEfecPos_1, [1, 3, 2]);              % dim1:[xyz], dim2:targetTip , dim3:roboArmTip

    % 先端級半径設定
    r = DualArmRobo.r;

    % ターゲット頂点計算
    targetTipsPos_0 = calc_SquareTips(Target.SV.R0(1:2), tWidth, tDepth, Target.SV.Q0(3));  % 頂点計算 2*4
    targetTipsPos_1 = [targetTipsPos_0; zeros(1,4)];                                        % 外積計算のためにvec2からvec3に拡張3*4
    targetTipsPos = repmat(targetTipsPos_1, [1, 1, 4]);                                     % 外積計算のために次元拡張 3*4*4

    % ロボアームエンドエフェクター中心からターゲット頂点までの位置ベクトル
    endEfecCent2Tip = endEfecPos - targetTipsPos;        % dim1:[xyz], dim2:targetTip, dim3:roboArmEndEfector

    % ターゲット辺のベクトル計算
    targetSideVec = targetTipsPos(:, [2, 3, 4, 1], :) - targetTipsPos;      % 辺ベクトル計算
    targetSideDire = targetSideVec ./ [tDepth, tWidth, tDepth, tWidth];     % 辺ベクトル正規化
    
    % 外積計算により接触判定
    endEfecCent2Side = cross(endEfecCent2Tip, targetSideDire, 1);           % 端球中心と辺直線の距離計算 3*4*4
    endEfecCircuit2Side = endEfecCent2Side(3, :, :) + r;                    % 端球の周と辺直線の距離計算 1*4*4
    isPenetSide = (endEfecCircuit2Side > 0);                                % 端球の周の辺直線へのめり込み判定 1*4*4
    mayContact = all(isPenetSide, 2);                                       % 全ての辺直線に対して条件を満たしていたら接触可能性有　1*1*4
                                                                            % mayContact(:,:,i)はarmTip(i)の接触状況を示す.

    % 全ての先端球が接触していなければ，接触力は0
    % 接触可能性がない場合，後の計算をスキップ
    if ~any(mayContact)            
        edgeWrench = zeros(6,2);
        targetWrench = zeros(6,1);
        return
    end

    % 接触可能性有り，詳しい判定
    % mayContact False のendEfecに注意
    inSideArea = endEfecCent2Side(3, :, :) < 0;                             % 端球中心がターゲット辺と並行な領域にあるか判定 1*4*4
    inEdgeArea = inSideArea & inSideArea(:, [4, 1, 2, 3], :);               % 端球中心がターゲット辺直線で作る格子の角領域にあるか判定 1*4*4
    dEndEfecCent2Tip = vecnorm(endEfecCent2Tip);                            % 端球中心からターゲット頂点までの距離 1*4*4
    isTipContact = any((dEndEfecCent2Tip < r) & inEdgeArea, 2);             % 頂点周りの半径rの円内で端球中心が接触か判定 1*1*4
    isSideContact = mayContact & ~any(inEdgeArea, 2);                       % 頂点まわり以外での接触か判定 1*1*4
    isContact = isSideContact | isTipContact;                               % 接触判定 1*1*4, isContact(:,:,i)はarmTip(i)の接触状況を示す.

    
    % 全ての球が非接触．mayContactで判定しなかった頂点付近接触を判定
    if ~any(isContact)
        edgeWrench = zeros(6,2);
        targetWrench = zeros(6,1);
        return
    end

    % 接触している球に関して力とトルクを計算  
    % 最もめり込み量が小さい辺を接触辺とする
    % これまでと次元の取り扱いが異なる
    % 多次元配列は，dim1:xyz , dim2:roboArmEndEfector
    dXs = zeros(1,4,4);
    dXs(:, isContact, :) = permute(endEfecCircuit2Side(:, :, isContact), [1, 3, 2]);    % endEfec中心からターゲット辺までの距離,非接触の場合０とする 1*4*4
    targetSideNorm = cross(targetSideDire(:,:,1), targetZ);                             % ターゲット辺法線ベクトル，内向き 3*4
    [dX, contactSide] = min(dXs, [], 3);                                                % 最もめり込み量が小さい辺を接触辺とし，めり込み量計算 1*4
    targetSideVel = 0;
    forces = contactElast * dX .* targetSideNorm(:, contactSide);
    torques = zeros(3,4);
    roboForce = -forces(:,[1, 3]) - forces(:, [2, 4]);
    roboTorque = -torques(:, [1, 3]) - torques(:, [2, 4]);
    tageForce = sum(forces, 2);
    tageTorque = sum(torques, 2);
    edgeWrench = [roboForce; roboTorque];
    targetWrench = [tageForce; tageTorque]; 
end