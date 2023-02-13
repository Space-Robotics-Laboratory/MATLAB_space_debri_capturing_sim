% 双腕ロボ手先球と四角形ターゲットの接触力計算関数
% 
% 2023.1 uchida akiyoshi
% 
% input  :DualArmRobo class, Targer class, contactElast, contactDamp
% output :[roboEdgeWrench[LeftWrench 6*1, RightWrnech 6*1], TagrgetWrench 6*1, contactState 1*4] ; Wrench = [Fource 3*1; Torque 3*1]
%
% 接触している辺の判定は，のめり込み量が最も小さい辺を接触辺とする
% ターゲット辺，エンドエフェクタを同時に扱うために多次元配列を用いて計算しており，
% cross()使用には次元を合わせる必要があるため，repmatによる次元拡張が多用される.
% 
% 多次元配列は，dim1:xyz , dim2:targetTip , dim3:roboArmEndEfectorとして定義
%

function [edgeWrench, targetWrench, isContact] = calc_ContactForce(DualArmRobo, Target, contactElast, contactDamp, contactNu)
    tWidth = Target.width;                      % ターゲット横
    tDepth = Target.depth;                      % ターゲット縦
    m2G  = Target.m2G(1:2);                     % ターゲット質量重心から幾何中心への相対位置ベクトル 2*1
    targetZ = repmat([0, 0, 1]', [1,4]);        % ターゲットz方向

    % ロボット末端位置代入（端球の中点） 
    leftEdgePos =  repmat(DualArmRobo.POS_e_L, [1,2]);              % 後の計算のために拡張 3*2
    rightEdgePos = repmat(DualArmRobo.POS_e_R, [1,2]);              % 後の計算のために拡張 3*2
    edgePos = [leftEdgePos, rightEdgePos];                          % ロボ手先位置 3*4       

    % ターゲット位置・速度，角速度代入
    targetV0 = Target.SV.v0;
    targetW0 = repmat(Target.SV.w0, [1, 4]);
    targetR0 = Target.SV.R0;

    % ロボット端球速度代入
    endeffecVel = [DualArmRobo.VEL_es_L, DualArmRobo.VEL_es_R];         % 3*4

    % ロボット端球位置代入，第三次元方向にアーム端球
    endEffecPosLight= [DualArmRobo.POS_es_L, DualArmRobo.POS_es_R];     % dim1:[xyz], dim2:roboArmTip 3*4 
    endEffecPos_temp = repmat(endEffecPosLight, [1, 1, 4]);             % dim1:[xyz], dim2:roboArmTip, dim3:targetTip 3*4*4
    endEffecPos = permute(endEffecPos_temp, [1, 3, 2]);                 % dim1:[xyz], dim2:targetTip , dim3:roboArmTip 3*4*4

    % 先端級半径設定
    r = DualArmRobo.r;

    % ターゲット頂点計算
    targetTipsPos_temp = calc_SquareTips(Target.SV.R0(1:2)+m2G, tWidth, tDepth, Target.SV.Q0(3));   % 頂点計算 2*4
    targetTipsPos_temp = [targetTipsPos_temp; zeros(1,4)];                                          % 外積計算のためにvec2からvec3に拡張3*4 後に再び使用
    targetTipsPos = repmat(targetTipsPos_temp, [1, 1, 4]);                                          % 外積計算のために次元拡張 3*4*4

    % ターゲット頂点からロボアームエンドエフェクター中心までの位置ベクトル
    tip2EndEffecCent = endEffecPos - targetTipsPos;        % dim1:[xyz], dim2:targetTip, dim3:roboArmEndEfector

    % ターゲット辺のベクトル計算
    targetSideVec = targetTipsPos(:, [2, 3, 4, 1], :) - targetTipsPos;      % 辺ベクトル計算
    targetSideDire = targetSideVec ./ [tDepth, tWidth, tDepth, tWidth];     % 辺ベクトル正規化
    
    % 外積計算により接触判定
    side2endEffecCent = cross(tip2EndEffecCent, targetSideDire, 1);         % 端球中心と辺直線の距離計算 3*4*4 内向き
    side2endEffecCircuit = side2endEffecCent(3, :, :) + r;                  % 端球の周と辺直線の距離計算 1*4*4 内向き
    isPenetSide = (side2endEffecCircuit > 0);                               % 端球の周の辺直線へのめり込み判定 1*4*4
    mayContact = all(isPenetSide, 2);                                       % 全ての辺直線に対して条件を満たしていたら接触可能性有　1*1*4
                                                                            % mayContact(:,:,i)はarmTip(i)の接触状況を示す.

    % 全ての先端球が接触していなければ，接触力は0
    % 接触可能性がない場合，後の計算をスキップ
    if ~any(mayContact) 
        isContact = zeros(1, 4);            % 非接触
        edgeWrench = zeros(6,2);            % robo外力０
        targetWrench = zeros(6,1);          % ターゲット外力０
        return
    end

    % 接触可能性有り，詳しい判定
    % mayContact False のendEfecに注意
    inSideArea = side2endEffecCent(3, :, :) < 0;                            % 端球中心がターゲット辺と並行な領域にあるか判定 1*4*4
    inEdgeArea = inSideArea & inSideArea(:, [4, 1, 2, 3], :);               % 端球中心がターゲット辺直線で作る格子の角領域にあるか判定 1*4*4
    dEndEfecCent2Tip = vecnorm(tip2EndEffecCent);                           % 端球中心からターゲット頂点までの距離 1*4*4
    isTipContact = any((dEndEfecCent2Tip < r) & inEdgeArea, 2);             % 頂点周りの半径rの円内で端球中心が接触か判定 1*1*4
    isSideContact = mayContact & ~any(inEdgeArea, 2);                       % 頂点まわり以外での接触か判定 1*1*4
    isContact = squeeze(isSideContact | isTipContact)';                     % 接触判定 1*4, isContact(:,:,i)はarmTip(i)の接触状況を示す.

    
    % 全ての球が非接触．mayContactで判定しなかった頂点付近接触を判定
    if ~any(isContact)
        edgeWrench = zeros(6,2);
        targetWrench = zeros(6,1);
        return
    end
    
    % 最もめり込み量が小さい辺を接触辺とする
    
    dXs = zeros(1,4,4);
    dXs(:, :, isContact) = side2endEffecCircuit(:, :, isContact);           % endEfec中心からターゲット辺までの距離,非接触の場合０とする 1*4*4
    [dX, contactSide] = min(dXs, [], 2);                                    % 最もめり込み量が小さい辺を接触辺とし，めり込み量計算 1*1*4
                                                                            % contactSideは接触辺のインデックス 1*1*4
    dX = squeeze(dX)';                                                      % 次元削除 1*4

    % 接触相対位置計算
    targetSideNorm = cross(targetSideDire(:,:,1), targetZ);                 % ターゲット辺法線ベクトル，内向き 3*4
    contactPos = endEffecPosLight + r * targetSideNorm(:, contactSide) ;    % 各端球接触位置 3*4
    target2ContactPos = contactPos - targetR0;                              % ターゲット重心から接触点への相対位置 3*4 非接触点は後に除く
    edge2ContactPos  =  contactPos - edgePos;                               % ロボ手先からの相対位置 3*4 非接触点に注意
    
    % 弾性項計算
    elasTargetForces = contactElast * dX;                                   % 辺法線方向 1*4

    % 減衰項計算
    targetContactVel = targetV0 + cross(targetW0, target2ContactPos);       % ターゲット接触点速度 3*4
    velDiff = endeffecVel - targetContactVel;                               % 速度差計算 3*4
    dV = dot(velDiff, targetSideNorm(:, contactSide));                      % 法線方向のめり込み速さ 1*4
    dV(:, ~isContact) = 0;                                                  % 非接触点は０
    dampTargetForces = contactDamp * dV;                                    % 減衰係数による力 1*4

    % 摩擦項計算
    targetForcesN = elasTargetForces + dampTargetForces;                    % 法線方向成分力 1*4
    dVH = dot(velDiff, targetSideDire(:, contactSide));                     % 接線方向速度 1*4
    velDiffSign = sign(dVH);                                                % 接線方向速度符号 1*4
    targetForcesM = contactNu * targetForcesN .* velDiffSign;               % 接線方向力

    targetForces = targetForcesN .* targetSideNorm(:, contactSide) ...
                 + targetForcesM .* targetSideDire(:, contactSide);          % 各種力合算 3*4

    roboTorques = cross(edge2ContactPos, -targetForces);                    % ロボトルク 3*4
    tageTorques = cross(target2ContactPos, targetForces);                   % ターゲットトルク 3*4

    roboForce = -targetForces(:, [1, 3]) - targetForces(:, [2, 4]);         % 手先合力 3*2
    roboTorque = roboTorques(:, [1, 3]) + roboTorques(:, [1, 3]);           % 手先合トルク 3*2
    tageForce = sum(targetForces, 2);                                       % ターゲット合力 3*1
    tageTorque = sum(tageTorques, 2);                                       % ターゲット合トルク 3*1
    
    edgeWrench = [roboForce; roboTorque];
    targetWrench = [tageForce; tageTorque]; 
end