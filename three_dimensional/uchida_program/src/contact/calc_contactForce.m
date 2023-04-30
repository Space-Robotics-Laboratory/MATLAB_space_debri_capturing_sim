% 双腕ロボ手先球と四角形ターゲットの接触力計算関数
% 
% 2023.1 uchida akiyoshi
% 2023.3 ロボットトルクの計算で, torque[1, 3] + torque[2, 4]とすべきところを
%        torque[1, 3] + torque[1, 3]としていた．改善済み．
% 2023.3 回転角速度が大きい場合，計算誤差が大きくなることに注意．
% 
% input  :DualArmRobo class, Targer class, contactElast, contactDamp
% output :[roboEdgeWrench[LeftWrench 6*2, RightWrnech 6*2], TagrgetWrench 6*1, contactState 1*4] ; Wrench = [Fource 3*1; Torque 3*1]
%
% 接触している辺の判定は，のめり込み量が最も小さい辺を接触辺とする
% ターゲット辺，エンドエフェクタを同時に扱うために多次元配列を用いて計算しており，
% cross()使用には次元を合わせる必要があるため，repmatによる次元拡張が多用される.
% 
% 多次元配列は，dim1:xyz , dim2:targetTip , dim3:roboArmEndEfectorとして定義
%

function [edgeWrench, targetWrench, isContact] = calc_contactForce(DualArmRobo, Target, param)
    contactElast = param.contact.elast;         % 弾性係数
    contactDamp = param.contact.damper;         % 減衰係数
    contactNu = param.contact.friction;         % 摩擦係数
    targetWidth = Target.width;                      % ターゲット横
    targetDepth = Target.depth;                      % ターゲット縦
    targetHeight = Target.height;                     % ターゲット高
    m2G  = Target.m2G;                           % ターゲット質量重心から幾何中心への相対位置ベクトル 3*1
    targetZ = repmat([0, 0, 1]', [1,4]);        % ターゲットz方向

    % ロボット末端位置代入（端球の中点） 
    leftEdgePos =  repmat(DualArmRobo.POS_e_L, [1,2]);              % 後の計算のために拡張 3*2
    rightEdgePos = repmat(DualArmRobo.POS_e_R, [1,2]);              % 後の計算のために拡張 3*2
    edgePos = [leftEdgePos, rightEdgePos];                          % ロボ手先位置 3*4       

    % ターゲット位置・速度，角速度代入
    targetV0 = Target.SV.v0;
    targetW0 = repmat(Target.SV.w0, [1, 4]);
    targetR0 = Target.SV.R0;
    targetORI = Target.SV.A0;

    % ロボット端球速度代入
    endeffecVel = [DualArmRobo.VEL_es_L, DualArmRobo.VEL_es_R];         % 3*4

    % ロボット端球位置代入，第三次元方向にアーム端球
    endEffecPos_2mat = [DualArmRobo.POS_es_L, DualArmRobo.POS_es_R];    % dim1:[xyz], dim2:roboArmTip 3*n
    endEffecPos_3mat_temp = repmat(endEffecPos_2mat, [1, 1, 6]);        % dim1:[xyz], dim2:roboArmTip, dim3:targetFaces 3*n*6
    endEffecPos_3mat = permute(endEffecPos_3mat_temp, [1, 3, 2]);       % dim1:[xyz], dim2:targetFaces, dim3:roboArmTip 3*6*n
    [~, tipNum] = size(endEffecPos_2mat);
    % endEffecPosLight= [DualArmRobo.POS_es_L, DualArmRobo.POS_es_R];     % dim1:[xyz], dim2:roboArmTip 3*n 
    % endEffecPos_temp = repmat(endEffecPosLight, [1, 1, 4]);             % dim1:[xyz], dim2:roboArmTip, dim3:targetTip 3*4*n
    % endEffecPos = permute(endEffecPos_temp, [1, 3, 2]);                 % dim1:[xyz], dim2:targetTip , dim3:roboArmTip 3*4*n

    % 先端級半径設定
    r = DualArmRobo.r;

    % ターゲット頂点計算
    [vertices_t, faces] = calc_cubic(targetR0+m2G, targetWidth, targetDepth, targetHeight, targetORI); % 8*3, 6*4
    vertices = vertices_t'; % 3*8
    % targetTipsPos_temp = calc_SquareTips(Target.SV.R0(1:2)+m2G, targetWidth, targetDepth, Target.SV.Q0(3));   % 頂点計算 2*4
    % targetTipsPos_temp = [targetTipsPos_temp; zeros(1,4)];                                          % 外積計算のためにvec2からvec3に拡張3*4 後に再び使用
    % targetTipsPos = repmat(targetTipsPos_temp, [1, 1, 4]);                                          % 外積計算のために次元拡張 3*4*4

    % ターゲット頂点からロボアームエンドエフェクター中心までの位置ベクトル
    % tip2EndEffecCent = endEffecPos - targetTipsPos;        % dim1:[xyz], dim2:targetTip, dim3:roboArmEndEfector

    % ターゲット面法線計算（内向き）
    vertBases = vertices(:, [1, 2, 3, 4, 1, 5]);
    targetFaceNorm_temp = vertices(:, [4, 1, 2, 3, 5, 1]) - vertBases;   % 3*6
    targetFaceNorm_2mat = targetFaceNorm_temp ./ [targetDepth, targetWidth, targetDepth, targetWidth, targetHeight, targetHeight];  % 法線ベクトル正規化
    targetFaceNorm_3mat = repmat(targetFaceNorm_2mat, [1, 1, tipNum]);
    % ターゲット頂点からロボアームエンドエフェクター中心までの位置ベクトル
    vert2endEffecCent_3mat = endEffecPos_3mat - vertBases;               % 3*6*n
    % targetSideVec = targetTipsPos(:, [2, 3, 4, 1], :) - targetTipsPos;      % 辺ベクトル計算
    % targetSideDire = targetSideVec ./ [targetDepth, targetWidth, targetDepth, targetWidth];     % 辺ベクトル正規化

    % 内積計算により接触判定
    face2endEffecCentDistance = dot(vert2endEffecCent_3mat, targetFaceNorm_3mat);   % 1*6*n
    face2endEffecCircuitDistance = face2endEffecCentDistance + r;                   % 1*6*n
    isImmerseFace = (face2endEffecCircuitDistance > 0);     % 先端球の表面の，ターゲット表面平行な平面へのめり込み状態 1*6*n
    mayContact = all(isImmerseFace, 2);                     % 必要条件 1*1*n
                                                            % mayContact(:,:,i)はarmTip(i)の接触状況を示す.
                                                        
    
    % 外積計算により接触判定
    % side2endEffecCent = cross(tip2EndEffecCent, targetSideDire, 1);         % 端球中心と辺直線の距離計算 3*4*4 内向き
    % side2endEffecCircuit = side2endEffecCent(3, :, :) + r;                  % 端球の周と辺直線の距離計算 1*4*4 内向き
    % isPenetSide = (side2endEffecCircuit > 0);                               % 端球の周の辺直線へのめり込み判定 1*4*4
    % mayContact = all(isPenetSide, 2);                                       % 全ての辺直線に対して条件を満たしていたら接触可能性有　1*1*4
                                                                            

    % 全ての先端球が接触していなければ，接触力は0
    % 接触可能性がない場合，後の計算をスキップ
    % if ~any(mayContact)
    %     isContact = zeros(1, tipNum);            % 非接触
    %     edgeWrench = zeros(6,tipNum);            % robo外力０
    %     targetWrench = zeros(6,1);          % ターゲット外力０
    %     return
    % end

    %% 接触可能性有り，詳しい判定
    % mayContact False のendEfecに注意
    isPenetFace = face2endEffecCentDistance > 0;        % 先端球中心の，ターゲット表面平行な平面へのめり込み状態 1*6*n
    notPenetFace = ~isPenetFace;
    % ターゲット辺で27に区切られた格子における存在状態 
    inFaceArea_tmp =  notPenetFace([1,2,5]) & notPenetFace([3,4,6]) & notPenetFace([5,5,1]) & notPenetFace([6,6,3]); % 1*3*n
    inFaceArea = any(inFaceArea_tmp, 2);                                                                % 面で作られる6空間 1*1*n
    isFaceContact = inFaceArea & mayContact;
    notFaceContact = ~inFaceArea & mayContact;
    if any(notFaceContact)
        fx = [5, 3, 6, 1];  % 直方体の面番号の
        fy = [4, 6, 2, 5];
        fz = 1:4;
        fx_shift = circshift(fx,1);
        fy_shift = circshift(fy,1);
        fz_shift = circshift(fz,1);
        
        inEdgeArea = [notPenetFace(:, fz, :) & notPenetFace(:, fz_shift, :) & notPenetFace(:, 5, :),...
                      notPenetFace(:, fz, :) & notPenetFace(:, fz_shift, :) & notPenetFace(:, 6, :)];   % 頂点付近の8空間 1*8*n     
        isEdgeContact = inEdgeArea & mayContact;
        notEdgeContact = ~inEdgeArea & mayContact;

        if any(notEdgeContact)
            
            inSideArea = [notPenetFace(:, fx, :) & notPenetFace(:, fx_shift, :) & isPenetFace(:, 2, :) & isPenetFace(:, 4, :),...
                          notPenetFace(:, fy, :) & notPenetFace(:, fy_shift, :) & isPenetFace(:, 1, :) & isPenetFace(:, 3, :),...
                          notPenetFace(:, fz, :) & notPenetFace(:, fz_shift, :) & isPenetFace(:, 5, :) & isPenetFace(:, 6, :)];     % 辺周りの12空間 1*12*n
            inSideArea(1, 3, 1) = true
            face_of_side_2mat = [fx,       fy,       fz      ;
                                 fx_shift, fy_shift, fz_shift];                     % 2*12
            face_of_side_3mat = repmat(face_of_side_2mat, [1,1,tipNum]);            % 2*12*n
            sideContactFace = zeros(2, 1, tipNum);                                  % 2*1*n
            sideContactFace(inSideArea) = face_of_side_3mat(:, inSideArea)                     % 2*1*n or Nan
            nearSide2endEffec_temp = face2endEffecCentDistance(:, sideContactFace) % 1*2*n or Nan
            nearSide2endEffec = vecnorm(nearSide2endEffec_temp, 2)
        
            sideVec = vertices(:, [2,3,7,6, 4,8,7,3, 5:8]) - vertices(:, [1,4,8,5, 1,5,6,2, 1:4]);                                          % 上記に対応する辺 3*12
            sideDire = sideVec ./ clone_vec([targetWidth, targetDepth, targetHeight], 4);
            contactSideDire = sideDire(inSideArea);
        end
        % inSideArea = side2endEffecCent(3, :, :) < 0;                            % 端球中心がターゲット辺と並行な領域にあるか判定 1*4*4
        % inEdgeArea = inSideArea & inSideArea(:, [4, 1, 2, 3], :);               % 端球中心がターゲット辺直線で作る格子の角領域にあるか判定 1*4*4
        dEndEfecCent2Tip = vecnorm(tip2EndEffecCent);                           % 端球中心からターゲット頂点までの距離 1*4*4
        isTipContact = any((dEndEfecCent2Tip < r) & inEdgeArea, 2);             % 頂点周りの半径rの円内で端球中心が接触か判定 1*1*4
        isSideContact = mayContact & ~any(inEdgeArea, 2);                       % 頂点まわり以外での接触か判定 1*1*4
        isContact = squeeze(isSideContact | isTipContact)';                     % 接触判定 1*4, isContact(:,:,i)はarmTip(i)の接触状況を示す.
    end
    
    % 全ての球が非接触．mayContactで判定しなかった頂点付近接触を判定
    if ~any(isContact)
        edgeWrench = zeros(6,4);
        targetWrench = zeros(6,1);
        return
    end
    
    % 最もめり込み量が小さい辺を接触辺とする
    
%     dXs = zeros(1,4,4);
%     dXs(:, :, isContact) = side2endEffecCircuit(:, :, isContact);           % endEfec中心からターゲット辺までの距離,非接触の場合０とする 1*4*4
%     [dX, contactSide] = min(dXs, [], 2);                                    % 最もめり込み量が小さい辺を接触辺とし，めり込み量計算 1*1*4
%                                                                             % contactSideは接触辺のインデックス 1*1*4
%     dX = squeeze(dX)';                                                      % 次元削除 1*4
% 
%     % 接触相対位置計算
%     targetSideNorm = cross(targetSideDire(:,:,1), targetZ);                 % ターゲット辺法線ベクトル，内向き 3*4
%     contactPos = endEffecPosLight + r * targetSideNorm(:, contactSide) ;    % 各端球接触位置 3*4
%     target2ContactPos = contactPos - targetR0;                              % ターゲット重心から接触点への相対位置 3*4 非接触点は後に除く
%     edge2ContactPos  =  contactPos - edgePos;                               % ロボ手先からの相対位置 3*4 非接触点に注意
% 
%     % 弾性項計算
%     elasTargetForces = contactElast * dX;                                   % 辺法線方向 1*4
% 
%     % 減衰項計算
%     targetContactVel = targetV0 + cross(targetW0, target2ContactPos);       % ターゲット接触点速度 3*4
%     velDiff = endeffecVel - targetContactVel;                               % 速度差計算 3*4
%     dV = dot(velDiff, targetSideNorm(:, contactSide));                      % 法線方向のめり込み速さ 1*4
%     dV(:, ~isContact) = 0;                                                  % 非接触点は０
%     dampTargetForces = contactDamp * dV;                                    % 減衰係数による力 1*4
% 
%     % 摩擦項計算
%     targetForcesN = elasTargetForces + dampTargetForces;                    % 法線方向成分力 1*4
%     targetForcesN(targetForcesN<0) = 0;                                     % 引き付ける力は発生させない
%     dVH = dot(velDiff, targetSideDire(:, contactSide));                     % 接線方向速度 1*4
%     velDiffSign = sign(dVH);                                                % 接線方向速度符号 1*4
%     targetForcesM = contactNu * targetForcesN .* velDiffSign;               % 接線方向力
% 
%     targetForces = targetForcesN .* targetSideNorm(:, contactSide) ...
%                  + targetForcesM .* targetSideDire(:, contactSide);         % 各種力合算 3*4
% 
%     roboForces  = -targetForces;                                            % ロボット手先力 3*4
%     roboTorques = cross(edge2ContactPos, -targetForces);                    % ロボトルク 3*4
%     tageTorques = cross(target2ContactPos, targetForces);                   % ターゲットトルク 3*4
% 
%     % roboForce = -targetForces(:, [1, 3]) - targetForces(:, [2, 4]);         % 手先合力 3*2
%     % roboTorque = roboTorques(:, [1, 3]) + roboTorques(:, [2, 4]);           % 手先合トルク 3*2
%     tageForce = sum(targetForces, 2);                                       % ターゲット合力 3*1
%     tageTorque = sum(tageTorques, 2);                                       % ターゲット合トルク 3*1
% 
%     edgeWrench = [roboForces; roboTorques];                                 % ロボ手先レンチ 6*4
%     targetWrench = [tageForce; tageTorque];                                 % ターゲットレンチ 6*1
%     % edgeWrench = zeros(6,4);
%     % targetWrench = zeros(6,1);
% end