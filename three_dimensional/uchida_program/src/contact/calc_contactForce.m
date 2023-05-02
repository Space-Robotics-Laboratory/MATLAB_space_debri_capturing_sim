% 双腕ロボ手先球と直方体ターゲットの接触力計算関数
% 
% 2023.4 akiyohsi uchida
% 
% input  :DualArmRobo class, Targer class, contactElast, contactDamp
% output :[roboEndEffectorWrench 6*n, TagrgetWrench 6*1, contactState 1*n] ; Wrench = [Fource 3*1; Torque 3*1]
%
% 接触している辺の判定は，のめり込み量が最も小さい辺を接触辺とする
% ターゲット辺，エンドエフェクタを同時に扱うために多次元配列を用いて計算しており，
% repmatによる次元拡張が多用される.
% 
% 多次元配列は，dim1:xyz , dim2:targetVertices(face,side) , dim3:roboArmEndEfectorとして定義
%

function [endEffectorWrench, targetWrench, isContact] = calc_contactForce(dualArmRobo, target, param)
    contactElast = param.contact.elast;         % 弾性係数
    contactDamp = param.contact.damper;         % 減衰係数
    contactNu = param.contact.friction;         % 摩擦係数
    targetWidth = target.width;                      % ターゲット横
    targetDepth = target.depth;                      % ターゲット縦
    targetHeight = target.height;                     % ターゲット高
    m2G  = target.m2G;                           % ターゲット質量重心から幾何中心への相対位置ベクトル 3*1

    % ロボット末端位置代入（端球の中点） 
    leftEdgePos =  repmat(dualArmRobo.POS_e_L, [1,2]);              % 後の計算のために拡張 3*2
    rightEdgePos = repmat(dualArmRobo.POS_e_R, [1,2]);              % 後の計算のために拡張 3*2
    fulcrumPos = [leftEdgePos, rightEdgePos];                       % ロボトルク支点(ee中点) 3*4       

    % ターゲット位置・速度，角速度代入
    targetV0 = target.SV.v0;
    targetW0 = target.SV.w0;
    targetR0 = target.SV.R0;
    targetORI = target.SV.A0;

    % ロボット端球速度代入
    endEffecVel = [dualArmRobo.VEL_es_L, dualArmRobo.VEL_es_R];         % 3*4

    % ロボット端球位置代入，第三次元方向にアーム端球
    endEffecPos_2mat = [dualArmRobo.POS_es_L, dualArmRobo.POS_es_R];    % dim1:[xyz], dim2:roboArmTip 3*n
    endEffecPos_3mat_temp = repmat(endEffecPos_2mat, [1, 1, 12]);        % dim1:[xyz], dim2:roboArmTip, dim3:targetFaces(vert,side) 3*n*6
    endEffecPos_3mat = permute(endEffecPos_3mat_temp, [1, 3, 2]);       % dim1:[xyz], dim2:targetFaces(vert,side), dim3:roboArmTip 3*6*n
    [~, tipNum] = size(endEffecPos_2mat);
    % endEffecPosLight= [DualArmRobo.POS_es_L, DualArmRobo.POS_es_R];     % dim1:[xyz], dim2:roboArmTip 3*n 
    % endEffecPos_temp = repmat(endEffecPosLight, [1, 1, 4]);             % dim1:[xyz], dim2:roboArmTip, dim3:targetTip 3*4*n
    % endEffecPos = permute(endEffecPos_temp, [1, 3, 2]);                 % dim1:[xyz], dim2:targetTip , dim3:roboArmTip 3*4*n

    % 先端級半径設定
    r = dualArmRobo.r;

    % ターゲット頂点計算
    [vertices_t, ~] = calc_cubic(targetR0+m2G, targetWidth, targetDepth, targetHeight, targetORI);  % 8*3
    vertices = vertices_t'; % 3*8
  
    % ターゲット面法線計算（内向き）
    faceRootVert = [1, 2, 3, 4, 1, 5]; % 各面の始点頂点 3*6
    faceStemVert = [4, 1, 2, 3, 5, 1]; % 各面の終点頂点 3*6
    targetFaceNorm_temp = vertices(:, faceStemVert) - vertices(:, faceRootVert);   % 面法線ベクトル 3*6
    targetFaceNorm_2mat = targetFaceNorm_temp ./ [targetDepth, targetWidth, targetDepth, targetWidth, targetHeight, targetHeight];  % 法線ベクトル正規化 3*6
    targetFaceNorm_3mat = repmat(targetFaceNorm_2mat, [1, 1, tipNum]);  % 正規化各面法線ベクトル 3*6*n
    % 面の基準頂点からロボアームエンドエフェクター中心までの位置ベクトル
    vert2endEffecCent_3mat = endEffecPos_3mat(:, 1:8, :) - vertices;    % 頂点からエンドエフェクタ中心までのベクトル 3*8*n
    faceRootVert2endEffecCent_3mat = vert2endEffecCent_3mat(:, faceRootVert,:);              % 3*6*n

    %% 必要条件計算. 有限の面の前に，無限の面への接触を考える．
    % 内積計算により接触判定
    face2endEffecCentDistance = dot(faceRootVert2endEffecCent_3mat, targetFaceNorm_3mat);   % 各面から手先中心までの距離 1*6*n
    face2endEffecCircuitDistance = face2endEffecCentDistance + r;                           % 各面から手先周上までの距離 1*6*n
    isImmerseFace = (face2endEffecCircuitDistance > 0);     % 先端球の表面の，ターゲット表面平行な平面へのめり込み状態(無限面) 1*6*n
    mayContact = all(isImmerseFace, 2);                     % 必要条件 1*1*n
                                                            % mayContact(:,:,i)はarmTip(i)の接触状況を示す.
                                                              
    % 全ての先端球が接触していなければ，接触力は0
    % 接触可能性がない場合，後の計算をスキップ
    if ~any(mayContact) 
        isContact = zeros(1, tipNum);           % 非接触
        endEffectorWrench = zeros(6,tipNum);    % robo外力０
        targetWrench = zeros(6,1);              % ターゲット外力０
        return
    end

    %% 接触可能性有り，詳しい判定
    % mayContact False のendEfecに注意
    isPenetFace = face2endEffecCentDistance > 0;        % 先端球中心の，ターゲット表面平行な平面(無限面)へのめり込み状態 1*6*n
    notPenetFace = ~isPenetFace;
    % ターゲット辺で27に区切られた格子における存在状態 
    inFaceArea =  isPenetFace(:, [1,2,5], :) & isPenetFace(:, [3,4,6], :) & isPenetFace(:, [5,5,1], :) & isPenetFace(:, [6,6,3], :); % 1*3*n
    inFaceArea_sqd = any(inFaceArea, 2);            % 面で作られる6空間 1*1*n
    isFaceContact = inFaceArea_sqd & mayContact;    % 上空間内で接触しているか 1*1*n
    notFaceContact = ~inFaceArea_sqd & mayContact;  % 上空間内以外で接触 1*1*n
    isSideContact = false(1, 1, tipNum);
    isEdgeContact = false(1, 1, tipNum);

    % 面以外で接触している場合
    if any(notFaceContact) 
        % 頂点接触を考える
        fx = [5, 3, 6, 1];  % x軸周りの直方体の面番号
        fy = [4, 6, 2, 5];  % y軸周りの直方体の面番号
        fz = 1:4;           % z軸周りの直方体の面番号
        fx_shift = circshift(fx,1);
        fy_shift = circshift(fy,1);
        fz_shift = circshift(fz,1);
        
        inEdgeArea = [notPenetFace(:, fz, :) & notPenetFace(:, fz_shift, :) & notPenetFace(:, 5, :),...
                      notPenetFace(:, fz, :) & notPenetFace(:, fz_shift, :) & notPenetFace(:, 6, :)];   % 頂点付近の8空間 1*8*n   
        vert2endEffecCent_distant = vecnorm(vert2endEffecCent_3mat, 1);     % 頂点までの距離 1*8*n
        isEdgeContact_temp = (vert2endEffecCent_distant <= r) & inEdgeArea; % 頂点接触 1*8*n
        isEdgeContact = any(isEdgeContact_temp, 2);                         % 頂点接触 1*1*n
        notEdgeContact = ~any(isEdgeContact,2) & mayContact;                % 上空間内以外で接触している 1*1*n

        % 頂点付近以外で接触している場合
        if any(notEdgeContact) 
            % 辺接触を考える
            inSideArea = [notPenetFace(:, fx, :) & notPenetFace(:, fx_shift, :) & isPenetFace(:, 2, :) & isPenetFace(:, 4, :),...
                          notPenetFace(:, fy, :) & notPenetFace(:, fy_shift, :) & isPenetFace(:, 1, :) & isPenetFace(:, 3, :),...
                          notPenetFace(:, fz, :) & notPenetFace(:, fz_shift, :) & isPenetFace(:, 5, :) & isPenetFace(:, 6, :)];     % 辺周りの12空間 1*12*n
            face_of_side_2mat = [fx,       fy,       fz      ;
                                 fx_shift, fy_shift, fz_shift];                     % 2*12
            face_of_side_3mat = repmat(face_of_side_2mat, [1,1,tipNum]);            % 2*12*n
            sideLine2endEffecCent_3mat = zeros(2, 12, tipNum);                      % 2*12*n
            contactSideFace = face_of_side_3mat(:,inSideArea);                      % 2*contactSideNum
            sideLine2endEffecCent_3mat(:,inSideArea) = face2endEffecCentDistance(contactSideFace); % 辺からエンドエフェクター中心への距離 2*12*n

            side2endEffecCent_distance = vecnorm(sideLine2endEffecCent_3mat, 1);    % 1*12*n all plus
            isClose2sideLine = side2endEffecCent_distance <= r;                     % 辺を含む直線に手先が近い 1*12*n
            isSideContact_temp = isClose2sideLine & inSideArea;                     % 1*12*n
            isSideContact = any(isSideContact_temp, 2);                             % 1*1*n
        end
    end

    % 全ての条件を統合して接触判定
    isContact_temp = isFaceContact | isEdgeContact | isSideContact;     % 1*1*n
    isContact = squeeze(isContact_temp)';                                % 1*n
    
    % 全ての球が非接触．mayContactで判定しなかった頂点付近接触を判定
    if ~any(isContact) 
        endEffectorWrench = zeros(6,4);
        targetWrench = zeros(6,1);
        return
    end
    
    % 最もめり込み量が小さい面を接触面とする
    dXs = zeros(1, 6, tipNum);
    dXs(:, :, isContact) = face2endEffecCircuitDistance(:, :, isContact);   % endEfec中心からターゲット辺までの距離,非接触の場合０とする 1*6*n
    [dX_temp, contactFace] = min(dXs, [], 2);                               % 最もめり込み量が小さい辺を接触辺とし，めり込み量計算 1*1*n
    dX = squeeze(dX_temp)';                                                 % 次元削除 1*n

    % 弾性項計算
    elastTargetForces = contactElast * dX;                                  % 辺法線方向 1*n

    % 接触相対位置計算
    contactFaceNorm = targetFaceNorm_3mat(:, contactFace);
    contactPos = endEffecPos_2mat + r * contactFaceNorm;
    target2ContactPos = contactPos - targetR0;                              % ターゲット中心から接触点までの相対位置 3*n
    roboFulcrum2ContactPos  =  contactPos - fulcrumPos;                     % 手先支点から接触点までの相対位置 3*n

    % 減衰項計算
    targetW0_3mat = repmat(targetW0, [1,tipNum]);
    targetContactVel = targetV0 + cross(targetW0_3mat, target2ContactPos);  % ターゲット接触点速度 3*n
    velDiff = endEffecVel - targetContactVel;                               % 速度差計算 3*n
    velDiff(:, ~isContact) = 0;                                             % 非接触点は０
    dV = dot(velDiff, contactFaceNorm);                                     % 法線方向のめり込み速さ 1*n
    dampTargetForces = contactDamp * dV;                                    % 減衰係数による力 1*4

    % 摩擦項計算
    targetForcesN = elastTargetForces + dampTargetForces;                   % 法線方向成分力 1*n
    targetForcesN(targetForcesN<0) = 0;                                     % 引き付ける力は発生させない
    velDiff_surface = velDiff - dV .* contactFaceNorm;                      % 表面方向速度 3*n
    dVs = vecnorm(velDiff_surface);                                         % 表面方向速さ 1*n
    velDiff_sfc_sign = sign(dVs);                                           % 表面方向速度符号 1*n
    targetForces_s = contactNu * targetForcesN .* velDiff_sfc_sign;         % 表面方向力 3*n
    contactFaceVec = zeros(3, tipNum);
    contactFaceVec(isContact) =velDiff_surface(isContact) ./ dVs(isContact);% 表面方向単位vector 3*n

    targetForces = targetForcesN .* contactFaceNorm ...
                 + targetForces_s .* contactFaceVec;                        % 各種力合算 3*n

    roboForces  = -targetForces;                                            % ロボット手先力 3*4
    roboTorques = cross(roboFulcrum2ContactPos, roboForces);                % ロボトルク 3*4
    targetTorques = cross(target2ContactPos, targetForces);                 % ターゲットトルク 3*4

    % roboForce = -targetForces(:, [1, 3]) - targetForces(:, [2, 4]);         % 手先合力 3*2
    % roboTorque = roboTorques(:, [1, 3]) + roboTorques(:, [2, 4]);           % 手先合トルク 3*2
    tageForce = sum(targetForces, 2);                                       % ターゲット合力 3*1
    tageTorque = sum(targetTorques, 2);                                       % ターゲット合トルク 3*1

    endEffectorWrench = [roboForces; roboTorques];                          % ロボ手先レンチ 6*n
    targetWrench = [tageForce; tageTorque];                                 % ターゲットレンチ 6*1
    % edgeWrench = zeros(6,tipNum);
    % targetWrench = zeros(6,1);
% end