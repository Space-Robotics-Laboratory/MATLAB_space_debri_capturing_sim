function [ contactflag_R, F_NN_R, FR_N_R, FR_T_R, norm_R, tang_R, FR_R, delta_R, deltaVel_R, delta_tmp_R, TB0_R, DB0_R, curPosAP33_R, curPosAP3_delta_R, curPosAP3_tmp_R, curPosBP33_R, PointC_R, l_x_R ] ...
= Shikaku_FourTips_RIGHT_CONTACT_v03( SV_d, SV_ts, d_time, half_side, r_tip, contact_flag_R, contactflag_R, shikaku1, shikaku2, shikaku3, shikaku4, ...
                        l_side_min, l_corner_min, ts_geo, q0, Rg_d, POS_eR, delta_tmp_R, curPosAP3_tmp_R, l1, l2, l3, l4, gamma1, gamma2, gamma3, gamma4, cof_R, kkk, ccc )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 右側 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% 手先が領域11,22,33,44にあるとき(角で接触しているとき) %%%%%%%%   % 手先球と頂点との接触を，仮想的に円との接触として考える

if     contact_flag_R(1,1) > 0  ||  contact_flag_R(2,2) > 0  ||  contact_flag_R(3,3) > 0  ||  contact_flag_R(4,4) > 0

    if     contact_flag_R(1,1) > 0
        shikaku_R = shikaku1;   % 角のベクトルをshikaku_Rとする   % 右手先に一番近い頂点
        gamma_R = gamma1;
        l_tsR0 = l1;
    elseif contact_flag_R(2,2) > 0
        shikaku_R = shikaku2;   % 角のベクトルをshikaku_Rとする
        gamma_R = gamma2;
        l_tsR0 = l2;
    elseif contact_flag_R(3,3) > 0
        shikaku_R = shikaku3;   % 角のベクトルをshikaku_Rとする
        gamma_R = gamma3;
        l_tsR0 = l3;
    elseif contact_flag_R(4,4) > 0
        shikaku_R = shikaku4;   % 角のベクトルをshikaku_Rとする
        gamma_R = gamma4;
        l_tsR0 = l4;
    end

   r_R = sqrt( ( shikaku_R(1,1) - POS_eR(1,1) )^2 + ( shikaku_R(2,1) - POS_eR(2,1) )^2 );   % ターゲット中心から手先球中心までの距離
   l_x_R =  0.0;
% めり込み量設定
   delta_R = abs( r_tip - r_R );   % 正値
% めり込み速度設定
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_R = delta_R;   % 更新
% 接触面角度
    if shikaku_R(1,1) == POS_eR(1,1)   % 手先球中心とターゲット中心が同じx座標のとき
      if shikaku_R(2,1) > POS_eR(2,1)   % 手先球の真上にターゲットがある場合
          theta_R_yaw = -pi/2;   % 右に右手先，左にターゲットがある状態を角度0として，接触面は-π/2
          norm_R = [ cos(-pi/2) sin(-pi/2) 0 ]';   %法線単位ベクトル(ターゲット外側向き，手先が受ける法線力の向き) [0 -1 0]
      else                                 % 手先球の真下にターゲットがある場合
          theta_R_yaw = pi/2;   % 右に右手先，左にターゲットがある状態を角度0として，接触面はπ/2
          norm_R = [ cos(pi/2) sin(pi/2) 0 ]';   % [0 1 0]
      end
    else   % 手先球中心とターゲット中心が異なるx座標とき
          theta_R_yaw = atan( ( shikaku_R(2,1) - POS_eR(2,1) ) / ( shikaku_R(1,1) - POS_eR(1,1) ) );   % 逆正接→角度導出  θ1'
          norm_R = [ cos(theta_R_yaw) sin(theta_R_yaw) 0 ]';
    end

% 慣性座標系からみた接触位置
    PointC_R = shikaku_R;
    curPosAP3_R = PointC_R;   % ターゲット用に保存
    curPosBP3_R = PointC_R;   % ロボット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_R = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_RR = TB0_R * curPosAP3_R;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_R = tilde( curPosAP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触面y方向の速度(ターゲット)
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_R = curPosAP3_RR;   % 更新
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_RR = DB0_R * curPosBP3_R;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_R = tilde( curPosBP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_R >= 0.0  ||  r_tip - r_R >= 0.0  ||  l_corner_min >= ( sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 ) - r_tip )  % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % 接触角度計算
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
       if F_NN_R < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
          contactflag_R = 0;
          F_NN_R = 0.0;   % 接触力 = 0
       end
  else   % めり込み量が負のとき
      contactflag_R = 0;
      F_NN_R = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_R >= 0.0  ||  r_tip - r_R >= 0.0  ||  l_corner_min >= ( sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 ) - r_tip )

    if     contact_flag_R(1,1) > 0
        psi_R = abs( pi/2 - abs( q0 + gamma_R - theta_R_yaw ) );
        corner_vel_R = l_tsR0 * SV_ts.w0(3,1);   % 四角の頂点の速度(ターゲット重心から頂点までの距離×ターゲットの角速度)
    elseif contact_flag_R(2,2) > 0
        psi_R = abs( q0 + gamma_R - theta_R_yaw );
        corner_vel_R = l_tsR0 * SV_ts.w0(3,1);   % 四角の頂点の速度(ターゲット重心から頂点までの距離×ターゲットの角速度)
    elseif contact_flag_R(3,3) > 0
        psi_R = abs( pi/2 - abs( q0 + gamma_R - theta_R_yaw ) );
        corner_vel_R = l_tsR0 * SV_ts.w0(3,1);   % 四角の頂点の速度(ターゲット重心から頂点までの距離×ターゲットの角速度)
    elseif contact_flag_R(4,4) > 0
        psi_R = abs( q0 + gamma_R - theta_R_yaw );
        corner_vel_R = l_tsR0 * SV_ts.w0(3,1);   % 四角の頂点の速度(ターゲット重心から頂点までの距離×ターゲットの角速度)
    end

   if     POS_eR(1,1) < SV_ts.R0(1,1)
       corner_plate_yvel_R = -corner_vel_R * cos( psi_R );% * cos( abs( theta_1_yaw ) );   % 四角の頂点の速度(接触面におけるside_velの速度)の接触面におけるy成分 (角速度が正なら四角の左側のy成分は負)
   elseif POS_eR(1,1) > SV_ts.R0(1,1)
       corner_plate_yvel_R =  corner_vel_R * cos( psi_R );% * cos( abs( theta_1_yaw ) );   % 四角の頂点の速度(接触面におけるside_velの速度)の接触面におけるy成分 (角速度が正なら四角の右側のy成分は正)
   end

     if curPosAP3_vel_R(2,1) + corner_plate_yvel_R == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_R = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_R = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_R(2,1) + corner_plate_yvel_R ) )' * norm_R;
     end
  else   % 接触していないとき
        contactflag_R = 0;   % 接触フラグ = 0
        tang_R = zeros(3,1);
  end



%%%%%%%% 手先が領域12,23,34,41にあるとき(面で接触しているとき) %%%%%%%%

elseif contact_flag_R(1,2) > 0  ||  contact_flag_R(2,3) > 0  ||  contact_flag_R(3,4) > 0  ||  contact_flag_R(4,1) > 0


if     contact_flag_R(1,2) > 0

   l_R = sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 );   % ターゲット中心から右手先球中心までの距離 正
    if     0 < q0  &&  q0 < pi/4   % この分岐内容が接触領域によって変わります
        if     POS_eR(2,1) > ts_geo(2,1)
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) + q0 );   % ターゲット中心位置と手先中心位置を結ぶ斜辺と，ターゲット中心を通りターゲット四角形の上下辺に平行な直線が作る底辺を用いた，直角三角形の鋭角φ
        elseif POS_eR(2,1) < ts_geo(2,1)
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - q0 );
        else
            phi_R = q0;
        end
    elseif pi/4 < q0  &&  q0 < pi/2
        if     POS_eR(1,1) < ts_geo(1,1)
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - q0 );
        elseif POS_eR(1,1) > ts_geo(1,1)
            phi_R = abs( pi - acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - q0 );
        else
            phi_R = abs( pi/2 - q0 );
        end
    else   % q0 = π/4のとき
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - pi/4 );
    end
   l_x_R = l_R * cos( phi_R );   % ターゲット重心座標における右手先球中心のx座標の絶対値

% めり込み量設定
   delta_R = abs( r_tip + half_side - l_x_R );   % 正
% めり込み速度設定
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_R = delta_R;   % 更新
% 接触面角度
%    theta_R_yaw = q0 - pi;
   norm_R = [ -cos(q0), -sin(q0), 0 ]';
% 慣性座標系からみた接触位置
    PointC_R = POS_eR + (r_tip - delta_R) * ( -norm_R );
    curPosAP3_R = PointC_R;   % ターゲット用に保存
    curPosBP3_R = PointC_R;   % ロボット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_R = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_RR = TB0_R * curPosAP3_R;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_R = tilde( curPosAP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_R = curPosAP3_RR;   % 更新
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_RR = DB0_R * curPosBP3_R;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_R = tilde( curPosBP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_R >= 0.0  ||  r_tip + half_side - l_x_R >= 0.0  ||  l_side_min >= l_x_R   % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % 接触角度計算
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
    if F_NN_R < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
      contactflag_R = 0;
      F_NN_R = 0.0;   % 接触力 = 0
    end
  else   % めり込み量が負のとき
      contactflag_R = 0;
      F_NN_R = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_R >= 0.0  &&  r_tip + half_side - l_x_R >= 0.0  &&  l_side_min >= l_x_R
     if curPosAP3_vel_R(2,1) == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_R = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_R = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_R(2,1) ) )' * norm_R;
     end
  else   % 接触していないとき
        contactflag_R = 0;   % 接触フラグ = 0
        tang_R = zeros(3,1);
  end


elseif contact_flag_R(2,3) > 0

   l_R = sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 );   % ターゲット中心から手先球中心までの距離 正
    if     0 < q0  &&  q0 < pi/4   % この分岐内容が接触領域によって変わります
        if     POS_eR(1,1) < ts_geo(1,1)
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) + q0 );   % ターゲット中心位置と手先中心位置を結ぶ斜辺と，ターゲット中心を通りターゲット四角形の上下辺に平行な直線が作る底辺を用いた，直角三角形の鋭角φ
        elseif POS_eR(1,1) > ts_geo(1,1)
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - q0 );
        else
            phi_R = q0;
        end
    elseif pi/4 < q0  &&  q0 < pi/2
        if     POS_eR(2,1) < ts_geo(2,1)
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - q0 );
        elseif POS_eR(2,1) > ts_geo(2,1)
            phi_R = abs( pi - acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - q0 );
        else
            phi_R = abs( pi/2 - q0 );
        end
    else   % q0 = π/4のとき
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - pi/4 );
    end
   l_x_R = l_R * cos( phi_R );   % ターゲット重心座標における左手先球中心のx座標の絶対値

% めり込み量設定
   delta_R = abs( r_tip + half_side - l_x_R );   % 正
% めり込み速度設定
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   %左側   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_R = delta_R;   % 更新
% 接触面角度
%    theta_R_yaw = q0 - pi/2;
   norm_R = [ sin(q0), -cos(q0), 0 ]';
% 慣性座標系からみた接触位置
    PointC_R = POS_eR + (r_tip - delta_R) * ( -norm_R );
    curPosAP3_R = PointC_R;   % ターゲット用に保存
    curPosBP3_R = PointC_R;   % ロボット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_R = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_RR = TB0_R * curPosAP3_R;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_R = tilde( curPosAP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_R = curPosAP3_RR;   % 更新
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_RR = DB0_R * curPosBP3_R;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_R = tilde( curPosBP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_R >= 0.0  ||  r_tip + half_side - l_x_R >= 0.0  ||  l_side_min >= l_x_R   % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % 接触角度計算
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
    if F_NN_R < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
      contactflag_R = 0;
      F_NN_R = 0.0;   % 接触力 = 0
    end
  else   % めり込み量が負のとき
      contactflag_R = 0;
      F_NN_R = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_R >= 0.0  &&  r_tip + half_side - l_x_R >= 0.0  &&  l_side_min >= l_x_R
     if curPosAP3_vel_R(1,1) == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_R = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_R = rpy2dc( 0, 0, -pi/2 * sign( curPosAP3_vel_R(1,1) ) )' * norm_R;
     end
  else   % 接触していないとき
        contactflag_R = 0;   % 接触フラグ = 0
        tang_R = zeros(3,1);
  end


elseif contact_flag_R(3,4) > 0

   l_R = sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 );   % ターゲット中心から手先球中心までの距離 正
    if     0 < q0  &&  q0 < pi/4   % この分岐内容が接触領域によって変わります
        if     POS_eR(2,1) < ts_geo(2,1)
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) + q0 );   % ターゲット中心位置と手先中心位置を結ぶ斜辺と，ターゲット中心を通りターゲット四角形の上下辺に平行な直線が作る底辺を用いた，直角三角形の鋭角φ
        elseif POS_eR(2,1) > ts_geo(2,1)
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - q0 );
        else
            phi_R = q0;
        end
    elseif pi/4 < q0  &&  q0 < pi/2
        if     POS_eR(1,1) > ts_geo(1,1)
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - q0 );
        elseif POS_eR(1,1) < ts_geo(1,1)
            phi_R = abs( pi - acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - q0 );
        else
            phi_R = abs( pi/2 - q0 );
        end
    else   % q0 = π/4のとき
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - pi/4 );
    end
   l_x_R = l_R * cos( phi_R );   % ターゲット重心座標における左手先球中心のx座標の絶対値

% めり込み量設定
   delta_R = abs( r_tip + half_side - l_x_R );   % 正
% めり込み速度設定
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   %左側   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_R = delta_R;   % 更新
% 接触面角度
%    theta_R_yaw = q0;
   norm_R = [ cos(q0), sin(q0), 0 ]';
% 慣性座標系からみた接触位置
    PointC_R = POS_eR + (r_tip - delta_R) * ( -norm_R );
    curPosAP3_R = PointC_R;   % ターゲット用に保存
    curPosBP3_R = PointC_R;   % ロボット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_R = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_RR = TB0_R * curPosAP3_R;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_R = tilde( curPosAP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_R = curPosAP3_RR;   % 更新
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_RR = DB0_R * curPosBP3_R;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_R = tilde( curPosBP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_R >= 0.0  ||  r_tip + half_side - l_x_R >= 0.0  ||  l_side_min >= l_x_R   % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % 接触角度計算
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
    if F_NN_R < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
      contactflag_R = 0;
      F_NN_R = 0.0;   % 接触力 = 0
    end
  else   % めり込み量が負のとき
      contactflag_R = 0;
      F_NN_R = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_R >= 0.0  &&  r_tip + half_side - l_x_R >= 0.0  &&  l_side_min >= l_x_R
     if curPosAP3_vel_R(2,1) == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_R = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_R = rpy2dc( 0, 0, -pi/2 * sign( curPosAP3_vel_R(2,1) ) )' * norm_R;
     end
  else   % 接触していないとき
        contactflag_R = 0;   % 接触フラグ = 0
        tang_R = zeros(3,1);
  end


elseif contact_flag_R(4,1) > 0

   l_R = sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 );   % ターゲット中心から手先球中心までの距離 正
    if     0 < q0  &&  q0 < pi/4   % この分岐内容が接触領域によって変わります
        if     POS_eR(1,1) > ts_geo(1,1)
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) + q0 );   % ターゲット中心位置と手先中心位置を結ぶ斜辺と，ターゲット中心を通りターゲット四角形の上下辺に平行な直線が作る底辺を用いた，直角三角形の鋭角φ
        elseif POS_eR(1,1) < ts_geo(1,1)
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - q0 );
        else
            phi_R = q0;
        end
    elseif pi/4 < q0  &&  q0 < pi/2
        if     POS_eR(2,1) > ts_geo(2,1)
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - q0 );
        elseif POS_eR(2,1) < ts_geo(2,1)
            phi_R = abs( pi - acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - q0 );
        else
            phi_R = abs( pi/2 - q0 );
        end
    else   % q0 = π/4のとき
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - pi/4 );
    end
   l_x_R = l_R * cos( phi_R );   % ターゲット重心座標における左手先球中心のx座標の絶対値

% めり込み量設定
   delta_R = abs( r_tip + half_side - l_x_R );   % 正
% めり込み速度設定
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   %左側   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_R = delta_R;   % 更新
% 接触面角度
%    theta_R_yaw = q0 + pi/2;
   norm_R = [ -sin(q0), cos(q0), 0 ]';
% 慣性座標系からみた接触位置
    PointC_R = POS_eR + (r_tip - delta_R) * ( -norm_R );
    curPosBP3_R = PointC_R;   % ロボット用に保存
    curPosAP3_R = PointC_R;   % ターゲット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_R = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_RR = TB0_R * curPosAP3_R;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_R = tilde( curPosAP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_R = curPosAP3_RR;   % 更新
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_RR = DB0_R * curPosBP3_R;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_R = tilde( curPosBP3_RR );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_R >= 0.0  ||  r_tip + half_side - l_x_R >= 0.0  ||  l_side_min >= l_x_R   % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % 接触角度計算
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
    if F_NN_R < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
      contactflag_R = 0;
      F_NN_R = 0.0;   % 接触力 = 0
    end
  else   % めり込み量が負のとき
      contactflag_R = 0;
      F_NN_R = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_R >= 0.0  &&  r_tip + half_side - l_x_R >= 0.0  &&  l_side_min >= l_x_R
     if curPosAP3_vel_R(1,1) == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_R = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_R = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_R(1,1) ) )' * norm_R;
     end
  else   % 接触していないとき
        contactflag_R = 0;   % 接触フラグ = 0
        tang_R = zeros(3,1);
  end

end


else   % 接触していないとき(一応)
     delta_R = 0;
     deltaVel_R = 0.0;
     l_x_R =  0.0;
     F_NN_R = 0.0;
     curPosAP33_R = zeros(3,3);
     curPosBP33_R = zeros(3,3);
     TB0_R = zeros(3,3);
     DB0_R = zeros(3,3);
     curPosAP3_delta_R = zeros(3,1);
     norm_R = zeros(3,1);
     tang_R = zeros(3,1);
     PointC_R = zeros(3,1);

end

FR_N_R = F_NN_R * norm_R;   % ロボット側が受ける法線力
FR_T_R = cof_R * F_NN_R * tang_R;   % 手先が受ける摩擦力
FR_R = FR_N_R + FR_T_R;

end