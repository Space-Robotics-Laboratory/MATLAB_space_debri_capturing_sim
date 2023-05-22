function [ contactflag_L, F_NN_L, FR_N_L, FR_T_L, norm_L, tang_L, FR_L, delta_L, deltaVel_L, delta_tmp_L, TB0_L, DB0_L, curPosAP33_L, curPosAP3_delta_L, curPosAP3_tmp_L, curPosBP33_L, PointC_L, l_x_L ] ...
= Shikaku_FourTips_LEFT_CONTACT_v03( LP_d, SV_d, SV_ts, d_time, half_side, r_tip, contact_flag_L, contactflag_L, shikaku1, shikaku2, shikaku3, shikaku4, ...
                        l_side_min, l_corner_min, ts_geo, q0, Rg_d, POS_eL, delta_tmp_L, curPosAP3_tmp_L, l1, l2, l3, l4, gamma1, gamma2, gamma3, gamma4, cof_L, kkk, ccc, jointsL, k_p, c_p )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 左側 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% 手先が領域11,22,33,44にあるとき(角で接触しているとき) %%%%%%%%   % 手先球と頂点との接触を，仮想的に円との接触として考える

if     contact_flag_L(1,1) > 0  ||  contact_flag_L(2,2) > 0  ||  contact_flag_L(3,3) > 0  ||  contact_flag_L(4,4) > 0

    if     contact_flag_L(1,1) > 0
        shikaku_L = shikaku1;   % 角のベクトルをshikaku_Lとする   % 左手先に一番近い頂点
        gamma_L = gamma1;
        l_tsR0 = l1;
    elseif contact_flag_L(2,2) > 0
        shikaku_L = shikaku2;   % 角のベクトルをshikaku_Lとする
        gamma_L = gamma2;
        l_tsR0 = l2;
    elseif contact_flag_L(3,3) > 0
        shikaku_L = shikaku3;   % 角のベクトルをshikaku_Lとする
        gamma_L = gamma3;
        l_tsR0 = l3;
    elseif contact_flag_L(4,4) > 0
        shikaku_L = shikaku4;   % 角のベクトルをshikaku_Lとする
        gamma_L = gamma4;
        l_tsR0 = l4;
    end

   r_L = sqrt( ( shikaku_L(1,1) - POS_eL(1,1) )^2 + ( shikaku_L(2,1) - POS_eL(2,1) )^2 );   % ターゲット中心から手先球中心までの距離
   l_x_L =  0.0;
% めり込み量設定
   delta_L = abs( r_tip - r_L );   % 正値
% めり込み速度設定
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_L = delta_L;   % 更新
% 接触面角度
    if shikaku_L(1,1) == POS_eL(1,1)   % 手先球中心とターゲット中心が同じx座標のとき    
      if shikaku_L(2,1) > POS_eL(2,1)   % 手先球の真上にターゲットがある場合
          theta_L_yaw = pi/2;   % 左に左手先，右にターゲットがある状態を角度0として，接触面はπ/2
          norm_L = [ cos(-pi/2) sin(-pi/2) 0 ]';   %法線単位ベクトル(ターゲット外側向き，手先が受ける法線力の向き) [0 -1 0]
      else                                 % 手先球の真下にターゲットがある場合
          theta_L_yaw = -pi/2;   % 左に左手先，右にターゲットがある状態を角度0として，接触面は-π/2
          norm_L = [ cos(pi/2) sin(pi/2) 0 ]';   % [0 1 0]
      end
    else   % 手先球中心とターゲット中心が異なるx座標とき
          theta_L_yaw = atan( ( shikaku_L(2,1) - POS_eL(2,1) ) / ( shikaku_L(1,1) - POS_eL(1,1) ) );   % 逆正接→角度導出  θ1'
          norm_L = [ cos(theta_L_yaw-pi) sin(theta_L_yaw-pi) 0 ]';
    end

% 慣性座標系からみた接触位置
    PointC_L = shikaku_L;
    curPosAP3_L = PointC_L;   % ターゲット用に保存
    curPosBP3_L = PointC_L;   % ロボット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_L = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_LL = TB0_L * curPosAP3_L;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_L = tilde( curPosAP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_L = curPosAP3_LL;   % 更新
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_LL = DB0_L * curPosBP3_L;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_L = tilde( curPosBP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_L >= 0.0  ||  r_tip - r_L >= 0.0  ||  l_corner_min >= ( sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 ) - r_tip )  % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % 左手
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % 接触角度計算
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
      %kw_L = kkk;
      %cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
       if F_NN_L < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
          contactflag_L = 0;
          F_NN_L = 0.0;   % 接触力 = 0
       end
  else   % めり込み量が負のとき
      contactflag_L = 0;
      F_NN_L = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_L >= 0.0  ||  r_tip - r_L >= 0.0  ||  l_corner_min >= ( sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 ) - r_tip )

    if     contact_flag_L(1,1) > 0
        psi_L = abs( pi/2 - abs( q0 + gamma_L - theta_L_yaw ) );
        corner_vel_L = l_tsR0 * SV_ts.w0(3,1);   % 四角の頂点の速度(ターゲット重心から頂点までの距離×ターゲットの角速度)
    elseif contact_flag_L(2,2) > 0
        psi_L = abs( q0 + gamma_L - theta_L_yaw );
        corner_vel_L = l_tsR0 * SV_ts.w0(3,1);   % 四角の頂点の速度(ターゲット重心から頂点までの距離×ターゲットの角速度)
    elseif contact_flag_L(3,3) > 0
        psi_L = abs( pi/2 - abs( q0 + gamma_L - theta_L_yaw ) );
        corner_vel_L = l_tsR0 * SV_ts.w0(3,1);   % 四角の頂点の速度(ターゲット重心から頂点までの距離×ターゲットの角速度)
    elseif contact_flag_L(4,4) > 0
        psi_L = abs( q0 + gamma_L - theta_L_yaw );
        corner_vel_L = l_tsR0 * SV_ts.w0(3,1);   % 四角の頂点の速度(ターゲット重心から頂点までの距離×ターゲットの角速度)
    end

   if     POS_eL(1,1) < SV_ts.R0(1,1)
       corner_plate_yvel_L = -corner_vel_L * cos( psi_L );% * cos( abs( theta_1_yaw ) );   % 四角の頂点の速度(接触面におけるside_velの速度)の接触面におけるy成分 (角速度が正なら四角の左側のy成分は負)
   elseif POS_eL(1,1) > SV_ts.R0(1,1)
       corner_plate_yvel_L =  corner_vel_L * cos( psi_L );% * cos( abs( theta_1_yaw ) );   % 四角の頂点の速度(接触面におけるside_velの速度)の接触面におけるy成分 (角速度が正なら四角の右側のy成分は正)
   end

     if curPosAP3_vel_L(2,1) + corner_plate_yvel_L == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_L = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_L = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_L(2,1) + corner_plate_yvel_L ) )' * norm_L;
     end
  else   % 接触していないとき
        contactflag_L = 0;   % 接触フラグ = 0
        tang_L = zeros(3,1);
  end



%%%%%%%% 手先が領域12,23,34,41にあるとき(面で接触しているとき) %%%%%%%%

elseif contact_flag_L(1,2) > 0  ||  contact_flag_L(2,3) > 0  ||  contact_flag_L(3,4) > 0  ||  contact_flag_L(4,1) > 0


if     contact_flag_L(1,2) > 0

   l_L = sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 );   % ターゲット中心から左手先球中心までの距離 正
    if     0 < q0  &&  q0 < pi/4   % この分岐内容が接触領域によって変わります
        if     POS_eL(2,1) > ts_geo(2,1)
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) + q0 );   % ターゲット中心位置と手先中心位置を結ぶ斜辺と，ターゲット中心を通りターゲット四角形の上下辺に平行な直線が作る底辺を用いた，直角三角形の鋭角φ
        elseif POS_eL(2,1) < ts_geo(2,1)
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - q0 );
        else
            phi_L = q0;
        end
    elseif pi/4 < q0  &&  q0 < pi/2
        if     POS_eL(1,1) < ts_geo(1,1)
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - q0 );
        elseif POS_eL(1,1) > ts_geo(1,1)
            phi_L = abs( pi - acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - q0 );
        else
            phi_L = abs( pi/2 - q0 );
        end
    else   % q0 = π/4のとき
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - pi/4 );
    end
   l_x_L = l_L * cos( phi_L );   % ターゲット重心座標における左手先球中心のx座標の絶対値

% めり込み量設定
   delta_L = abs( r_tip + half_side - l_x_L );   % 正
% めり込み速度設定
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_L = delta_L;   % 更新
% 接触面角度
    theta_L_yaw = q0;
   norm_L = [ -cos(q0), -sin(q0), 0 ]';
% 慣性座標系からみた接触位置
    PointC_L = POS_eL + (r_tip - delta_L) * ( -norm_L );
    curPosAP3_L = PointC_L;   % ターゲット用に保存
    curPosBP3_L = PointC_L;   % ロボット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_L = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_LL = TB0_L * curPosAP3_L;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_L = tilde( curPosAP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_L = curPosAP3_LL;   % 更新
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_LL = DB0_L * curPosBP3_L;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_L = tilde( curPosBP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_L >= 0.0  ||  r_tip + half_side - l_x_L >= 0.0  ||  l_side_min >= l_x_L   % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % 左手
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % 接触角度計算
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
%      kw_L = kkk;
%      cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
    if F_NN_L < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
      contactflag_L = 0;
      F_NN_L = 0.0;   % 接触力 = 0
    end
  else   % めり込み量が負のとき
      contactflag_L = 0;
      F_NN_L = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_L >= 0.0  &&  r_tip + half_side - l_x_L >= 0.0  &&  l_side_min >= l_x_L
     if curPosAP3_vel_L(2,1) == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_L = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_L = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_L(2,1) ) )' * norm_L;
     end
  else   % 接触していないとき
        contactflag_L = 0;   % 接触フラグ = 0
        tang_L = zeros(3,1);
  end


elseif contact_flag_L(2,3) > 0

   l_L = sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 );   % ターゲット中心から手先球中心までの距離 正
    if     0 < q0  &&  q0 < pi/4   % この分岐内容が接触領域によって変わります
        if     POS_eL(1,1) < ts_geo(1,1)
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) + q0 );   % ターゲット中心位置と手先中心位置を結ぶ斜辺と，ターゲット中心を通りターゲット四角形の上下辺に平行な直線が作る底辺を用いた，直角三角形の鋭角φ
        elseif POS_eL(1,1) > ts_geo(1,1)
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - q0 );
        else
            phi_L = q0;
        end
    elseif pi/4 < q0  &&  q0 < pi/2
        if     POS_eL(2,1) < ts_geo(2,1)
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - q0 );
        elseif POS_eL(2,1) > ts_geo(2,1)
            phi_L = abs( pi - acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - q0 );
        else
            phi_L = abs( pi/2 - q0 );
        end
    else   % q0 = π/4のとき
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - pi/4 );
    end
   l_x_L = l_L * cos( phi_L );   % ターゲット重心座標における左手先球中心のx座標の絶対値

% めり込み量設定
   delta_L = abs( r_tip + half_side - l_x_L );   % 正
% めり込み速度設定
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   %左側   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_L = delta_L;   % 更新
% 接触面角度
    theta_L_yaw = q0 + pi/2;
   norm_L = [ sin(q0), -cos(q0), 0 ]';
% 慣性座標系からみた接触位置
    PointC_L = POS_eL + (r_tip - delta_L) * ( -norm_L );
    curPosAP3_L = PointC_L;   % ターゲット用に保存
    curPosBP3_L = PointC_L;   % ロボット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_L = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_LL = TB0_L * curPosAP3_L;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_L = tilde( curPosAP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_L = curPosAP3_LL;   % 更新
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_LL = DB0_L * curPosBP3_L;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_L = tilde( curPosBP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_L >= 0.0  ||  r_tip + half_side - l_x_L >= 0.0  ||  l_side_min >= l_x_L   % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % 左手
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % 接触角度計算
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
%      kw_L = kkk;
%      cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
    if F_NN_L < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
      contactflag_L = 0;
      F_NN_L = 0.0;   % 接触力 = 0
    end
  else   % めり込み量が負のとき
      contactflag_L = 0;
      F_NN_L = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_L >= 0.0  &&  r_tip + half_side - l_x_L >= 0.0  &&  l_side_min >= l_x_L
     if curPosAP3_vel_L(1,1) == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_L = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_L = rpy2dc( 0, 0, -pi/2 * sign( curPosAP3_vel_L(1,1) ) )' * norm_L;
     end
  else   % 接触していないとき
        contactflag_L = 0;   % 接触フラグ = 0
        tang_L = zeros(3,1);
  end


elseif contact_flag_L(3,4) > 0

   l_L = sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 );   % ターゲット中心から手先球中心までの距離 正
    if     0 < q0  &&  q0 < pi/4   % この分岐内容が接触領域によって変わります
        if     POS_eL(2,1) < ts_geo(2,1)
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) + q0 );   % ターゲット中心位置と手先中心位置を結ぶ斜辺と，ターゲット中心を通りターゲット四角形の上下辺に平行な直線が作る底辺を用いた，直角三角形の鋭角φ
        elseif POS_eL(2,1) > ts_geo(2,1)
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - q0 );
        else
            phi_L = q0;
        end
    elseif pi/4 < q0  &&  q0 < pi/2
        if     POS_eL(1,1) > ts_geo(1,1)
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - q0 );
        elseif POS_eL(1,1) < ts_geo(1,1)
            phi_L = abs( pi - acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - q0 );
        else
            phi_L = abs( pi/2 - q0 );
        end
    else   % q0 = π/4のとき
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - pi/4 );
    end
   l_x_L = l_L * cos( phi_L );   % ターゲット重心座標における左手先球中心のx座標の絶対値

% めり込み量設定
   delta_L = abs( r_tip + half_side - l_x_L );   % 正
% めり込み速度設定
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   %左側   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_L = delta_L;   % 更新
% 接触面角度
    theta_L_yaw = q0 - pi;
   norm_L = [ cos(q0), sin(q0), 0 ]';
% 慣性座標系からみた接触位置
    PointC_L = POS_eL + (r_tip - delta_L) * ( -norm_L );
    curPosAP3_L = PointC_L;   % ターゲット用に保存
    curPosBP3_L = PointC_L;   % ロボット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_L = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_LL = TB0_L * curPosAP3_L;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_L = tilde( curPosAP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_L = curPosAP3_LL;   % 更新
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_LL = DB0_L * curPosBP3_L;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_L = tilde( curPosBP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_L >= 0.0  ||  r_tip + half_side - l_x_L >= 0.0  ||  l_side_min >= l_x_L   % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % 左手
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % 接触角度計算
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
%      kw_L = kkk;
%      cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
    if F_NN_L < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
      contactflag_L = 0;
      F_NN_L = 0.0;   % 接触力 = 0
    end
  else   % めり込み量が負のとき
      contactflag_L = 0;
      F_NN_L = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_L >= 0.0  &&  r_tip + half_side - l_x_L >= 0.0  &&  l_side_min >= l_x_L
     if curPosAP3_vel_L(2,1) == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_L = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_L = rpy2dc( 0, 0, -pi/2 * sign( curPosAP3_vel_L(2,1) ) )' * norm_L;
     end
  else   % 接触していないとき
        contactflag_L = 0;   % 接触フラグ = 0
        tang_L = zeros(3,1);
  end


elseif contact_flag_L(4,1) > 0

   l_L = sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 );   % ターゲット中心から手先球中心までの距離 正
    if     0 < q0  &&  q0 < pi/4   % この分岐内容が接触領域によって変わります
        if     POS_eL(1,1) > ts_geo(1,1)
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) + q0 );   % ターゲット中心位置と手先中心位置を結ぶ斜辺と，ターゲット中心を通りターゲット四角形の上下辺に平行な直線が作る底辺を用いた，直角三角形の鋭角φ
        elseif POS_eL(1,1) < ts_geo(1,1)
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - q0 );
        else
            phi_L = q0;
        end
    elseif pi/4 < q0  &&  q0 < pi/2
        if     POS_eL(2,1) > ts_geo(2,1)
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - q0 );
        elseif POS_eL(2,1) < ts_geo(2,1)
            phi_L = abs( pi - acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - q0 );
        else
            phi_L = abs( pi/2 - q0 );
        end
    else   % q0 = π/4のとき
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - pi/4 );
    end
   l_x_L = l_L * cos( phi_L );   % ターゲット重心座標における左手先球中心のx座標の絶対値

% めり込み量設定
   delta_L = abs( r_tip + half_side - l_x_L );   % 正
% めり込み速度設定
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   %左側   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_L = delta_L;   % 更新
% 接触面角度
    theta_L_yaw = q0 - pi/2;
   norm_L = [ -sin(q0), cos(q0), 0 ]';
% 慣性座標系からみた接触位置
    PointC_L = POS_eL + (r_tip - delta_L) * ( -norm_L );
    curPosAP3_L = PointC_L;   % ターゲット用に保存
    curPosBP3_L = PointC_L;   % ロボット用に保存
% ターゲット重心座標系から見た接触位置
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_L = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_LL = TB0_L * curPosAP3_L;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_L = tilde( curPosAP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_L = curPosAP3_LL;   % 更新
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_LL = DB0_L * curPosBP3_L;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_L = tilde( curPosBP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_L >= 0.0  ||  r_tip + half_side - l_x_L >= 0.0  ||  l_side_min >= l_x_L   % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % 左手
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % 接触角度計算
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
%      kw_L = kkk;
%      cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
    if F_NN_L < 0.0   % 法線方向の力が負の場合(吸引力は発生していないので)
      contactflag_L = 0;
      F_NN_L = 0.0;   % 接触力 = 0
    end
  else   % めり込み量が負のとき
      contactflag_L = 0;
      F_NN_L = 0.0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_L >= 0.0  &&  r_tip + half_side - l_x_L >= 0.0  &&  l_side_min >= l_x_L
     if curPosAP3_vel_L(1,1) == 0.0   % 接触面y方向の速度(target)が 0 のとき 
        tang_L = zeros(3,1);
     else   % 接触面y方向の速度(target)が 0 ではないとき 接触面y方向の速度の符号によって摩擦力の符号も変える
        tang_L = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_L(1,1) ) )' * norm_L;
     end
  else   % 接触していないとき
        contactflag_L = 0;   % 接触フラグ = 0
        tang_L = zeros(3,1);
  end

end


else   % 接触していないとき(一応)
     delta_L = 0;
     deltaVel_L = 0.0;
     l_x_L =  0.0;
     F_NN_L = 0.0;
     curPosAP33_L = zeros(3,3);
     curPosBP33_L = zeros(3,3);
     TB0_L = zeros(3,3);
     DB0_L = zeros(3,3);
     curPosAP3_delta_L = zeros(3,1);
     norm_L = zeros(3,1);
     tang_L = zeros(3,1);
     PointC_L = zeros(3,1);

end

FR_N_L = F_NN_L * norm_L;   % ロボット側が受ける法線力
FR_T_L = cof_L * F_NN_L * tang_L;   % 手先が受ける摩擦力
FR_L = FR_N_L + FR_T_L;

end