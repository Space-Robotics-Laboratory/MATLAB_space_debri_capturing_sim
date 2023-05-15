function [ contactflag_L, F_NN_L, FR_N_L, FR_T_L, norm_L, tang_L, FR_L, delta_L, deltaVel_L, delta_tmp_L, TB0_L, DB0_L, curPosAP33_L, curPosAP3_delta_L, curPosAP3_tmp_L, curPosBP33_L, PointC_L, PointC_tmp_L ] ...
= Maru_FourTips_LEFT_CONTACT_v01( SV_d, SV_tm, d_time, r_target, r_tip, contact_flag_L, l_min, l_surf_L, tm_geo, Rg_d, POS_eL, delta_tmp_L, curPosAP3_tmp_L, PointC_tmp_L, cof, kkk, ccc )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 左側 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if     contact_flag_L > 0   % 接触していたら

% めり込み量設定
   delta_L = abs( r_tip - l_surf_L );   % 正値
% めり込み速度設定
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   % めり込み速度 = ( 現在のめり込み量 - 1ステップ前のめり込み量 )/微小時間
   delta_tmp_L = delta_L;   % 更新
% 接触面角度
    if tm_geo(1,1) == POS_eL(1,1)   % 手先球中心とターゲット中心が同じx座標のとき    
      if tm_geo(2,1) > POS_eL(2,1)   % 手先球の真上にターゲットがある場合
          theta_yaw = pi/2;   % 左に左手先，右にターゲットがある状態を角度0として，接触面はπ/2
          norm_L = [ cos(-theta_yaw) sin(-theta_yaw) 0 ]';   %法線単位ベクトル(ターゲット外側向き，手先が受ける法線力の向き) [0 -1 0]
      else                                 % 手先球の真下にターゲットがある場合
          theta_yaw = -pi/2;   % 左に左手先，右にターゲットがある状態を角度0として，接触面は-π/2
          norm_L = [ cos(-theta_yaw) sin(-theta_yaw) 0 ]';   % [0 1 0]
      end
    else   % 手先球中心とターゲット中心が異なるx座標とき
          theta_yaw = atan2( ( POS_eL(2,1) - tm_geo(2,1) ), ( POS_eL(1,1) - tm_geo(1,1) ) );   % 逆正接→角度導出  θ1'
          norm_L = [ cos(theta_yaw) sin(theta_yaw) 0 ]';
    end

% 慣性座標系からみた接触位置
    PointC_L = tm_geo + r_target * norm_L;
    curPosAP3_L = PointC_L;   % ターゲット用に保存
    curPosBP3_L = PointC_L;   % ロボット用に保存
    PointC_vel_L = ( PointC_L - PointC_tmp_L ) / d_time;   % 微分(速度)
    PointC_tmp_L = PointC_L;   % 更新
% ターゲット重心座標系から見た接触位置
    curPosAP3_L = curPosAP3_L - SV_tm.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
    TB0_L = rpy2dc( SV_tm.Q0 );   % ターゲット回転角の方向余弦行列を作る
    curPosAP3_LL = TB0_L * curPosAP3_L;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33_L = tilde( curPosAP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% ターゲット重心座標系から見た接触位置の速度
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % 接触位置の差分 = 現在の接触位置 - 1ステップ前の接触位置
    curPosAP3_tmp_L = curPosAP3_LL;   % 更新
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % 微分(速度)
% ロボット全体の重心座標から見た位置
%     curPosBP3 = curPosBP3 - SV_d.R0;   % BPを使って慣性座標中心をロボット全体重心座標中心に替える
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ロボット回転角の方向余弦行列を作る
    curPosBP3_LL = DB0_L * curPosBP3_L;   % 接触位置をロボット重心座標に合わせ回転
    curPosBP33_L = tilde( curPosBP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
% 接触力の計算
  if delta_L >= 0  ||  r_tip - l_surf_L >= 0  ||  l_min >= ( sqrt( ( tm_geo(1,1) - POS_eL(1,1) )^2 + ( tm_geo(2,1) - POS_eL(2,1) )^2 ) - r_tip )  % めり込み量が正のとき(接触した瞬間からめり込んでいるとき)
      contactflag_L = 1;
%       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % 接触角度計算
%       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % θの多項式を用いて最適な k(剛性係数) を導出
%       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % 粘性減衰係数 = 0
      kw = kkk;
      cw = ccc;
      F_NN_L = kw * delta_L + cw * deltaVel_L;   % 接触面に対する法線方向の力をバネ・ダンパモデルで導出 正値? 
       if F_NN_L < 0   % 法線方向の力が負の場合(吸引力は発生していないので)
          contactflag_L = 0;
          F_NN_L = 0;   % 接触力 = 0
       end
  else   % めり込み量が負のとき
      contactflag_L = 0;
      F_NN_L = 0;   % 接触力は発生しない 接触力 = 0
  end
% 摩擦力の計算
  if delta_L >= 0  ||  r_tip - l_surf_L >= 0  ||  l_min >= ( sqrt( ( tm_geo(1,1) - POS_eL(1,1) )^2 + ( tm_geo(2,1) - POS_eL(2,1) )^2 ) - r_tip )

    r_L = sqrt( ( SV_tm.R0(1,1) - PointC_L(1,1) )^2 + ( SV_tm.R0(2,1) - PointC_L(2,1) )^2 );   % 重心位置から接触点までの距離
    tang_L = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_L(2,1) + r_L * SV_tm.w0(3,1) ) )' * norm_L;

  else   % 接触していないとき
        contactflag_L = 0;   % 接触フラグ = 0
        tang_L = zeros(3,1);
  end



else   % 接触していないとき(一応)
     contactflag_L = 0;
     delta_L = 0;
     deltaVel_L = 0;
     F_NN_L = 0;
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
FR_T_L = cof * F_NN_L * tang_L;   % 手先が受ける摩擦力
FR_L = FR_N_L + FR_T_L;


end