function [ est_P, est_V, est_erP, NorTraj_12, est_Q, est_q, est_W, est_geo, est_half_side, est_r1, est_r2, est_phi1, est_phi2, est_R12, est_Theta12 ] ...
= Maru_Estimation_v01( d_time, time, Obs1, Obs2, Obs1_tmp, Obs2_tmp, NorTraj_12_tmp, est_P_tmp, est_Q_tmp, contactflag_L1, contactflag_L2, contactflag_R1, contactflag_R2, est_phi1, est_phi2, est_r1, est_r2, est_P, est_V, est_erP, NorTraj_12, est_Theta12 )

%%% 距離など
est_R12 = sqrt( ( Obs1(1,1) - Obs2(1,1) )^2 + ( Obs1(2,1) - Obs2(2,1) )^2 );   % ten1とten2の距離
est_half_side = est_R12 / 2;
est_geo = Obs1 + 1/sqrt(2) * rpy2dc( 0, 0, -pi/4 ) * ( Obs2 - Obs1 );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 重心位置の推定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if time >= d_time * 3

  if contactflag_L1 == 1 || contactflag_L2 == 1 || contactflag_R1 == 1 || contactflag_R2 == 1

  est_P = Obs1 + rpy2dc( 0, 0, -est_phi1 ) * ( Obs2 - Obs1 ) * est_r1 / est_R12;

  else     % 非接触の場合のみ，重心位置推定
  %%% 重心位置の推定

%%% 軌跡の法線の傾きと切片(tenとten_tmpの中点を通る法線)
an_Obs1_traj = -( Obs1(1,1) - Obs1_tmp(1,1) ) / ( Obs1(2,1) - Obs1_tmp(2,1) );
an_Obs2_traj = -( Obs2(1,1) - Obs2_tmp(1,1) ) / ( Obs2(2,1) - Obs2_tmp(2,1) );
bn_Obs1_traj = ( Obs1(2,1) + Obs1_tmp(2,1) ) / 2 - an_Obs1_traj * ( Obs1(1,1) + Obs1_tmp(1,1) ) / 2;
bn_Obs2_traj = ( Obs2(2,1) + Obs2_tmp(2,1) ) / 2 - an_Obs2_traj * ( Obs2(1,1) + Obs2_tmp(1,1) ) / 2;

%%% 法線の交点の座標を求める 直線1,2の交点
NorTraj_12(1,1) = -( bn_Obs2_traj - bn_Obs1_traj ) / ( an_Obs2_traj - an_Obs1_traj );
NorTraj_12(2,1) = ( an_Obs2_traj * bn_Obs1_traj - an_Obs1_traj * bn_Obs2_traj ) / ( an_Obs2_traj - an_Obs1_traj );
NorTraj_12(3,1) = 0;

%%% 推定速度，誤差
est_V_dash = ( NorTraj_12 - NorTraj_12_tmp )/d_time;

%%% ten_tmp→平行移動→ten_dash_est→回転→ten
Obs1_dash_est = Obs1_tmp + est_V_dash * d_time;
Obs2_dash_est = Obs2_tmp + est_V_dash * d_time;


%%% 弦の法線の傾きと切片(tenとten_dash_estの中点を通る法線)
a_Obs1_gen_normal = -( Obs1(1,1) - Obs1_dash_est(1,1) ) / ( Obs1(2,1) - Obs1_dash_est(2,1) );
a_Obs2_gen_normal = -( Obs2(1,1) - Obs2_dash_est(1,1) ) / ( Obs2(2,1) - Obs2_dash_est(2,1) );
b_Obs1_gen_normal = ( Obs1(2,1) + Obs1_dash_est(2,1) ) / 2 - a_Obs1_gen_normal * ( Obs1(1,1) + Obs1_dash_est(1,1) ) / 2;
b_Obs2_gen_normal = ( Obs2(2,1) + Obs2_dash_est(2,1) ) / 2 - a_Obs2_gen_normal * ( Obs2(1,1) + Obs2_dash_est(1,1) ) / 2;

%%% 弦の法線の交点(円の推定座標，誤差)
est_P(1,1) = -( b_Obs2_gen_normal - b_Obs1_gen_normal ) / ( a_Obs2_gen_normal - a_Obs1_gen_normal );
est_P(2,1) = ( a_Obs2_gen_normal * b_Obs1_gen_normal - a_Obs1_gen_normal * b_Obs2_gen_normal ) / ( a_Obs2_gen_normal - a_Obs1_gen_normal );
est_P(3,1) = 0;


%%% 推定半径，推定角速度，推定角度，その誤差
est_r1 = sqrt( ( est_P(1,1) - Obs1(1,1) )^2 + ( est_P(2,1) - Obs1(2,1) )^2 );
est_r2 = sqrt( ( est_P(1,1) - Obs2(1,1) )^2 + ( est_P(2,1) - Obs2(2,1) )^2 );


%%% 余弦定理から推定重心，点1,2の三角形内の角度を推定
est_Theta12 = acos( ( est_r1^2 + est_r2^2 - est_R12^2 ) / ( 2 * est_r1 * est_r2 ) );
est_phi1 = acos( ( est_R12^2 + est_r1^2 - est_r2^2 ) / ( 2 * est_r1 * est_R12 ) );
est_phi2 = acos( ( est_R12^2 + est_r2^2 - est_r1^2 ) / ( 2 * est_r2 * est_R12 ) );

%%% 幾何中心からの重心の距離
est_erP = sqrt( ( est_P(1,1) - est_geo(1,1) )^2 + ( est_P(2,1) - est_geo(2,1) )^2 );

  end
  
end


r = sqrt( ( est_P(1,1) - est_geo(1,1) )^2 + ( est_P(2,1) - est_geo(2,1) )^2 );
if  Obs1_tmp(1,1) == Obs1(1,1) || Obs2_tmp(1,1) == Obs2(1,1) || Obs1_tmp(2,1) == Obs1(2,1) || Obs2_tmp(2,1) == Obs2(2,1) || ( est_half_side * sqrt(2) ) < r
  est_P = est_geo;
end


%%% 推定速度，誤差
est_V = ( est_P - est_P_tmp )/d_time;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 角度の推定 %%%%%%%%%%%%%%%%%%%%   % 後でatan2()で
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% est_Q は(3,1)の角度ベクトル (値は-π<Q<=π)
%%% ターゲットの推定角度   Obs1, Obs2が正方形左上，左下の点の場合
% if     Obs1(1,1) < Obs2(1,1)   % Obs1の方が左
%    if     Obs1(2,1) > Obs2(2,1)   % Obs1の方が上
%           Q = atan( abs( Obs2(1,1) - Obs1(1,1) ) / abs( Obs2(2,1) - Obs1(2,1) ) );
%    elseif Obs1(2,1) < Obs2(2,1)   % Obs1の方が下
%           Q = atan( abs( Obs2(1,1) - Obs1(1,1) ) / abs( Obs2(2,1) - Obs1(2,1) ) ) + pi/2;
%    else   % y座標が同じ
%           Q = pi/2;
%    end
% elseif Obs1(1,1) > Obs2(1,1)   % Obs1の方が右
%    if     Obs1(2,1) > Obs2(2,1)   % Obs1の方が上
%           Q = -atan( abs( Obs2(1,1) - Obs1(1,1) ) / abs( Obs2(2,1) - Obs1(2,1) ) );
%    elseif Obs1(2,1) < Obs2(2,1)   % Obs1の方が下
%           Q = -atan( abs( Obs2(1,1) - Obs1(1,1) ) / abs( Obs2(2,1) - Obs1(2,1) ) ) - pi/2;
%    else   % y座標が同じ
%           Q = -pi/2;
%    end
% else   % x座標が同じ
%    if     Obs1(2,1) > Obs2(2,1)   % Obs1の方が上
%           Q = 0;
%    elseif Obs1(2,1) < Obs2(2,1)   % Obs1の方が下
%           Q = pi;
%    else   % 2観測点ダブってる
%    %%%%%%%%%%%%%%%%%%%%%%%%
%    end
% end
Q = atan2( ( Obs2(2,1) - Obs1(2,1) ), ( Obs2(1,1) - Obs1(1,1) ) );

est_Q(3,1) = Q;


%%% est_q0 は角度のスカラー (値は0<=q0<π/2)

if      pi/2 <= Q && Q < pi    % ターゲットがπ/2～πに傾くとき
    est_q = Q - pi/2;
elseif -pi/2 <= Q && Q < 0     % ターゲットが-π/2～0に傾くとき
    est_q = Q + pi/2;
elseif -pi <= Q && Q < -pi/2   % ターゲットが-π～-π/2に傾くとき
    est_q = Q + pi;
else                           % ターゲットが0～π/2のときはそのまま代入
    est_q = Q;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 角速度の推定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
est_W = 1/d_time * ( est_Q - est_Q_tmp );



end