function [ xe_des, ve_des, ae_des, POS_j4_tmp, POS_j8_tmp, d_POS_j4, d_POS_j8, VEL_j4, VEL_j8, ...
           d_QL_3, d_QR_7, aL, aR, q_joints, q_wrist, qd_joints, qd_wrist, delta_t, delta_q, t0, Phase, catchtime, fff, d_geo ] ...
= Maru_DesiredHandMotion_v02( d_time, time, minus_time, SV_d, est_P, est_V, est_W, est_geo, ...
                              POS_j2, POS_j6, POS_j3, POS_j7, POS_j4, POS_j8, POS_j4_tmp, POS_j8_tmp, d_POS_j4, d_POS_j8,...
                              D, d_QL_3, d_QR_7, aL, aR, delta_t, delta_q, t0, contactflag_L1, contactflag_L2, contactflag_R1, contactflag_R2, ...
                              catchtime, POS_eL1, POS_eL2, POS_eR1, POS_eR2, hokakutime, d_geo, r_target )

fff = 0;
D = D * 1.000;

%%%%%% 関数の初期化 %%%%%%

% 関節角，角速度をそれぞれ異なるベクトルに分ける
    n = 8;
%     d_qd_joints = zeros(n,1);   % 目標関節角速度
    qd_joints = zeros(n,1);   % 関節角速度
    qd_wrist = zeros(n,1);   % 手先バネ機構角速度
    qd_joints(1:3,1) = SV_d.qd(1:3,1);   % 関節角速度を代入
    qd_joints(5:7,1) = SV_d.qd(5:7,1);   % 関節角速度を代入
    qd_wrist(4,1) =  SV_d.qd(4,1);   % 手先バネ機構角速度を代入
    qd_wrist(8,1) =  SV_d.qd(8,1);   % 手先バネ機構角速度を代入
    
    d_q_joints = zeros(n,1);   % 目標関節角度
    q_joints = zeros(n,1);   % 関節角度
    q_wrist = zeros(n,1);   % 手先バネ機構角度
    q_joints(1:3,1) = SV_d.q(1:3,1);   % 関節角度を代入
    q_joints(5:7,1) = SV_d.q(5:7,1);   % 関節角度を代入
    q_wrist(4,1) =  SV_d.q(4,1);   % 手先バネ機構角度を代入
    q_wrist(8,1) =  SV_d.q(8,1);   % 手先バネ機構角度を代入


% 関節4,8位置の速度
    VEL_j4 = ( POS_j4 - POS_j4_tmp ) / d_time;
    VEL_j8 = ( POS_j8 - POS_j8_tmp ) / d_time;

% % ターゲット頂点の設定
%     ten1 = Obs1;
%     ten2 = Obs2;
%     ten3 = 2 * est_geo - ten1;
%     ten4 = 2 * est_geo - ten2;

 POS_j2_st = 2 * POS_j3 - POS_j2;   % 関節3から見たとき関節2から丁度180°の点
 POS_j6_st = 2 * POS_j7 - POS_j6;   % 関節7から見たとき関節6から丁度180°の点

%  Q_pi_4 = est_Q(3,1) - pi/4;
%  Q_target = [ cos( Q_pi_4 ), sin( Q_pi_4 ), 0 ]';


%%% 目標位置，速度，加速度，角度，角速度，各加速度を設定 %%%


%%% Phase 1   アームは動かさない
if time < t0 + minus_time
  if time < t0 + minus_time && time >= minus_time
     Phase = 1;
  else
     Phase = 0;
  end

   d_POS_j4 = [ POS_j4' 0 ]';   % (3,1)
   d_POS_j8 = [ POS_j8' 0 ]';   % (3,1)
   d_q_joints = q_joints;   % 目標手先角度
   d_VEL_j4 = VEL_j4;   % (2,1)   % 目標手先速度   % (2,1)
   d_VEL_j8 = VEL_j8;   % (2,1)
   d_qd_joints = qd_joints;   % 目標手先角速度

 xeL_des = [ d_POS_j4' zeros(1,2) d_q_joints(3,1) ]';   % 目標手先位置角度
 xeR_des = [ d_POS_j8' zeros(1,2) d_q_joints(7,1) ]';
 veL_des = [ d_VEL_j4' zeros(1,3) d_qd_joints(3,1) ]';   % 目標手先速度角速度
 veR_des = [ d_VEL_j8' zeros(1,3) d_qd_joints(7,1) ]';
 aeL_des = zeros(6,1);   % 目標手先加速度角加速度
 aeR_des = zeros(6,1);

%%% Phase 2
elseif time > ( t0 + minus_time - d_time/2 ) && time < ( t0 + minus_time + d_time/2 )    % なぜかPhase2に入らないという現象のため，==ではなく不等式にした
Phase = 2;

% 重心位置と幾何中心の位置関係
if norm( est_geo - est_P ) < 1e-5
%    theta_com = 0;
   delta_q = 0;
   delta_t = 2;

else
   theta_com = atan2( ( est_geo(2,1) - est_P(2,1) ), ( est_geo(1,1) - est_P(1,1) ) );

 % 重心の丁度真上に幾何中心がくるまでの時間を計算
 if      est_W(3,1) > 0
      if theta_com > 0
%          delta_q = 3*pi/2 - theta_com;   %%% koko
         delta_q = pi/2 - theta_com;
         fff = 1;
      else
         delta_q = pi/2 - theta_com;
         fff = 2;
      end
   delta_t = abs( delta_q / est_W(3,1) );
 elseif  est_W(3,1) < 0
      if theta_com > 0
         delta_q = pi/2 + theta_com;
         fff = 3;
      else
         delta_q = 3*pi/2 + theta_com;
         fff = 4;
      end
   delta_t = abs( delta_q / est_W(3,1) );
 else   % 回転していないときは，2秒間で手先を移動
   delta_q = 0;
   delta_t = 2;
        fff = 0;
 end
%  if      est_W(3,1) > 0
%       if theta_com < 0
%          delta_q = pi/2 - theta_com;
%          fff = 1;
%       else
%          delta_q = pi - theta_com + 3/2*pi;
%          fff = 2;
%       end
%    delta_t = abs( delta_q / est_W(3,1) );
%  elseif  est_W(3,1) < 0
%       if theta_com < 0
%          delta_q = pi + theta_com + pi/2;
%          fff = 3;
%       else
%          delta_q = theta_com + 3/2*pi;
%          fff = 4;
%       end
%    delta_t = abs( delta_q / est_W(3,1) );
%  else   % 回転していないときは，2秒間で手先を移動
%    delta_q = 0;
%    delta_t = 2;
%         fff = 0;
%  end

end

%%% Δt後の幾何中心位置を計算
  d_geo = est_P + rpy2dc( 0, 0, delta_q )' * ( est_geo - est_P ) + delta_t * est_V;

  L = [ 1 0 0 ]';
  d_POS_j4 = d_geo - D * L;   % 目標手先位置
  d_POS_j8 = d_geo + D * L;
  d_QL_3 = atan2( ( est_geo(2,1) - POS_j3(2,1) ), ( est_geo(1,1) - POS_j3(1,1) ) );
  d_QR_7 = atan2( ( est_geo(2,1) - POS_j7(2,1) ), ( est_geo(1,1) - POS_j7(1,1) ) );

%   d_ten1 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten1 - est_P ) + est_V * delta_t;   % 各頂点のΔt後の位置を計算
%   d_ten2 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten2 - est_P ) + est_V * delta_t;
%   d_ten3 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten3 - est_P ) + est_V * delta_t;
%   d_ten4 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten4 - est_P ) + est_V * delta_t;

%   if     est_W(3,1) > 0   % ターゲット角速度が正のとき
%     d_POS_j4 = d_ten1 - D * L_target;   % 目標手先位置
%     d_POS_j8 = d_ten3 + D * L_target;
%     tenL = ten1;
%     tenR = ten3;
%       d_QL_3 = atan2( ( ten1(2,1) - POS_j3(2,1) ), ( ten1(1,1) - POS_j3(1,1) ) );
%       d_QR_7 = atan2( ( ten3(2,1) - POS_j7(2,1) ), ( ten3(1,1) - POS_j7(1,1) ) );
%     
%   elseif est_W(3,1) < 0   % ターゲット角速度が負のとき
%     d_POS_j4 = d_ten2 - D * L_target;   % 目標手先位置
%     d_POS_j8 = d_ten4 + D * L_target;
%     tenL = ten2;
%     tenR = ten4;
%       d_QL_3 = atan2( ( ten2(2,1) - POS_j3(2,1) ), ( ten2(1,1) - POS_j3(1,1) ) );
%       d_QR_7 = atan2( ( ten4(2,1) - POS_j7(2,1) ), ( ten4(1,1) - POS_j7(1,1) ) );
% 
%   else                    % ターゲット角速度が0のとき
%     d_POS_j4 = ( ten1 + ten2 ) / 2 - D_0 * L_target;   % 目標手先位置
%     d_POS_j8 = ( ten3 + ten4 ) / 2 + D_0 * L_target;
%       d_QL_3 = 0;
%       d_QR_7 = pi;
% 
%   end

  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
  d_q_joints(3,1) = kakuL;   % 目標手先角度
  d_q_joints(7,1) = kakuR;

  %   目標位置制御→スプライン曲線軌跡追従   とりあえず手先速度制御
  x0L = POS_j4;   % 初期位置   % (2,1)
  v0L = zeros(2,1);   % 初期速度
  xfL = d_POS_j4(1:2,1);   % 目標位置
  vfL = zeros(2,1);   % 目標速度
    a0L = x0L;   % 係数に代入
    a1L = v0L;
    a2L = ( -3 * ( x0L - xfL ) - delta_t * ( 2 * v0L + vfL ) )/( delta_t )^2;
    a3L = ( 2 * ( x0L - xfL ) - delta_t * ( v0L + vfL ) )/( delta_t )^3;
  aL = [ a0L, a1L, a2L, a3L ];

  x0R = POS_j8;   % 初期位置   % (2,1)
  v0R = zeros(2,1);   % 初期速度
  xfR = d_POS_j8(1:2,1);
  vfR = zeros(2,1);   % 目標速度
    a0R = x0R;   % 係数に代入
    a1R = v0R;
    a2R = ( -3 * ( x0R - xfR ) - delta_t * ( 2 * v0R + vfR ) )/( delta_t )^2;
    a3R = ( 2 * ( x0R - xfR ) - delta_t * ( v0R + vfR ) )/( delta_t )^3;
  aR = [ a0R, a1R, a2R, a3R ];

  x_L = a0L + a1L * ( time - ( t0 + minus_time ) ) + a2L * ( time - ( t0 + minus_time ) )^2 + a3L * ( time - ( t0 + minus_time ) )^3;   % スプライン曲線軌跡上の目標手先位置を計算
  x_R = a0R + a1R * ( time - ( t0 + minus_time ) ) + a2R * ( time - ( t0 + minus_time ) )^2 + a3R * ( time - ( t0 + minus_time ) )^3;

% % 角に垂直に
%   kL = kakudo( est_P, [ x_L' 0 ]', tenL );
%   kR = kakudo( est_P, [ x_R' 0 ]', tenR );
%   x_LL = est_P + rpy2dc( 0, 0, kL )' * ( [ x_L' 0 ]' - est_P );
%   x_RR = est_P + rpy2dc( 0, 0, kR )' * ( [ x_R' 0 ]' - est_P );
%   x_L = x_LL(1:2,1);
%   x_R = x_RR(1:2,1);

 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);



%%% Phase 3以降
elseif time > ( t0 + minus_time )  &&  time <= ( t0 + delta_t + minus_time )%  &&  contactflag_L1==0  &&  contactflag_L2==0  &&  contactflag_R1==0  &&  contactflag_R2==0   % && Phase == 3
Phase = 3;

%   if     est_W(3,1) > 0   % ターゲット角速度が正のとき
%     tenL = ten1;
%     tenR = ten3;
%      d_QL_3 = atan2( ( ten1(2,1) - POS_j3(2,1) ), ( ten1(1,1) - POS_j3(1,1) ) );
%      d_QR_7 = atan2( ( ten3(2,1) - POS_j7(2,1) ), ( ten3(1,1) - POS_j7(1,1) ) );
%     
%   elseif est_W(3,1) < 0   % ターゲット角速度が負のとき
%     tenL = ten2;
%     tenR = ten4;
%      d_QL_3 = atan2( ( ten2(2,1) - POS_j3(2,1) ), ( ten2(1,1) - POS_j3(1,1) ) );
%      d_QR_7 = atan2( ( ten4(2,1) - POS_j7(2,1) ), ( ten4(1,1) - POS_j7(1,1) ) );
% 
%   else                    % ターゲット角速度が0のとき
%     tenL = ( ten1 + ten2 ) / 2;
%     tenR = ( ten3 + ten4 ) / 2;
%      d_QL_3 = 0;
%      d_QR_7 = pi;
% 
%   end

%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j7' 0 ]' );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j3' 0 ]' );
  d_q_joints(3,1) = kakuL;   % 目標手先角度
  d_q_joints(7,1) = kakuR;

  x_L = aL(:,1) + aL(:,2) * ( time - ( t0 + minus_time ) ) + aL(:,3) * ( time - ( t0 + minus_time ) )^2 + aL(:,4) * ( time - t0 - minus_time )^3;
  x_R = aR(:,1) + aR(:,2) * ( time - ( t0 + minus_time ) ) + aR(:,3) * ( time - ( t0 + minus_time ) )^2 + aR(:,4) * ( time - t0 - minus_time )^3;

% % 角に垂直に
%   kL = kakudo( est_P, [ x_L' 0 ]', tenL );
%   kR = kakudo( est_P, [ x_R' 0 ]', tenR );
%   x_LL = est_P + rpy2dc( 0, 0, kL )' * ( [ x_L' 0 ]' - est_P );
%   x_RR = est_P + rpy2dc( 0, 0, kR )' * ( [ x_R' 0 ]' - est_P );
%   x_L = x_LL(1:2,1);
%   x_R = x_RR(1:2,1);

 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);


% %%% Phase 4→5に変更して動かさないことにする(v02)
% elseif time >= ( t0 + delta_t + minus_time )  ||  contactflag_L1==1  ||  contactflag_L2==1  ||  contactflag_R1==1  ||  contactflag_R2==1    ||    Phase==4  ||  Phase==5
% Phase = 5;
% 
% %   thetaTL = atan2( ( tenL(2,1) - est_geo(2,1) ), ( tenL(1,1) - est_geo(1,1) ) );   % ここ考える
% %   thetaTL = atan2( ( tenL(2,1) - tenR(2,1) ), ( tenL(1,1) - tenR(1,1) ) );   % ここ考える
% %   thetaTR = atan2( ( tenR(2,1) - tenL(2,1) ), ( tenR(1,1) - tenL(1,1) ) );
% %   thetaTL = atan2( ( POS_j4(2,1) - POS_j8(2,1) ), ( POS_j4(1,1) - POS_j8(1,1) ) );   % ここ考える
%   thetaTL = atan2( ( POS_j4(2,1) - est_geo(2,1) ), ( POS_j4(1,1) - est_geo(1,1) ) );   % ここ考える
%   thetaTR = atan2( ( POS_j8(2,1) - POS_j4(2,1) ), ( POS_j8(1,1) - POS_j4(1,1) ) );
%   x_L = est_geo(1:2,1) + D * [ cos(thetaTL), sin(thetaTL) ]';
% %   x_R = est_geo(1:2,1) + D * [ cos(thetaTR), sin(thetaTR) ]';
%   x_R = x_L + 2 * D * [ cos(thetaTR), sin(thetaTR) ]';
% 
% %   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j7' 0 ]' );
% %   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j3' 0 ]' );
% %   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j8' 0 ]' );
% %   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j4' 0 ]' );
% %   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ est_geo' 0 ]' );
% %   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ est_geo' 0 ]' );
% %   d_q_joints(3,1) = kakuL;   % 目標手先角度
% %   d_q_joints(7,1) = kakuR;

else

Phase = 5;


%%% 捕獲フェーズ Phase5   4点の中に推定幾何中心がcatchtime秒入っている

A = cross( ( est_geo - POS_eL2 ), ( POS_eL1 - est_geo ) );
B = cross( ( est_geo - POS_eR1 ), ( POS_eL2 - est_geo ) );
C = cross( ( est_geo - POS_eR2 ), ( POS_eR1 - est_geo ) );
D = cross( ( est_geo - POS_eL1 ), ( POS_eR2 - est_geo ) );
AA = norm( POS_eL1 - POS_eL2 );
BB = norm( POS_eR1 - POS_eR2 );
CC = norm( POS_eR1 - POS_eL2 );
DD = norm( POS_eL1 - POS_eR2 );

if A(3,1) > 0 && B(3,1) > 0 && C(3,1) > 0 && D(3,1) > 0 && AA < (2*r_target) && BB < (2*r_target) && CC < (2*r_target) && DD < (2*r_target)
   catchtime = catchtime + d_time;
     if catchtime >= hokakutime
        Phase = 5;
     end
else
   catchtime = 0;
end



% delta_t_2 = 2;
% 
% if catchtime == d_time
% 
% starttime = time;
% 
%   thetaT = atan2( ( POS_j8(2,1) - POS_j4(2,1) ), ( POS_j8(1,1) - POS_j4(1,1) ) );
% LL = [ cos( thetaT ), sin( thetaT ), 0 ]';
% d_POS_j4 = est_geo - D * LL;   % 目標手先位置
% d_POS_j8 = est_geo + D * LL;
% 
%   x0L = POS_j4;   % 初期位置   % (2,1)
%   v0L = VEL_j4(2,1);   % 初期速度
%   xfL = d_POS_j4(1:2,1);   % 目標位置
%   vfL = zeros(2,1);   % 目標速度
%     a0L = x0L;   % 係数に代入
%     a1L = v0L;
%     a2L = ( -3 * ( x0L - xfL ) - delta_t_2 * ( 2 * v0L + vfL ) )/( delta_t_2 )^2;
%     a3L = ( 2 * ( x0L - xfL ) - delta_t_2 * ( v0L + vfL ) )/( delta_t_2 )^3;
%   aL = [ a0L, a1L, a2L, a3L ];
% 
%   x0R = POS_j8;   % 初期位置   % (2,1)
%   v0R = zeros(2,1);   % 初期速度
%   xfR = d_POS_j8(1:2,1);
%   vfR = zeros(2,1);   % 目標速度
%     a0R = x0R;   % 係数に代入
%     a1R = v0R;
%     a2R = ( -3 * ( x0R - xfR ) - delta_t_2 * ( 2 * v0R + vfR ) )/( delta_t_2 )^2;
%     a3R = ( 2 * ( x0R - xfR ) - delta_t_2 * ( v0R + vfR ) )/( delta_t_2 )^3;
%   aR = [ a0R, a1R, a2R, a3R ];
% 
%   x_L = a0L + a1L * ( time - starttime ) + a2L * ( time - starttime )^2 + a3L * ( time - starttime )^3;   % スプライン曲線軌跡上の目標手先位置を計算
%   x_R = a0R + a1R * ( time - starttime ) + a2R * ( time - starttime )^2 + a3R * ( time - starttime )^3;
% 
% elseif catchtime > d_time && catchtime < ( d_time + delta_t_2 )
% 
%   x_L = a0L + a1L * ( time - starttime ) + a2L * ( time - starttime )^2 + a3L * ( time - starttime )^3;   % スプライン曲線軌跡上の目標手先位置を計算
%   x_R = a0R + a1R * ( time - starttime ) + a2R * ( time - starttime )^2 + a3R * ( time - starttime )^3;
% 
% end


  x_L = POS_j4;
  x_R = POS_j8 ;
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j7' 0 ]' );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j3' 0 ]' );
  d_q_joints(3,1) = kakuL;   % 目標手先角度
  d_q_joints(7,1) = kakuR;

 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);




% aaL1L2 = ( POS_eL1(2,1) - POS_eL2(2,1) )/( POS_eL1(1,1) - POS_eL2(1,1) );
% bbL1L2 = ( POS_eL2(2,1)*POS_eL1(1,1) - POS_eL1(2,1)*POS_eL2(1,1) )/( POS_eL1(1,1) - POS_eL2(1,1) );
% aaR1R2 = ( POS_eR1(2,1) - POS_eR2(2,1) )/( POS_eR1(1,1) - POS_eR2(1,1) );
% bbR1R2 = ( POS_eR2(2,1)*POS_eR1(1,1) - POS_eR1(2,1)*POS_eR2(1,1) )/( POS_eR1(1,1) - POS_eR2(1,1) );
% aaL1R2 = ( POS_eL1(2,1) - POS_eR2(2,1) )/( POS_eL1(1,1) - POS_eR2(1,1) );
% bbL1R2 = ( POS_eL2(2,1)*POS_eR1(1,1) - POS_eR1(2,1)*POS_eL2(1,1) )/( POS_eR1(1,1) - POS_eL2(1,1) );
% aaL2R1 = ( POS_eR1(2,1) - POS_eL2(2,1) )/( POS_eR1(1,1) - POS_eL2(1,1) );
% bbL2R1 = ( POS_eL2(2,1)*POS_eR1(1,1) - POS_eR1(2,1)*POS_eL2(1,1) )/( POS_eR1(1,1) - POS_eL2(1,1) );
% 
% if POS_j4(1,1) < POS_j8(1,1)
%    if POS_j4(2,1) < POS_j8(2,1)
%      if  est_geo(2,1)>(aaL1L2*est_geo(1,1)+bbL1L2)  &&  est_geo(2,1)<(aaR1R2*est_geo(1,1)+bbR1R2)  &&  est_geo(2,1)<(aaL1R2*est_geo(1,1)+bbL1R2)  &&  est_geo(2,1)>(aaL2R1*est_geo(1,1)+bbL2R1)  ||  Phase == 5
%          catchtime = catchtime + d_time;
%             if catchtime >= hokakutime
%                Phase = 5;
%             end
%      else
%          catchtime = 0;
%      end
%    else
%      if  est_geo(2,1)<(aaL1L2*est_geo(1,1)+bbL1L2)  &&  est_geo(2,1)>(aaR1R2*est_geo(1,1)+bbR1R2)  &&  est_geo(2,1)<(aaL1R2*est_geo(1,1)+bbL1R2)  &&  est_geo(2,1)>(aaL2R1*est_geo(1,1)+bbL2R1)  ||  Phase == 5
%          catchtime = catchtime + d_time;
%             if catchtime >= hokakutime
%                Phase = 5;
%             end
%      else
%          catchtime = 0;
%      end
%    end
% else
%    if POS_j4(2,1) < POS_j8(2,1)
%      if  est_geo(2,1)>(aaL1L2*est_geo(1,1)+bbL1L2)  &&  est_geo(2,1)<(aaR1R2*est_geo(1,1)+bbR1R2)  &&  est_geo(2,1)>(aaL1R2*est_geo(1,1)+bbL1R2)  &&  est_geo(2,1)<(aaL2R1*est_geo(1,1)+bbL2R1)  ||  Phase == 5
%          catchtime = catchtime + d_time;
%             if catchtime >= hokakutime
%                Phase = 5;
%             end
%      else
%          catchtime = 0;
%      end
%    else
%      if  est_geo(2,1)<(aaL1L2*est_geo(1,1)+bbL1L2)  &&  est_geo(2,1)>(aaR1R2*est_geo(1,1)+bbR1R2)  &&  est_geo(2,1)>(aaL1R2*est_geo(1,1)+bbL1R2)  &&  est_geo(2,1)<(aaL2R1*est_geo(1,1)+bbL2R1)  ||  Phase == 5
%          catchtime = catchtime + d_time;
%             if catchtime >= hokakutime
%                Phase = 5;
%             end
%      else
%          catchtime = 0;
%      end
%    end
% end



end



xe_des = [ xeL_des' xeR_des' ]';
ve_des = [ veL_des' veR_des' ]';
ae_des = [ aeL_des' aeR_des' ]';




% 関節位置保存
POS_j4_tmp = POS_j4;
POS_j8_tmp = POS_j8;



 end