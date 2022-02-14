function [ xe_des, ve_des, ae_des, POS_jL6_tmp, POS_jR6_tmp, d_POS_jL6, d_POS_jR6, VEL_jL6, VEL_jR6, ...
           d_QL_3, d_QR_7, aL, aR, q_joints, q_wrist, qd_joints, qd_wrist, delta_t, t0, Phase, catchtime] ...
= Onehand_3dim_DesiredHandMotion_v06zyagu( d_time, time, minus_time, SV_d, SV_ts, contactflag_LR, Phase, change5,  ...
                         POS_eL, POS_eR,POS_jL4, POS_jR4, POS_jL5, POS_jR5, POS_jL6, POS_jR6, POS_jL6_tmp, POS_jR6_tmp, d_POS_jL6, d_POS_jR6,...
                         D, d_QL_3, d_QR_7, aL, aR, delta_t, t0, catchtime, hokakutime ,ts1,ts2,ts3,ts4,ts5,ts6,ts3m,ts6m,caging3dim,POS_ee1,POS_ee4)

                     
%推定関数からこの関数にもってきている値、推定情報　est_P, est_V, est_erP, NorTraj_12, est_Q, est_q, est_W, est_geo
%est_P：重心位置  est_erP：？？？
%est_V：重心速度
%
%est_Q：姿勢
%est_q：姿勢(0~pi/2)
%est_W：角速度
%est_geo：幾何中心
%推定関数がない今、いったんこれらを既知とする

est_P = SV_ts.R0;       
est_V = SV_ts.v0;           %いまのところ、並進速度は０
est_q = SV_ts.Q0(3,1);      %いまのところ、Z軸周りの角度にしている
est_W = SV_ts.w0();         %角速度
                     
% global aL
% global aR

 POS_jL4_st = 2 * POS_jL5 - POS_jL4;
 POS_jR4_st = 2 * POS_jR5 - POS_jR4;
 
%%%%%% 関数の初期化 %%%%%%
% 関節角，角速度をそれぞれ異なるベクトルに分ける
     n = length(SV_d.q);     %目黒モデルの配列長さは　12*1　で、n=12となっている。長谷さんのは片側４関節で　n=8
%     d_qd_joints = zeros(n,1); % 目標関節角速度
    qd_joints = zeros(n,1);     % 関節角速度
    qd_wrist = zeros(n,1);      % 手先バネ機構角速度
    
    qd_joints(1:n,1) = SV_d.qd(1:n,1);  % 関節角速度を代入
    
    qd_wrist(n/2,1) =  SV_d.qd(n/2,1);  % 手先バネ機構角速度を代入
    qd_wrist(n,1) =  SV_d.qd(n,1);      % 手先バネ機構角速度を代入
    
    d_q_joints = zeros(n,1);    % 目標関節角度
    q_joints = zeros(n,1);      % 関節角度
    q_wrist = zeros(n,1);       % 手先バネ機構角度
    
    q_joints(1:n,1) = SV_d.q(1:n,1);    % 関節角度を代入
    
    q_wrist(n/2,1) =  SV_d.q(n/2,1);    % 手先バネ機構角度を代入
    q_wrist(n,1) =  SV_d.q(n,1);        % 手先バネ機構角度を代入
    
    
% 手先速度
    VEL_jL6 = ( POS_jL6 - POS_jL6_tmp ) / d_time;
    VEL_jR6 = ( POS_jR6 - POS_jR6_tmp ) / d_time;
    
%%% 目標位置，速度，加速度，角度，角速度，各加速度を設定 %%%
%%% Phase 0
if time < t0 + minus_time
  if time < t0 + minus_time && time >= minus_time
     Phase = 1;
  else
     Phase = 0;
  end
  
  
%%% Phase 1

   d_POS_jL6 = POS_jL6;   %  3*1 両手先関節の位置座標　現在のまま
   d_POS_jR6 = POS_jR6;   %  3*1
   
   d_q_joints = q_joints;       % 目標各関節角度   12*1　左６関節、右６関節分　←　現在の位置をそのまま
   d_VEL_jL6 = VEL_jL6;           % 目標　手先　　速度    2*1
   d_VEL_jR6 = VEL_jR6;
   d_qd_joints = qd_joints;     % 目標関節角速度 12*1 

   
%  xeL_des = [ d_POS_jL6' zeros(1,2) d_q_joints(n/2-1,1) ]';    % 目標手先位置と角度
%  xeR_des = [ d_POS_jR6' zeros(1,2) d_q_joints(n-1,1) ]';
%  veL_des = [ d_VEL_jL6' zeros(1,3) d_qd_joints(n/2-1,1) ]'; % 目標手先速度と角速度
%  veR_des = [ d_VEL_jR6' zeros(1,3) d_qd_joints(n-1,1) ]';
    
    kakuL = kakudo_3dim( POS_jL5, POS_jL4_st, ts6 ); % 目標手先角度
    kakuR = kakudo_3dim( POS_jR5, POS_jR4_st, ts3 );
    kakuL = kakudo_3dim( POS_jL5, POS_jL4_st, SV_ts.R0 ); % 目標手先角度
    kakuR = kakudo_3dim( POS_jR5, POS_jR4_st, SV_ts.R0 );

% KL = kakudo( [ POS_j3' 0 ]', [ ( 2 * POS_j3 - POS_j2 )' 0 ]', [ POS_j7' 0 ]' );
% KR = kakudo( [ POS_j7' 0 ]', [ ( 2 * POS_j7 - POS_j6 )' 0 ]', [ POS_j3' 0 ]' );

 xeL_des = [ d_POS_jL6' kakuL ]';    % 目標手先位置と角度
 xeR_des = [ d_POS_jR6' kakuR ]';
 veL_des = [ d_VEL_jL6' 0 0 0 ]'; % 目標手先速度と角速度
 veR_des = [ d_VEL_jR6' 0 0 0 ]';
 aeL_des = zeros(6,1);   % 目標手先加速度
 aeR_des = zeros(6,1);

 
%%% Phase 2
elseif contactflag_LR(1) > 0 || contactflag_LR(2) > 0 || contactflag_LR(3) > 0   || Phase == 2 
Phase = 2;

%   delta_q = ( pi/4 - est_q );   % ターゲット角がπ/4になるまでの角度Δq       今は回転正なのでπ/4
%   delta_t = delta_q / est_W(3,1);   % ターゲット角がπ/4になるまでの時間Δt
% 
%     d_POS_jL6 = ts6m;  % 目標手先位置
%     d_POS_jR6 = ts3m;
%     tenL = ts6;
%     tenR = ts3;
%     
    kakuL = kakudo_3dim( POS_jL5, POS_jL4_st, SV_ts.R0 ); % 目標手先角度
    kakuR = kakudo_3dim( POS_jR5, POS_jR4_st, SV_ts.R0 );
% %   d_q_joints(n/2-1,1) = kakuL;   % 目標手先角度
% %   d_q_joints(n,1)     = kakuR;
% 
% 	% 目標位置制御→スプライン曲線軌跡追従   とりあえず手先速度制御
%   x0L = POS_eR;   % 初期位置 
%   v0L = zeros(3,1);   % 初期速度
%   xfL = d_POS_jL6(1:3,1);   % 目標位置
%   vfL = zeros(3,1);   % 目標速度
%     a0L = x0L;   % 係数に代入
%     a1L = v0L;
%     a2L = ( -3 * ( x0L - xfL ) -  delta_t * ( 2 * v0L + vfL ) )/( delta_t )^2;
%     a3L = ( 2 * ( x0L - xfL ) -  delta_t * ( v0L + vfL ) )/( delta_t )^3;
%   aL = [ a0L, a1L, a2L, a3L ];
% 
%   x0R = POS_eR;   % 初期位置   % (2,1)
%   v0R = zeros(3,1);   % 初期速度
%   xfR = d_POS_jR6(1:3,1);
%   vfR = zeros(3,1);   % 目標速度
%     a0R = x0R;   % 係数に代入
%     a1R = v0R;
%     a2R = ( -3 * ( x0R - xfR ) - delta_t * ( 2 * v0R + vfR ) )/( delta_t )^2;
%     a3R = ( 2 * ( x0R - xfR ) - delta_t * ( v0R + vfR ) )/( delta_t )^3;
%   aR = [ a0R, a1R, a2R, a3R ];
% 
% 	% スプライン曲線軌跡上の目標手先位置を計算
%   x_L = a0L + a1L * ( time - ( t0 + minus_time ) ) + a2L * ( time - ( t0 + minus_time ) )^2 + a3L * ( time - ( t0 + minus_time ) )^3;
%   x_R = a0R + a1R * ( time - ( t0 + minus_time ) ) + a2R * ( time - ( t0 + minus_time ) )^2 + a3R * ( time - ( t0 + minus_time ) )^3;

    x_L = ts6m;  % 目標手先位置
    x_R = ts3m;  
  
%  xeL_des = [ x_L' zeros(1,3) d_q_joints(n/2-1,1) ]';
%  xeR_des = [ x_R' zeros(1,3) d_q_joints(n,1) ]';
 xeL_des = [ x_L'  kakuL ]';  % 目標手先位置と角度
 xeR_des = [ x_R'  kakuR ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);


 %%% Phase 3以降 スプラインフェーズ　右手だけ動く
% elseif ( time > ( t0 + minus_time )  && time < ( t0 + delta_t + minus_time ) && contactflag_L1==0 && contactflag_L2==0 && contactflag_R1==0 && contactflag_R2==0 )% || Phase == 3
% Phase = 3;
elseif max(contactflag_LR) == 0 && Phase ~= 4 && Phase ~= 5 
Phase = 3;
 
  delta_q = ( pi/4 - est_q );   % ターゲット角がπ/4になるまでの角度Δq       今は回転正なのでπ/4
  delta_t = delta_q / est_W(3,1);   % ターゲット角がπ/4になるまでの時間Δt

    d_POS_jL6 = POS_jL6;  % 目標手先位置
    d_POS_jR6 = ts3;
    tenL = ts6;
    tenR = ts3;
    
%   kakuL = (SV_ts.R0 - POS_jL5)';
%   kakuR = (SV_ts.R0 - POS_jR5)';
%     kakuL = kakudo_3dim( POS_jL5, POS_jL4_st, ts6 ); % 目標手先角度
%     kakuR = kakudo_3dim( POS_jR5, POS_jR4_st, ts3 );
    kakuL = kakudo_3dim( POS_jL5, POS_jL4_st, SV_ts.R0 ); % 目標手先角度
    kakuR = kakudo_3dim( POS_jR5, POS_jR4_st, SV_ts.R0 );
    
%   d_q_joints(n/2-1,1) = kakuL;   % 目標手先角度
%   d_q_joints(n,1)     = kakuR;

	% 目標位置制御→スプライン曲線軌跡追従   とりあえず手先速度制御
  x0L = POS_jL6;   % 初期位置 
  v0L = zeros(3,1);   % 初期速度
  xfL = d_POS_jL6(1:3,1);   % 目標位置
  vfL = zeros(3,1);   % 目標速度
    a0L = x0L;   % 係数に代入
    a1L = v0L;
    a2L = ( -3 * ( x0L - xfL ) -  delta_t * ( 2 * v0L + vfL ) )/( delta_t )^2;
    a3L = ( 2 * ( x0L - xfL ) -  delta_t * ( v0L + vfL ) )/( delta_t )^3;
  aL = [ a0L, a1L, a2L, a3L ];

  x0R = POS_jR6;   % 初期位置   % (2,1)
  v0R = zeros(3,1);   % 初期速度
  xfR = d_POS_jR6(1:3,1);
  vfR = zeros(3,1);   % 目標速度
    a0R = x0R;   % 係数に代入
    a1R = v0R;
    a2R = ( -3 * ( x0R - xfR ) - delta_t * ( 2 * v0R + vfR ) )/( delta_t )^2;
    a3R = ( 2 * ( x0R - xfR ) - delta_t * ( v0R + vfR ) )/( delta_t )^3;
  aR = [ a0R, a1R, a2R, a3R ];
  

  x_L = aL(1:3,1) + aL(1:3,2) * ( time - ( t0 + minus_time ) ) + aL(1:3,3) * ( time - ( t0 + minus_time ) )^2 + aL(1:3,4) * ( time - ( t0 + minus_time ) )^3;   % スプライン曲線軌跡上の目標手先位置を計算
  x_R = aR(1:3,1) + aR(1:3,2) * ( time - ( t0 + minus_time ) ) + aR(1:3,3) * ( time - ( t0 + minus_time ) )^2 + aR(1:3,4) * ( time - ( t0 + minus_time ) )^3;


    xeL_des = [ x_L' kakuL ]';
    xeR_des = [ x_R' kakuR ]';
    veL_des = zeros(6,1);
    veR_des = zeros(6,1);
    aeL_des = zeros(6,1);
    aeR_des = zeros(6,1);
    
    
%%% Phase 4 右手接触後停止　左手が追従するフェーズ
% elseif time >= ( t0 + delta_t + minus_time ) || max(contactflag_LR) > 0  ||  Phase==4  ||  Phase==5 || time >=5
elseif  contactflag_LR(4) > 0 || contactflag_LR(5) > 0 || contactflag_LR(6) > 0   || Phase == 4 && Phase ~= 5
Phase = 4;
    d_POS_jL6 = ts6;  % 目標手先位置
    d_POS_jR6 = ts3m;
    tenL = ts6;
    tenR = ts3;
    
%   kakuL = (SV_ts.R0 - POS_jL5)';
%   kakuR = (SV_ts.R0 - POS_jR5)';
%     kakuL = kakudo_3dim( POS_jL5, POS_jL4_st, ts6 ); % 目標手先角度
%     kakuR = kakudo_3dim( POS_jR5, POS_jR4_st, ts3 );
    kakuL = kakudo_3dim( POS_jL5, POS_jL4_st, SV_ts.R0 ); % 目標手先角度
    kakuR = kakudo_3dim( POS_jR5, POS_jR4_st, SV_ts.R0 );

	% 目標位置制御→スプライン曲線軌跡追従   とりあえず手先速度制御
  x0L = POS_jL6;   % 初期位置 
  v0L = zeros(3,1);   % 初期速度
  xfL = d_POS_jL6(1:3,1);   % 目標位置
  vfL = zeros(3,1);   % 目標速度
    a0L = x0L;   % 係数に代入
    a1L = v0L;
    a2L = ( -3 * ( x0L - xfL ) -  delta_t * ( 2 * v0L + vfL ) )/( delta_t )^2;
    a3L = ( 2 * ( x0L - xfL ) -  delta_t * ( v0L + vfL ) )/( delta_t )^3;
  aL = [ a0L, a1L, a2L, a3L ];

  x0R = POS_jR6;   % 初期位置   % (2,1)
  v0R = zeros(3,1);   % 初期速度
  xfR = d_POS_jR6(1:3,1);
  vfR = zeros(3,1);   % 目標速度
    a0R = x0R;   % 係数に代入
    a1R = v0R;
    a2R = ( -3 * ( x0R - xfR ) - delta_t * ( 2 * v0R + vfR ) )/( delta_t )^2;
    a3R = ( 2 * ( x0R - xfR ) - delta_t * ( v0R + vfR ) )/( delta_t )^3;
  aR = [ a0R, a1R, a2R, a3R ];
  

  x_L = aL(1:3,1) + aL(1:3,2) * ( time - ( t0 + minus_time ) ) + aL(1:3,3) * ( time - ( t0 + minus_time ) )^2 + aL(1:3,4) * ( time - ( t0 + minus_time ) )^3;   % スプライン曲線軌跡上の目標手先位置を計算
  x_R = aR(1:3,1) + aR(1:3,2) * ( time - ( t0 + minus_time ) ) + aR(1:3,3) * ( time - ( t0 + minus_time ) )^2 + aR(1:3,4) * ( time - ( t0 + minus_time ) )^3;
  
    xeL_des = [ x_L' kakuL ]';
    xeR_des = [ x_R' kakuR ]';
    veL_des = zeros(6,1);
    veR_des = zeros(6,1);
    aeL_des = zeros(6,1);
    aeR_des = zeros(6,1);
    
 
%  
% %  Phase 5   左手が接触したら入る
%  elseif contactflag_LR(1) > 0 || contactflag_LR(2) > 0 || contactflag_LR(3) > 0   || Phase == 5 
% Phase = 5;
% 
%     d_POS_jL6 = ts6;  % 目標手先位置
%     d_POS_jR6 = ts3;
%     tenL = ts6;
%     tenR = ts3;
%     
% %   kakuL = (SV_ts.R0 - POS_jL5)';
% %   kakuR = (SV_ts.R0 - POS_jR5)';
% %     kakuL = kakudo_3dim( POS_jL5, POS_jL4_st, ts6 ); % 目標手先角度
% %     kakuR = kakudo_3dim( POS_jR5, POS_jR4_st, ts3 );
%     kakuL = kakudo_3dim( POS_jL5, POS_jL4_st, SV_ts.R0 ); % 目標手先角度
%     kakuR = kakudo_3dim( POS_jR5, POS_jR4_st, SV_ts.R0 );
% 
% 	% 目標位置制御→スプライン曲線軌跡追従   とりあえず手先速度制御
%   x0L = POS_jL6;   % 初期位置 
%   v0L = zeros(3,1);   % 初期速度
%   xfL = d_POS_jL6(1:3,1);   % 目標位置
%   vfL = zeros(3,1);   % 目標速度
%     a0L = x0L;   % 係数に代入
%     a1L = v0L;
%     a2L = ( -3 * ( x0L - xfL ) -  delta_t * ( 2 * v0L + vfL ) )/( delta_t )^2;
%     a3L = ( 2 * ( x0L - xfL ) -  delta_t * ( v0L + vfL ) )/( delta_t )^3;
%   aL = [ a0L, a1L, a2L, a3L ];
% 
%   x0R = POS_jR6;   % 初期位置   % (2,1)
%   v0R = zeros(3,1);   % 初期速度
%   xfR = d_POS_jR6(1:3,1);
%   vfR = zeros(3,1);   % 目標速度
%     a0R = x0R;   % 係数に代入
%     a1R = v0R;
%     a2R = ( -3 * ( x0R - xfR ) - delta_t * ( 2 * v0R + vfR ) )/( delta_t )^2;
%     a3R = ( 2 * ( x0R - xfR ) - delta_t * ( v0R + vfR ) )/( delta_t )^3;
%   aR = [ a0R, a1R, a2R, a3R ];
%   
% 
%   x_L = aL(1:3,1) + aL(1:3,2) * ( time - ( t0 + minus_time ) ) + aL(1:3,3) * ( time - ( t0 + minus_time ) )^2 + aL(1:3,4) * ( time - ( t0 + minus_time ) )^3;   % スプライン曲線軌跡上の目標手先位置を計算
%   x_R = aR(1:3,1) + aR(1:3,2) * ( time - ( t0 + minus_time ) ) + aR(1:3,3) * ( time - ( t0 + minus_time ) )^2 + aR(1:3,4) * ( time - ( t0 + minus_time ) )^3;
%   
%     xeL_des = [ x_L' kakuL ]';
%     xeR_des = [ x_R' kakuR ]';
%     veL_des = zeros(6,1);
%     veR_des = zeros(6,1);
%     aeL_des = zeros(6,1);
%     aeR_des = zeros(6,1);
%     

else %Phase 1～５以外
 x_L = zeros(3,1);
 x_R = zeros(3,1);
    
 xeL_des = [ x_L' 0 0 0 ]'; %Phase4は POS_eL,eR の値が0
 xeR_des = [ x_R' 0 0 0 ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);
 
end
 
%=========================================================================

xe_des = [ xeL_des' xeR_des' ]'; %xeL_des(6*1) + xeR_des(6*1)　結果的に12*1行列になっている。[0;0;0;0;-0.06;-0.018;    0;0;0;0; -0.05; 0.019]
ve_des = [ veL_des' veR_des' ]';
ae_des = [ aeL_des' aeR_des' ]';

% 関節位置保存
POS_jL6_tmp = POS_jL6;
POS_jR6_tmp = POS_jR6;
%手先角保存

end
         
                     
                    
