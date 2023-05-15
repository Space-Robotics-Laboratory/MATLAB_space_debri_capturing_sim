function [ xe_des, ve_des, ae_des, POS_j4_tmp, POS_j8_tmp, d_POS_j4, d_POS_j8, VEL_j4, VEL_j8, ...
           d_QL_3, d_QR_7, aL, aR, q_joints, q_wrist, qd_joints, qd_wrist, delta_t, delta_q, t0, t00, Phase, catchtime, fff, d_geo, d_ten1, d_ten2, d_ten3, d_ten4, d_cen_L, d_cen_R, cen_L, cen_R ] ...
= Chohou_DesiredHandMotion_v01( d_time, time, minus_time, SV_d, Obs1, Obs2, est_P, est_Q, est_V, est_W, est_geo, ...
                              POS_j2, POS_j6, POS_j3, POS_j7, POS_j4, POS_j8, POS_j4_tmp, POS_j8_tmp, d_POS_j4, d_POS_j8,...
                              D, D_0, half_side_tan, half_side_cho, d_QL_3, d_QR_7, aL, aR, delta_t, delta_q, t0, contactflag_L1, contactflag_L2, contactflag_R1, contactflag_R2, ...
                              catchtime, POS_eL1, POS_eL2, POS_eR1, POS_eR2, hokakutime, d_geo, d_ten1, d_ten2, d_ten3, d_ten4, t00, r_tip, L_hiraki, theta_hiraki, d_cen_L, d_cen_R, cen_L, cen_R )

fff = 0;
D = D * 1.0;

%%%%%% �֐��̏����� %%%%%%

% �֐ߊp�C�p���x�����ꂼ��قȂ�x�N�g���ɕ�����
    n = 8;
%     d_qd_joints = zeros(n,1);   % �ڕW�֐ߊp���x
    qd_joints = zeros(n,1);   % �֐ߊp���x
    qd_wrist = zeros(n,1);   % ���o�l�@�\�p���x
    qd_joints(1:3,1) = SV_d.qd(1:3,1);   % �֐ߊp���x����
    qd_joints(5:7,1) = SV_d.qd(5:7,1);   % �֐ߊp���x����
    qd_wrist(4,1) =  SV_d.qd(4,1);   % ���o�l�@�\�p���x����
    qd_wrist(8,1) =  SV_d.qd(8,1);   % ���o�l�@�\�p���x����
    
    d_q_joints = zeros(n,1);   % �ڕW�֐ߊp�x
    q_joints = zeros(n,1);   % �֐ߊp�x
    q_wrist = zeros(n,1);   % ���o�l�@�\�p�x
    q_joints(1:3,1) = SV_d.q(1:3,1);   % �֐ߊp�x����
    q_joints(5:7,1) = SV_d.q(5:7,1);   % �֐ߊp�x����
    q_wrist(4,1) =  SV_d.q(4,1);   % ���o�l�@�\�p�x����
    q_wrist(8,1) =  SV_d.q(8,1);   % ���o�l�@�\�p�x����


% �֐�4,8�ʒu�̑��x
    VEL_j4 = ( POS_j4 - POS_j4_tmp ) / d_time;
    VEL_j8 = ( POS_j8 - POS_j8_tmp ) / d_time;

% �^�[�Q�b�g���_�̐ݒ�
    ten1 = Obs1;
    ten2 = Obs2;
    ten3 = 2 * est_geo - ten1;
    ten4 = 2 * est_geo - ten2;

 POS_j2_st = 2 * POS_j3 - POS_j2;   % �֐�3���猩���Ƃ��֐�2���璚�x180���̓_
 POS_j6_st = 2 * POS_j7 - POS_j6;   % �֐�7���猩���Ƃ��֐�6���璚�x180���̓_

%  Q_pi_4 = est_Q(3,1) - pi/4;
%  Q_target = [ cos( Q_pi_4 ), sin( Q_pi_4 ), 0 ]';


%%% �ڕW�ʒu�C���x�C�����x�C�p�x�C�p���x�C�e�����x��ݒ� %%%


%%% Phase 1   �A�[���͓������Ȃ�
if time < t0 + minus_time
  if time < t0 + minus_time && time >= minus_time
     Phase = 1;
  else
     Phase = 0;
  end

   d_POS_j4 = [ POS_j4' 0 ]';   % (3,1)
   d_POS_j8 = [ POS_j8' 0 ]';   % (3,1)
   d_q_joints = q_joints;   % �ڕW���p�x
   d_VEL_j4 = VEL_j4;   % (2,1)   % �ڕW��摬�x   % (2,1)
   d_VEL_j8 = VEL_j8;   % (2,1)
   d_qd_joints = qd_joints;   % �ڕW���p���x

 xeL_des = [ d_POS_j4' zeros(1,2) d_q_joints(3,1) ]';   % �ڕW���ʒu�p�x
 xeR_des = [ d_POS_j8' zeros(1,2) d_q_joints(7,1) ]';
 veL_des = [ d_VEL_j4' zeros(1,3) d_qd_joints(3,1) ]';   % �ڕW��摬�x�p���x
 veR_des = [ d_VEL_j8' zeros(1,3) d_qd_joints(7,1) ]';
 aeL_des = zeros(6,1);   % �ڕW�������x�p�����x
 aeR_des = zeros(6,1);

  theta_chohou = abs( atan( half_side_cho / half_side_tan ) );
  t0 = abs( ( pi/2 - theta_chohou ) / est_W(3,1) );

% %%% Phase 1-2   �A�[���͓������Ȃ�
% elseif time >= t0 + minus_time  &&  time <= t00 + minus_time
%     
%     Phase = 1;
% 
%    d_POS_j4 = [ POS_j4' 0 ]';   % (3,1)
%    d_POS_j8 = [ POS_j8' 0 ]';   % (3,1)
%    d_q_joints = q_joints;   % �ڕW���p�x
%    d_VEL_j4 = VEL_j4;   % (2,1)   % �ڕW��摬�x   % (2,1)
%    d_VEL_j8 = VEL_j8;   % (2,1)
%    d_qd_joints = qd_joints;   % �ڕW���p���x
% 
%  xeL_des = [ d_POS_j4' zeros(1,2) d_q_joints(3,1) ]';   % �ڕW���ʒu�p�x
%  xeR_des = [ d_POS_j8' zeros(1,2) d_q_joints(7,1) ]';
%  veL_des = [ d_VEL_j4' zeros(1,3) d_qd_joints(3,1) ]';   % �ڕW��摬�x�p���x
%  veR_des = [ d_VEL_j8' zeros(1,3) d_qd_joints(7,1) ]';
%  aeL_des = zeros(6,1);   % �ڕW�������x�p�����x
%  aeR_des = zeros(6,1);





%%% Phase 2
elseif time > ( t0 + minus_time - d_time )  &&  time < ( t0 + minus_time + d_time )    % �Ȃ���Phase2�ɓ���Ȃ��Ƃ������ۂ̂��߁C==�ł͂Ȃ��s�����ɂ���
Phase = 2;

% �d�S�ʒu�Ɗ􉽒��S�̈ʒu�֌W
% if norm( est_geo - est_P ) < 1e-5
% %    theta_com = 0;
%    delta_q = 0;
%    delta_t = 2;
% 
% else
%    theta_com = abs( atan2( ( ten3(2,1) - ten2(2,1) ), ( ten3(1,1) - ten2(1,1) ) ) );  % �^�[�Q�b�g�p��
   theta_chohou = abs( atan( half_side_cho / half_side_tan ) );
%    theta_chohou = abs( asin( ( 2 * L_hiraki * sin( theta_hiraki ) ) / ( half_side_tan + r_tip ) ) );
%    delta_q = theta_chohou - theta_com;
   delta_q = theta_chohou - est_Q(3,1)-pi/2;
   delta_t = abs( delta_q / est_W(3,1) );

%  % �d�S�̒��x�^��Ɋ􉽒��S������܂ł̎��Ԃ��v�Z
%  if      est_W(3,1) > 0
%       if theta_com > 0
%          delta_q = 3*pi/2 - theta_com;
%          fff = 1;
%       else
%          delta_q = pi/2 - theta_com;
%          fff = 2;
%       end
%    delta_t = abs( delta_q / est_W(3,1) );
%  elseif  est_W(3,1) < 0
%       if theta_com > 0
%          delta_q = pi/2 + theta_com;
%          fff = 3;
%       else
%          delta_q = 3*pi/2 + theta_com;
%          fff = 4;
%       end
%    delta_t = abs( delta_q / est_W(3,1) );
%  else   % ��]���Ă��Ȃ��Ƃ��́C2�b�ԂŎ����ړ�
%    delta_q = 0;
%    delta_t = 2;
%         fff = 0;
%  end
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
%  else   % ��]���Ă��Ȃ��Ƃ��́C2�b�ԂŎ����ړ�
%    delta_q = 0;
%    delta_t = 2;
%         fff = 0;
%  end

% end

%%% ��t��̊􉽒��S�ʒu���v�Z
  d_geo = est_P + rpy2dc( 0, 0, delta_q * sign( est_W(3,1) ) )' * ( est_geo - est_P ) + delta_t * est_V;
%   DD = ( half_side_cho + r_tip )/( half_side_tan + r_tip ) * L_hiraki * sin( theta_hiraki );

  d_ten1 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten1 - est_P ) + est_V * delta_t;   % �e���_�̃�t��̈ʒu���v�Z
  d_ten2 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten2 - est_P ) + est_V * delta_t;
  d_ten3 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten3 - est_P ) + est_V * delta_t;
  d_ten4 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten4 - est_P ) + est_V * delta_t;

  if     est_W(3,1) > 0
  d_cen_L = d_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou + pi/2 ) sin( theta_chohou + pi/2 ) 0 ]';
  d_cen_R = d_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou - pi/2 ) sin( theta_chohou - pi/2 ) 0 ]';
%   d_POS_j4 = d_cen_L + D * [ cos( theta_chohou + pi/2 + pi/4 ) sin( theta_chohou + pi/2 + pi/4 ) 0 ]';
%   d_POS_j8 = d_cen_R + D * [ cos( theta_chohou - pi/2 + pi/4 ) sin( theta_chohou - pi/2 + pi/4 ) 0 ]';]';
  d_POS_j4 = d_ten1 + D * [ cos( theta_chohou + pi/2 + pi/4 ) sin( theta_chohou + pi/2 + pi/4 ) 0 ]';
  d_POS_j8 = d_ten3 + D * [ cos( theta_chohou - pi/2 + pi/4 ) sin( theta_chohou - pi/2 + pi/4 ) 0 ]';
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', d_cen_L );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', d_cen_R );
%   d_POS_j4 = d_geo - DD * [ 1 0 0 ]';
%   d_POS_j8 = d_geo + DD * [ 1 0 0 ]';
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', ten1 );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', ten3 );
  elseif est_W(3,1) < 0
  d_cen_L = d_geo + ( half_side_cho - half_side_tan ) * [ cos( -pi/2 - theta_chohou ) sin( -pi/2 - theta_chohou ) 0 ]';
  d_cen_R = d_geo + ( half_side_cho - half_side_tan ) * [ cos( pi/2 - theta_chohou ) sin( pi/2 - theta_chohou ) 0 ]';
%   d_POS_j4 = d_cen_L + D * [ cos( -pi/2 - theta_chohou - pi/4 ) sin( -pi/2 - theta_chohou - pi/4 ) 0 ]';
%   d_POS_j8 = d_cen_R + D * [ cos( pi/2 - theta_chohou - pi/4 ) sin( pi/2 - theta_chohou - pi/4 ) 0 ]';
  d_POS_j4 = d_ten2 + D * [ cos( -pi/2 - theta_chohou - pi/4 ) sin( -pi/2 - theta_chohou - pi/4 ) 0 ]';
  d_POS_j8 = d_ten4 + D * [ cos( pi/2 - theta_chohou - pi/4 ) sin( pi/2 - theta_chohou - pi/4 ) 0 ]';
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', d_cen_L );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', d_cen_R );
%   d_POS_j4 = est_geo - DD * [ 1 0 0 ]';
%   d_POS_j8 = est_geo + DD * [ 1 0 0 ]';
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', ten2 );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', ten4 );
  else
  d_POS_j4 = d_geo - D_0 * [ 1 0 0 ]';   % �ڕW���ʒu
  d_POS_j8 = d_geo + D_0 * [ 1 0 0 ]';
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
  end

  d_QL_3 = atan2( ( est_geo(2,1) - POS_j3(2,1) ), ( est_geo(1,1) - POS_j3(1,1) ) );
  d_QR_7 = atan2( ( est_geo(2,1) - POS_j7(2,1) ), ( est_geo(1,1) - POS_j7(1,1) ) );

%   if     est_W(3,1) > 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
%     d_POS_j4 = d_ten1 - D * L_target;   % �ڕW���ʒu
%     d_POS_j8 = d_ten3 + D * L_target;
%     tenL = ten1;
%     tenR = ten3;
%       d_QL_3 = atan2( ( ten1(2,1) - POS_j3(2,1) ), ( ten1(1,1) - POS_j3(1,1) ) );
%       d_QR_7 = atan2( ( ten3(2,1) - POS_j7(2,1) ), ( ten3(1,1) - POS_j7(1,1) ) );
%     
%   elseif est_W(3,1) < 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
%     d_POS_j4 = d_ten2 - D * L_target;   % �ڕW���ʒu
%     d_POS_j8 = d_ten4 + D * L_target;
%     tenL = ten2;
%     tenR = ten4;
%       d_QL_3 = atan2( ( ten2(2,1) - POS_j3(2,1) ), ( ten2(1,1) - POS_j3(1,1) ) );
%       d_QR_7 = atan2( ( ten4(2,1) - POS_j7(2,1) ), ( ten4(1,1) - POS_j7(1,1) ) );
% 
%   else                    % �^�[�Q�b�g�p���x��0�̂Ƃ�
%     d_POS_j4 = ( ten1 + ten2 ) / 2 - D_0 * L_target;   % �ڕW���ʒu
%     d_POS_j8 = ( ten3 + ten4 ) / 2 + D_0 * L_target;
%       d_QL_3 = 0;
%       d_QR_7 = pi;
% 
%   end

  d_q_joints(3,1) = kakuL;   % �ڕW���p�x
  d_q_joints(7,1) = kakuR;

  %   �ڕW�ʒu���䁨�X�v���C���Ȑ��O�ՒǏ]   �Ƃ肠������摬�x����
  x0L = POS_j4;   % �����ʒu   % (2,1)
  v0L = zeros(2,1);   % �������x
  xfL = d_POS_j4(1:2,1);   % �ڕW�ʒu
  vfL = zeros(2,1);   % �ڕW���x
    a0L = x0L;   % �W���ɑ��
    a1L = v0L;
    a2L = ( -3 * ( x0L - xfL ) - delta_t * ( 2 * v0L + vfL ) )/( delta_t )^2;
    a3L = ( 2 * ( x0L - xfL ) - delta_t * ( v0L + vfL ) )/( delta_t )^3;
  aL = [ a0L, a1L, a2L, a3L ];

  x0R = POS_j8;   % �����ʒu   % (2,1)
  v0R = zeros(2,1);   % �������x
  xfR = d_POS_j8(1:2,1);
  vfR = zeros(2,1);   % �ڕW���x
    a0R = x0R;   % �W���ɑ��
    a1R = v0R;
    a2R = ( -3 * ( x0R - xfR ) - delta_t * ( 2 * v0R + vfR ) )/( delta_t )^2;
    a3R = ( 2 * ( x0R - xfR ) - delta_t * ( v0R + vfR ) )/( delta_t )^3;
  aR = [ a0R, a1R, a2R, a3R ];

  x_L = a0L + a1L * ( time - ( t0 + minus_time ) ) + a2L * ( time - ( t0 + minus_time ) )^2 + a3L * ( time - ( t0 + minus_time ) )^3;   % �X�v���C���Ȑ��O�Տ�̖ڕW���ʒu���v�Z
  x_R = a0R + a1R * ( time - ( t0 + minus_time ) ) + a2R * ( time - ( t0 + minus_time ) )^2 + a3R * ( time - ( t0 + minus_time ) )^3;

% % �p�ɐ�����
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



%%% Phase 3�ȍ~
elseif time >= ( t0 + minus_time + d_time )  &&  time <= ( t0 + delta_t + minus_time )%  &&  contactflag_L1==0  &&  contactflag_L2==0  &&  contactflag_R1==0  &&  contactflag_R2==0   % && Phase == 3
Phase = 3;

   theta_chohou = abs( atan( half_side_cho / half_side_tan ) );
  if     est_W(3,1) > 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', d_cen_L );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', d_cen_R );
  cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou + pi/2 ) sin( theta_chohou + pi/2 ) 0 ]';
  cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou - pi/2 ) sin( theta_chohou - pi/2 ) 0 ]';
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', cen_L );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', cen_R );
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
     d_QL_3 = atan2( ( ten1(2,1) - POS_j3(2,1) ), ( ten1(1,1) - POS_j3(1,1) ) );
     d_QR_7 = atan2( ( ten3(2,1) - POS_j7(2,1) ), ( ten3(1,1) - POS_j7(1,1) ) );
    
  elseif est_W(3,1) < 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', d_cen_L );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', d_cen_R );
  cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( -pi/2 - theta_chohou ) sin( -pi/2 - theta_chohou ) 0 ]';
  cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( pi/2 - theta_chohou ) sin( pi/2 - theta_chohou ) 0 ]';
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', cen_L );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', cen_R );
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
     d_QL_3 = atan2( ( ten2(2,1) - POS_j3(2,1) ), ( ten2(1,1) - POS_j3(1,1) ) );
     d_QR_7 = atan2( ( ten4(2,1) - POS_j7(2,1) ), ( ten4(1,1) - POS_j7(1,1) ) );

  else                    % �^�[�Q�b�g�p���x��0�̂Ƃ�
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
     d_QL_3 = 0;
     d_QR_7 = pi;

  end

  d_q_joints(3,1) = kakuL;   % �ڕW���p�x
  d_q_joints(7,1) = kakuR;

  x_L = aL(:,1) + aL(:,2) * ( time - ( t0 + minus_time ) ) + aL(:,3) * ( time - ( t0 + minus_time ) )^2 + aL(:,4) * ( time - t0 - minus_time )^3;
  x_R = aR(:,1) + aR(:,2) * ( time - ( t0 + minus_time ) ) + aR(:,3) * ( time - ( t0 + minus_time ) )^2 + aR(:,4) * ( time - t0 - minus_time )^3;

% % �p�ɐ�����
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


% %%% Phase 4��5�ɕύX���ē������Ȃ����Ƃɂ���(v02)
% elseif time >= ( t0 + delta_t + minus_time )  ||  contactflag_L1==1  ||  contactflag_L2==1  ||  contactflag_R1==1  ||  contactflag_R2==1    ||    Phase==4  ||  Phase==5
% Phase = 5;
% 
% %   thetaTL = atan2( ( tenL(2,1) - est_geo(2,1) ), ( tenL(1,1) - est_geo(1,1) ) );   % �����l����
% %   thetaTL = atan2( ( tenL(2,1) - tenR(2,1) ), ( tenL(1,1) - tenR(1,1) ) );   % �����l����
% %   thetaTR = atan2( ( tenR(2,1) - tenL(2,1) ), ( tenR(1,1) - tenL(1,1) ) );
% %   thetaTL = atan2( ( POS_j4(2,1) - POS_j8(2,1) ), ( POS_j4(1,1) - POS_j8(1,1) ) );   % �����l����
%   thetaTL = atan2( ( POS_j4(2,1) - est_geo(2,1) ), ( POS_j4(1,1) - est_geo(1,1) ) );   % �����l����
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
% %   d_q_joints(3,1) = kakuL;   % �ڕW���p�x
% %   d_q_joints(7,1) = kakuR;

else

Phase = 4;
% Phase = 5;


%%% �ߊl�t�F�[�Y Phase5   4�_�̒��ɐ���􉽒��S��catchtime�b�����Ă���

A = cross( ( est_geo - POS_eL2 ), ( POS_eL1 - est_geo ) );
B = cross( ( est_geo - POS_eR1 ), ( POS_eL2 - est_geo ) );
C = cross( ( est_geo - POS_eR2 ), ( POS_eR1 - est_geo ) );
D = cross( ( est_geo - POS_eL1 ), ( POS_eR2 - est_geo ) );
AA = norm( POS_eL1 - POS_eL2 );
BB = norm( POS_eR1 - POS_eR2 );
CC = norm( POS_eR1 - POS_eL2 );
DD = norm( POS_eL1 - POS_eR2 );

if A(3,1) > 0 && B(3,1) > 0 && C(3,1) > 0 && D(3,1) > 0 % && AA < (2*r_target) && BB < (2*r_target) && CC < (2*r_target) && DD < (2*r_target)
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
% d_POS_j4 = est_geo - D * LL;   % �ڕW���ʒu
% d_POS_j8 = est_geo + D * LL;
% 
%   x0L = POS_j4;   % �����ʒu   % (2,1)
%   v0L = VEL_j4(2,1);   % �������x
%   xfL = d_POS_j4(1:2,1);   % �ڕW�ʒu
%   vfL = zeros(2,1);   % �ڕW���x
%     a0L = x0L;   % �W���ɑ��
%     a1L = v0L;
%     a2L = ( -3 * ( x0L - xfL ) - delta_t_2 * ( 2 * v0L + vfL ) )/( delta_t_2 )^2;
%     a3L = ( 2 * ( x0L - xfL ) - delta_t_2 * ( v0L + vfL ) )/( delta_t_2 )^3;
%   aL = [ a0L, a1L, a2L, a3L ];
% 
%   x0R = POS_j8;   % �����ʒu   % (2,1)
%   v0R = zeros(2,1);   % �������x
%   xfR = d_POS_j8(1:2,1);
%   vfR = zeros(2,1);   % �ڕW���x
%     a0R = x0R;   % �W���ɑ��
%     a1R = v0R;
%     a2R = ( -3 * ( x0R - xfR ) - delta_t_2 * ( 2 * v0R + vfR ) )/( delta_t_2 )^2;
%     a3R = ( 2 * ( x0R - xfR ) - delta_t_2 * ( v0R + vfR ) )/( delta_t_2 )^3;
%   aR = [ a0R, a1R, a2R, a3R ];
% 
%   x_L = a0L + a1L * ( time - starttime ) + a2L * ( time - starttime )^2 + a3L * ( time - starttime )^3;   % �X�v���C���Ȑ��O�Տ�̖ڕW���ʒu���v�Z
%   x_R = a0R + a1R * ( time - starttime ) + a2R * ( time - starttime )^2 + a3R * ( time - starttime )^3;
% 
% elseif catchtime > d_time && catchtime < ( d_time + delta_t_2 )
% 
%   x_L = a0L + a1L * ( time - starttime ) + a2L * ( time - starttime )^2 + a3L * ( time - starttime )^3;   % �X�v���C���Ȑ��O�Տ�̖ڕW���ʒu���v�Z
%   x_R = a0R + a1R * ( time - starttime ) + a2R * ( time - starttime )^2 + a3R * ( time - starttime )^3;
% 
% end


%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j4' 0 ]' );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j8' 0 ]' );
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
%   if     est_W(3,1) > 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', ten1 );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', ten3 );
%      d_QL_3 = atan2( ( ten1(2,1) - POS_j3(2,1) ), ( ten1(1,1) - POS_j3(1,1) ) );
%      d_QR_7 = atan2( ( ten3(2,1) - POS_j7(2,1) ), ( ten3(1,1) - POS_j7(1,1) ) );
%     
%   elseif est_W(3,1) < 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', ten2 );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', ten4 );
%      d_QL_3 = atan2( ( ten2(2,1) - POS_j3(2,1) ), ( ten2(1,1) - POS_j3(1,1) ) );
%      d_QR_7 = atan2( ( ten4(2,1) - POS_j7(2,1) ), ( ten4(1,1) - POS_j7(1,1) ) );
% 
%   else                    % �^�[�Q�b�g�p���x��0�̂Ƃ�
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
%      d_QL_3 = 0;
%      d_QR_7 = pi;
% 
%   end

%   if     est_W(3,1) > 0
%   d_cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou + pi/2 ) sin( theta_chohou + pi/2 ) 0 ]';
%   d_cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou - pi/2 ) sin( theta_chohou - pi/2 ) 0 ]';
%   d_POS_j4 = d_cen_L + D * [ cos( theta_chohou + pi/2 + pi/4 ), sin( theta_chohou + pi/2 + pi/4 ), 0 ]';
%   d_POS_j8 = d_cen_R + D * [ cos( theta_chohou - pi/2 + pi/4 ), sin( theta_chohou - pi/2 + pi/4 ), 0 ]';
% %   d_POS_j4 = d_ten1 + D * [ cos( theta_chohou + pi/2 + pi/4 ) sin( theta_chohou + pi/2 + pi/4 ) 0 ]';
% %   d_POS_j8 = d_ten3 + D * [ cos( theta_chohou - pi/2 + pi/4 ) sin( theta_chohou - pi/2 + pi/4 ) 0 ]';
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', d_cen_L );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', d_cen_R );
% %   d_POS_j4 = d_geo - DD * [ 1 0 0 ]';
% %   d_POS_j8 = d_geo + DD * [ 1 0 0 ]';
% %   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', ten1 );
% %   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', ten3 );
%   elseif est_W(3,1) < 0
%   d_cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( -pi/2 - theta_chohou ) sin( -pi/2 - theta_chohou ) 0 ]';
%   d_cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( pi/2 - theta_chohou ) sin( pi/2 - theta_chohou ) 0 ]';
%   d_POS_j4 = d_cen_L + D * [ cos( -pi/2 - theta_chohou - pi/4 ) sin( -pi/2 - theta_chohou - pi/4 ) 0 ]';
%   d_POS_j8 = d_cen_R + D * [ cos( pi/2 - theta_chohou - pi/4 ) sin( pi/2 - theta_chohou - pi/4 ) 0 ]';
% %   d_POS_j4 = d_ten2 + D * [ cos( -pi/2 - theta_chohou - pi/4 ) sin( -pi/2 - theta_chohou - pi/4 ) 0 ]';
% %   d_POS_j8 = d_ten4 + D * [ cos( pi/2 - theta_chohou - pi/4 ) sin( pi/2 - theta_chohou - pi/4 ) 0 ]';
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', d_cen_L );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', d_cen_R );
% %   d_POS_j4 = est_geo - DD * [ 1 0 0 ]';
% %   d_POS_j8 = est_geo + DD * [ 1 0 0 ]';
% %   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', ten2 );
% %   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', ten4 );
%   else
%   d_POS_j4 = d_geo - D_0 * [ 1 0 0 ]';   % �ڕW���ʒu
%   d_POS_j8 = d_geo + D_0 * [ 1 0 0 ]';
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
%   end

   theta_com = atan2( ( ten3(2,1) - ten2(2,1) ), ( ten3(1,1) - ten2(1,1) ) );  % �^�[�Q�b�g�p��
%   theta_chohou = abs( atan( half_side_cho / half_side_tan ) );
%   D = abs( L_hiraki * ( cos( theta_hiraki ) - sin( theta_hiraki ) ) + r_tip * sqrt(2) );
   D = abs( L_hiraki * ( cos( theta_hiraki ) - sin( theta_hiraki ) ) + ( r_tip + half_side_tan ) * sqrt(2) );
  if     est_W(3,1) > 0
%   cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou + pi/2 ) sin( theta_chohou + pi/2 ) 0 ]';
%   cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou - pi/2 ) sin( theta_chohou - pi/2 ) 0 ]';
%   cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou + pi/2 + est_Q(3,1) ) sin( theta_chohou + pi/2 + est_Q(3,1) ) 0 ]';
%   cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( theta_chohou - pi/2 + est_Q(3,1) ) sin( theta_chohou - pi/2 + est_Q(3,1) ) 0 ]';
%   cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( + pi/2 + est_Q(3,1) ) sin( + pi/2 + est_Q(3,1) ) 0 ]';
%   cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( - pi/2 + est_Q(3,1) ) sin( - pi/2 + est_Q(3,1) ) 0 ]';
  cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( + pi/2 + theta_com ) sin( + pi/2 + theta_com ) 0 ]';
  cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( - pi/2 + theta_com ) sin( - pi/2 + theta_com ) 0 ]';
%   d_POS_j4 = d_cen_L + D * [ cos( theta_chohou + pi/2 + pi/4 ) sin( theta_chohou + pi/2 + pi/4 ) 0 ]';
%   d_POS_j8 = d_cen_R + D * [ cos( theta_chohou - pi/2 + pi/4 ) sin( theta_chohou - pi/2 + pi/4 ) 0 ]';
%   d_POS_j4 = cen_L + D * [ cos( theta_chohou + pi/2 + pi/4 ), sin( theta_chohou + pi/2 + pi/4 ), 0 ]';
%   d_POS_j8 = cen_R + D * [ cos( theta_chohou - pi/2 + pi/4 ), sin( theta_chohou - pi/2 + pi/4 ), 0 ]';
%   d_POS_j4 = cen_L + D * [ cos( theta_chohou + pi/2 + pi/4 + est_Q(3,1) ), sin( theta_chohou + pi/2 + pi/4 + est_Q(3,1) ), 0 ]';
%   d_POS_j8 = cen_R + D * [ cos( theta_chohou - pi/2 + pi/4 + est_Q(3,1) ), sin( theta_chohou - pi/2 + pi/4 + est_Q(3,1) ), 0 ]';
  d_POS_j4 = cen_L + D * [ cos( + pi/2 + pi/4 + theta_com ), sin( + pi/2 + pi/4 + theta_com ), 0 ]';
  d_POS_j8 = cen_R + D * [ cos( - pi/2 + pi/4 + theta_com ), sin( - pi/2 + pi/4 + theta_com ), 0 ]';
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', cen_L );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', cen_R );
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j7' 0 ]' );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j3' 0 ]' );
%   d_POS_j4 = d_geo - DD * [ 1 0 0 ]';
%   d_POS_j8 = d_geo + DD * [ 1 0 0 ]';
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', ten1 );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', ten3 );
  elseif est_W(3,1) < 0
% %   cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( -pi/2 - theta_chohou ) sin( -pi/2 - theta_chohou ) 0 ]';
% %   cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( pi/2 - theta_chohou ) sin( pi/2 - theta_chohou ) 0 ]';
% %   cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( -pi/2 - theta_chohou + est_Q(3,1) ) sin( -pi/2 - theta_chohou + est_Q(3,1) ) 0 ]';
% %   cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( pi/2 - theta_chohou + est_Q(3,1) ) sin( pi/2 - theta_chohou + est_Q(3,1) ) 0 ]';
% %   cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( - pi/2 + est_Q(3,1) ) sin( - pi/2 + est_Q(3,1) ) 0 ]';
% %   cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( + pi/2 + est_Q(3,1) ) sin( + pi/2 + est_Q(3,1) ) 0 ]';
%   cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( - pi/2 + theta_com ) sin( - pi/2 + theta_com ) 0 ]';
%   cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( + pi/2 + theta_com ) sin( + pi/2 + theta_com ) 0 ]';
% %   d_POS_j4 = d_cen_L + D * [ cos( -pi/2 - theta_chohou - pi/4 ) sin( -pi/2 - theta_chohou - pi/4 ) 0 ]';
% %   d_POS_j8 = d_cen_R + D * [ cos( pi/2 - theta_chohou - pi/4 ) sin( pi/2 - theta_chohou - pi/4 ) 0 ]';
% %   d_POS_j4 = cen_L + D * [ cos( -pi/2 - theta_chohou - pi/4 ), sin( -pi/2 - theta_chohou - pi/4 ), 0 ]';
% %   d_POS_j8 = cen_R + D * [ cos( pi/2 - theta_chohou - pi/4 ), sin( pi/2 - theta_chohou - pi/4 ), 0 ]';
% %   d_POS_j4 = cen_L + D * [ cos( -pi/2 - theta_chohou - pi/4 + est_Q(3,1) ), sin( -pi/2 - theta_chohou - pi/4 + est_Q(3,1) ), 0 ]';
% %   d_POS_j8 = cen_R + D * [ cos( pi/2 - theta_chohou - pi/4 + est_Q(3,1) ), sin( pi/2 - theta_chohou - pi/4 + est_Q(3,1) ), 0 ]';
% %   d_POS_j4 = cen_L + D * [ cos( - pi/2 - pi/4 + est_Q(3,1) ), sin( - pi/2 - pi/4 + est_Q(3,1) ), 0 ]';
% %   d_POS_j8 = cen_R + D * [ cos( + pi/2 - pi/4 + est_Q(3,1) ), sin( + pi/2 - pi/4 + est_Q(3,1) ), 0 ]';
%   d_POS_j4 = cen_L + D * [ cos( - pi/2 - pi/4 + theta_com ), sin( - pi/2 - pi/4 + theta_com ), 0 ]';
%   d_POS_j8 = cen_R + D * [ cos( + pi/2 - pi/4 + theta_com ), sin( + pi/2 - pi/4 + theta_com ), 0 ]';
%   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', cen_L );
%   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', cen_R );
% %   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j7' 0 ]' );
% %   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j3' 0 ]' );
% %   d_POS_j4 = est_geo - DD * [ 1 0 0 ]';
% %   d_POS_j8 = est_geo + DD * [ 1 0 0 ]';
% %   kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', ten2 );
% %   kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', ten4 );

  cen_L = est_geo + ( half_side_cho - half_side_tan ) * [ cos( + pi/2 + theta_com ) sin( + pi/2 + theta_com ) 0 ]';
  cen_R = est_geo + ( half_side_cho - half_side_tan ) * [ cos( - pi/2 + theta_com ) sin( - pi/2 + theta_com ) 0 ]';
  d_POS_j4 = cen_L + D * [ cos( + pi/2 + pi/4 + theta_com ), sin( + pi/2 + pi/4 + theta_com ), 0 ]';
  d_POS_j8 = cen_R + D * [ cos( - pi/2 + pi/4 + theta_com ), sin( - pi/2 + pi/4 + theta_com ), 0 ]';
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', cen_L );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', cen_R );

  else
  d_POS_j4 = d_geo - D_0 * [ 1 0 0 ]';   % �ڕW���ʒu
  d_POS_j8 = d_geo + D_0 * [ 1 0 0 ]';
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', est_geo );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', est_geo );
  end



  d_q_joints(3,1) = kakuL;   % �ڕW���p�x
  d_q_joints(7,1) = kakuR;

  x_L = d_POS_j4;
  x_R = d_POS_j8 ;

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




% �֐߈ʒu�ۑ�
POS_j4_tmp = POS_j4;
POS_j8_tmp = POS_j8;



 end