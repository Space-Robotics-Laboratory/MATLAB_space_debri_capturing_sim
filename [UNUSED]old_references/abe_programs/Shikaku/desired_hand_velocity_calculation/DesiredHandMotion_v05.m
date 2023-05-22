function [ xe_des, ve_des, ae_des, d_ten1, d_ten2, d_ten3, d_ten4, POS_j4_tmp, POS_j8_tmp, d_POS_j4, d_POS_j8, VEL_j4, VEL_j8, ...
           d_QL_3, d_QR_7, aL, aR, q_joints, q_wrist, qd_joints, qd_wrist, delta_t, t0, Phase, catchtime ] ...
= DesiredHandMotion_v05( d_time, time, minus_time, minus_time_2, minus_time_3, SV_d, SV_ts, Obs1, Obs2, est_P, est_V, est_Q, est_q, est_W, est_geo, ...
                         POS_j2, POS_j6, POS_j3, POS_j7, POS_j4, POS_j8, POS_j4_tmp, POS_j8_tmp, d_POS_j4, d_POS_j8, ...
                         D, D_0, d_QL_3, d_QR_7, aL, aR, d_ten1, d_ten2, d_ten3, d_ten4, delta_t, delta_t_2, delta_t_3, t0, ...
                         contactflag_L1, contactflag_L2, contactflag_R1, contactflag_R2, catchtime, POS_eL1, POS_eL2, POS_eR1, POS_eR2, hokakutime, DHD, Phase, t1 )
D = D*1.2;
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


 POS_j2_st = 2 * POS_j3 - POS_j2;
 POS_j6_st = 2 * POS_j7 - POS_j6;

 Q_pi_4 = est_Q(3,1) - pi/4;
%  Q_target = [ cos( Q_pi_4 ), sin( Q_pi_4 ), 0 ]';


%%% �ڕW�ʒu�C���x�C�����x�C�p�x�C�p���x�C�e�����x��ݒ� %%%

switch(Phase)
    case 0
         d_POS_j4 = [ POS_j4' 0 ]';   % (3,1)
         d_POS_j8 = [ POS_j8' 0 ]';   % (3,1)
         d_q_joints = q_joints;   % �ڕW���p�x
         d_VEL_j4 = VEL_j4;   % (2,1)   % �ڕW��摬�x   % (2,1)
         d_VEL_j8 = VEL_j8;   % (2,1)
         d_qd_joints = qd_joints;   % �ڕW���p���x

         xeL_des = [ d_POS_j4' zeros(1,2) d_q_joints(3,1) ]';   % �ڕW���ʒu�Ɗp�x
         xeR_des = [ d_POS_j8' zeros(1,2) d_q_joints(7,1) ]';
         veL_des = [ d_VEL_j4' zeros(1,3) d_qd_joints(3,1) ]';   % �ڕW��摬�x�Ɗp���x
         veR_des = [ d_VEL_j8' zeros(1,3) d_qd_joints(7,1) ]';
         aeL_des = zeros(6,1);   % �ڕW�������x
         aeR_des = zeros(6,1);
         
         
        if (time < t0 + minus_time && time >= minus_time)
        Phase = 1;
        end
        
        %%% Phase1
        %%% �^�[�Q�b�g��]�p��0�ɂȂ�܂ł�Phase
    case 1
        d_POS_j4 = [ POS_j4' 0 ]';   % (3,1)
         d_POS_j8 = [ POS_j8' 0 ]';   % (3,1)
         d_q_joints = q_joints;   % �ڕW���p�x
         d_VEL_j4 = VEL_j4;   % (2,1)   % �ڕW��摬�x   % (2,1)
         d_VEL_j8 = VEL_j8;   % (2,1)
         d_qd_joints = qd_joints;   % �ڕW���p���x

         xeL_des = [ d_POS_j4' zeros(1,2) d_q_joints(3,1) ]';   % �ڕW���ʒu�Ɗp�x
         xeR_des = [ d_POS_j8' zeros(1,2) d_q_joints(7,1) ]';
         veL_des = [ d_VEL_j4' zeros(1,3) d_qd_joints(3,1) ]';   % �ڕW��摬�x�Ɗp���x
         veR_des = [ d_VEL_j8' zeros(1,3) d_qd_joints(7,1) ]';
         aeL_des = zeros(6,1);   % �ڕW�������x
         aeR_des = zeros(6,1);
         
         
         if (time == ( t0 + minus_time ))
             Phase = 2;
         else
             Phase = 1;
         end

 
%%% Phase 2
%%% �ڕW���ʒu�ƃX�v���C���Ȑ��̌W���A�ڕW���p�x�𓱏o
    case 2

  delta_q = ( pi/4 - est_q );   % �^�[�Q�b�g�p����/4�ɂȂ�܂ł̊p�x��q       ���͉�]���Ȃ̂Ń�/4
  delta_t = delta_q / est_W(3,1);   % �^�[�Q�b�g�p����/4�ɂȂ�܂ł̎��ԃ�t

  d_ten1 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten1 - est_P ) + est_V * delta_t;   % �e���_�̃�t��̈ʒu���v�Z
  d_ten2 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten2 - est_P ) + est_V * delta_t;
  d_ten3 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten3 - est_P ) + est_V * delta_t;
  d_ten4 = est_P + rpy2dc( 0, 0, delta_q )' * ( ten4 - est_P ) + est_V * delta_t;

  L_target = [ 1 0 0 ]';
  L_target_2 = [ 1.7 2.1 0 ]';
  if     est_W(3,1) > 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
    %d_POS_j4 = d_ten1 - D * L_target;   % �ڕW���ʒu
    %d_POS_j4 = [ POS_j4' 0 ]';
    d_POS_j4 = d_ten1 - D * L_target_2;
    d_POS_j8 = d_ten3 + D * L_target;
    %d_POS_j8 = [ POS_j8' 0 ]';
    %d_POS_j8 = d_ten3 + D * L_target_2;
    tenL = ten1;
    tenR = ten3;
      d_QL_3 = atan2( ( ten1(2,1) - POS_j3(2,1) ), ( ten1(1,1) - POS_j3(1,1) ) );
      d_QR_7 = atan2( ( ten3(2,1) - POS_j7(2,1) ), ( ten3(1,1) - POS_j7(1,1) ) );
    
  elseif est_W(3,1) < 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
    d_POS_j4 = d_ten2 - D * L_target;   % �ڕW���ʒu
    d_POS_j8 = d_ten4 + D * L_target;
    tenL = ten2;
    tenR = ten4;
      d_QL_3 = atan2( ( ten2(2,1) - POS_j3(2,1) ), ( ten2(1,1) - POS_j3(1,1) ) );
      d_QR_7 = atan2( ( ten4(2,1) - POS_j7(2,1) ), ( ten4(1,1) - POS_j7(1,1) ) );

  else                    % �^�[�Q�b�g�p���x��0�̂Ƃ�
    d_POS_j4 = ( ten1 + ten2 ) / 2 - D_0 * L_target;   % �ڕW���ʒu
    d_POS_j8 = ( ten3 + ten4 ) / 2 + D_0 * L_target;
      d_QL_3 = 0;
      d_QR_7 = pi;

  end
  %{
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', tenL );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', tenR );
    d_q_joints(3,1) = kakuL;   % �ڕW���p�x
    d_q_joints(7,1) = kakuR;
  %}

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

   d_q_joints = q_joints;   % �ڕW���p�x
   %d_q_joints(3,1) = kakuL;


%SV_d.w0(3,1) = SV_d.w0(3,1) - (((kakuR - d_q_joints(7,1))) / delta_t) + (((kakuL - d_q_joints(3,1))) / delta_t);

 %xeL_des = [ x_L' zeros(1,3) 0 ]';
 %xeR_des = [ x_R' zeros(1,3) 0 ]';
 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);
 
 
 Phase = 3;



%%% Phase 3
%%% �e�X�e�b�v�ɂ�����ڕW���p���x���Z�o
%%% ����ڕW�ʒu�܂Ŏ����Ă���Phase
    case 3

  if     est_W(3,1) > 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
    tenL = ten1;
    tenR = ten3;
     d_QL_3 = atan2( ( ten1(2,1) - POS_j3(2,1) ), ( ten1(1,1) - POS_j3(1,1) ) );
     d_QR_7 = atan2( ( ten3(2,1) - POS_j7(2,1) ), ( ten3(1,1) - POS_j7(1,1) ) );
    
  elseif est_W(3,1) < 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
    tenL = ten2;
    tenR = ten4;
     d_QL_3 = atan2( ( ten2(2,1) - POS_j3(2,1) ), ( ten2(1,1) - POS_j3(1,1) ) );
     d_QR_7 = atan2( ( ten4(2,1) - POS_j7(2,1) ), ( ten4(1,1) - POS_j7(1,1) ) );

  else                    % �^�[�Q�b�g�p���x��0�̂Ƃ�
    tenL = ( ten1 + ten2 ) / 2;
    tenR = ( ten3 + ten4 ) / 2;
     d_QL_3 = 0;
     d_QR_7 = pi;

  end
  
  d_q_joints = q_joints;   % �ڕW���p�x
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', SV_ts.R0 );
  %kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', tenR );
    d_q_joints(3,1) = kakuL;   % �ڕW���p�x
    %d_q_joints(7,1) = kakuR;
  

  x_L = aL(:,1) + aL(:,2) * ( time - ( t0 + minus_time ) ) + aL(:,3) * ( time - ( t0 + minus_time ) )^2 + aL(:,4) * ( time - t0 - minus_time )^3;
  x_R = aR(:,1) + aR(:,2) * ( time - ( t0 + minus_time ) ) + aR(:,3) * ( time - ( t0 + minus_time ) )^2 + aR(:,4) * ( time - t0 - minus_time )^3;

% % �p�ɐ�����
%   kL = kakudo( est_P, [ x_L' 0 ]', tenL );
%   kR = kakudo( est_P, [ x_R' 0 ]', tenR );
%   x_LL = est_P + rpy2dc( 0, 0, kL )' * ( [ x_L' 0 ]' - est_P );
%   x_RR = est_P + rpy2dc( 0, 0, kR )' * ( [ x_R' 0 ]' - est_P );
%   x_L = x_LL(1:2,1);
%   x_R = x_RR(1:2,1);

%d_q_joints(3,1) = kakuL;


%SV_d.w0(3,1) = SV_d.w0(3,1) - (((kakuR - d_q_joints(7,1))) / delta_t) - (((kakuL - d_q_joints(3,1))) / delta_t);

 %xeL_des = [ x_L' zeros(1,3) 0 ]';
 %xeR_des = [ x_R' zeros(1,3) 0 ]';
 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);
 
 
 if ((time - t0 - minus_time) > delta_t)
     Phase = 4;
 else
     Phase = 3;
 end
 
%%% Phase 4
%%% �ҋ@Phase
    case 4

%     if     est_W(3,1) > 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
    tenL = ten1;
    tenR = ten3;
%   elseif est_W(3,1) < 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
%     tenL = ten2;
%     tenR = ten4;
%   else                    % �^�[�Q�b�g�p���x��0�̂Ƃ�
%     tenL = ( ten1 + ten2 ) / 2;
%     tenR = ( ten3 + ten4 ) / 2;
%     end

%{
  thetaTL = atan2( ( tenL(2,1) - tenR(2,1) ), ( tenL(1,1) - tenR(1,1) ) );
  thetaTR = atan2( ( tenR(2,1) - tenL(2,1) ), ( tenR(1,1) - tenL(1,1) ) );
  x_L = tenL(1:2,1) + D * [ cos(thetaTL), sin(thetaTL) ]';
  x_R = tenR(1:2,1) + D * [ cos(thetaTR), sin(thetaTR) ]';
%}
%{
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j7' 0 ]' );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j3' 0 ]' );
  d_q_joints(3,1) = kakuL;   % �ڕW���p�x
  d_q_joints(7,1) = kakuR;
%}
  d_q_joints = q_joints;   % �ڕW���p�x
  
  x_L = aL(:,1) + aL(:,2) * ( time - ( t0 + minus_time ) ) + aL(:,3) * ( time - ( t0 + minus_time ) )^2 + aL(:,4) * ( time - t0 - minus_time )^3;
  x_R = aR(:,1) + aR(:,2) * ( time - ( t0 + minus_time ) ) + aR(:,3) * ( time - ( t0 + minus_time ) )^2 + aR(:,4) * ( time - t0 - minus_time )^3;
  
  
  %SV_d.w0 = SV_d.w0 - (((kakuR - d_q_joints(7,1))) / delta_t) - (((kakuL - d_q_joints(3,1))) / delta_t);

 %xeL_des = [ x_L' zeros(1,3) 0 ]';
 %xeR_des = [ x_R' zeros(1,3) 0 ]';
 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);
 
 
 if ((time - t1) > 0.5) %&& contactflag_L2==1 ) %�ҋ@���Ԃ͎����Őݒ�
     Phase = 5;
 else
     Phase = 4;
 end
 
 
 %%% Phase 5
 %%% �ڕW���ʒu�𓱏o
    case 5
%D = 0.020;
        
L_target = [ 1 0 0 ]';
%L_target_2 = [ 5 0 0 ]';
 % if     est_W(3,1) > 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
 
 %{
    d_POS_j4(1,1) = SV_ts.R0(1,1) + ((DHD + D) / DHD) * (ten1(1,1) - SV_ts.R0(1,1));   % �ڕW���ʒu
    d_POS_j4(2,1) = SV_ts.R0(2,1) + ((DHD + D) / DHD) * (ten1(2,1) - SV_ts.R0(2,1));
    d_POS_j4(3,1) = SV_ts.R0(3,1) + ((DHD + D) / DHD) * (ten1(3,1) - SV_ts.R0(3,1));

    d_POS_j8(1,1) = SV_ts.R0(1,1) + ((DHD + D) / DHD) * (ten3(1,1) - SV_ts.R0(1,1));
    d_POS_j8(2,1) = SV_ts.R0(2,1) + ((DHD + D) / DHD) * (ten3(2,1) - SV_ts.R0(2,1));
    d_POS_j8(3,1) = SV_ts.R0(3,1) + ((DHD + D) / DHD) * (ten3(3,1) - SV_ts.R0(3,1));
%}
    %d_POS_j4 = [ POS_j4' 0 ]';
    %d_POS_j8 = [ POS_j8' 0 ]';
    
    d_POS_j4 = ten1 - D * L_target;
    %d_POS_j8 = ten3 + D * L_target_2;
    d_POS_j8 = ten3 + D * L_target;
%{
    tenL = ten1;
    tenR = ten3;
      d_QL_3 = atan2( ( ten1(2,1) - POS_j3(2,1) ), ( ten1(1,1) - POS_j3(1,1) ) );
      d_QR_7 = atan2( ( ten3(2,1) - POS_j7(2,1) ), ( ten3(1,1) - POS_j7(1,1) ) );
    %}
      %{
  elseif est_W(3,1) < 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
    d_POS_j4 = ten2 - D * L_target;   % �ڕW���ʒu
    d_POS_j8 = ten4 + D * L_target;
    tenL = ten2;
    tenR = ten4;
      d_QL_3 = atan2( ( ten2(2,1) - POS_j3(2,1) ), ( ten2(1,1) - POS_j3(1,1) ) );
      d_QR_7 = atan2( ( ten4(2,1) - POS_j7(2,1) ), ( ten4(1,1) - POS_j7(1,1) ) );

  else                    % �^�[�Q�b�g�p���x��0�̂Ƃ�
    d_POS_j4 = ( ten1 + ten2 ) / 2 - D_0 * L_target;   % �ڕW���ʒu
    d_POS_j8 = ( ten3 + ten4 ) / 2 + D_0 * L_target;
      d_QL_3 = 0;
      d_QR_7 = pi;

  end
      %}
    
    
  %{
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', tenL );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', tenR );
    d_q_joints(3,1) = kakuL;   % �ڕW���p�x
    d_q_joints(7,1) = kakuR;
  %}
    
%{
  %   �ڕW�ʒu���䁨�X�v���C���Ȑ��O�ՒǏ]   �Ƃ肠������摬�x����
  x0L = POS_j4;   % �����ʒu   % (2,1)
  v0L = zeros(2,1);   % �������x
  xfL = d_POS_j4(1:2,1);   % �ڕW�ʒu
  vfL = zeros(2,1);   % �ڕW���x
    a0L = x0L;   % �W���ɑ��
    a1L = v0L;
    a2L = ( -3 * ( x0L - xfL ) - delta_t_2 * ( 2 * v0L + vfL ) )/( delta_t_2 )^2;
    a3L = ( 2 * ( x0L - xfL ) - delta_t_2 * ( v0L + vfL ) )/( delta_t_2 )^3;
  aL = [ a0L, a1L, a2L, a3L ];

  x0R = POS_j8;   % �����ʒu   % (2,1)
  v0R = zeros(2,1);   % �������x
  xfR = d_POS_j8(1:2,1);
  vfR = zeros(2,1);   % �ڕW���x
    a0R = x0R;   % �W���ɑ��
    a1R = v0R;
    a2R = ( -3 * ( x0R - xfR ) - delta_t_2 * ( 2 * v0R + vfR ) )/( delta_t_2 )^2;
    a3R = ( 2 * ( x0R - xfR ) - delta_t_2 * ( v0R + vfR ) )/( delta_t_2 )^3;
  aR = [ a0R, a1R, a2R, a3R ];
%}
      
  x_L = POS_j4;   
  x_R = POS_j8;
  
    
% % �p�ɐ�����
%   kL = kakudo( est_P, [ x_L' 0 ]', tenL );
%   kR = kakudo( est_P, [ x_R' 0 ]', tenR );
%   x_LL = est_P + rpy2dc( 0, 0, kL )' * ( [ x_L' 0 ]' - est_P );
%   x_RR = est_P + rpy2dc( 0, 0, kR )' * ( [ x_R' 0 ]' - est_P );
%   x_L = x_LL(1:2,1);
%   x_R = x_RR(1:2,1);

   d_q_joints = q_joints;   % �ڕW���p�x
   %d_q_joints(3,1) = kakuL;
%d_q_joints(3,1) = -1.2;
%d_q_joints(7,1) = 1.2;

%SV_d.w0(3,1) = SV_d.w0(3,1) - (((kakuR - d_q_joints(7,1))) / delta_t) + (((kakuL - d_q_joints(3,1))) / delta_t);

 %xeL_des = [ x_L' zeros(1,3) 0 ]';
 %xeR_des = [ x_R' zeros(1,3) 0 ]';
 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);
 

 Phase = 6;


%%% Phase 6
%%% �ڕW��摬�x�𓱏o
    case 6

  %{
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', tenL );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', tenR );
    d_q_joints(3,1) = kakuL;   % �ڕW���p�x
    d_q_joints(7,1) = kakuR;
  %}
     %}
     
    d_VEL_j4 = ( d_POS_j4(1:2,1) - POS_j4_tmp ) / delta_t_2;   % (2,1)   % �ڕW��摬�x   % (2,1)
    d_VEL_j8 = ( d_POS_j8(1:2,1) - POS_j8_tmp ) / delta_t_2;   % (2,1)
    
    VEL_j4 = d_VEL_j4;
    VEL_j8 = d_VEL_j8;
     
  x_L = POS_j4 + VEL_j4 * d_time;
  x_R = POS_j8 + VEL_j8 * d_time;
  

% % �p�ɐ�����
%   kL = kakudo( est_P, [ x_L' 0 ]', tenL );
%   kR = kakudo( est_P, [ x_R' 0 ]', tenR );
%   x_LL = est_P + rpy2dc( 0, 0, kL )' * ( [ x_L' 0 ]' - est_P );
%   x_RR = est_P + rpy2dc( 0, 0, kR )' * ( [ x_R' 0 ]' - est_P );
%   x_L = x_LL(1:2,1);
%   x_R = x_RR(1:2,1);

d_q_joints = q_joints;   % �ڕW���p�x
%d_q_joints(3,1) = kakuL;

%d_q_joints(3,1) = -1.2;
%d_q_joints(7,1) = 1.2;

%SV_d.w0(3,1) = SV_d.w0(3,1) - (((kakuR - d_q_joints(7,1))) / delta_t) - (((kakuL - d_q_joints(3,1))) / delta_t);

 %xeL_des = [ x_L' zeros(1,3) 0 ]';
 %xeR_des = [ x_R' zeros(1,3) 0 ]';
 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);
 
 
 if ((time - minus_time_2) > delta_t_2)
     Phase = 7;
 else
     Phase = 6;
 end


%%% Phase 7
%%% �ҋ@Phase
    case 7

%     if     est_W(3,1) > 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
    tenL = ten1;
    tenR = ten3;
%   elseif est_W(3,1) < 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
%     tenL = ten2;
%     tenR = ten4;
%   else                    % �^�[�Q�b�g�p���x��0�̂Ƃ�
%     tenL = ( ten1 + ten2 ) / 2;
%     tenR = ( ten3 + ten4 ) / 2;
%     end
%{
  thetaTL = atan2( ( tenL(2,1) - tenR(2,1) ), ( tenL(1,1) - tenR(1,1) ) );
  thetaTR = atan2( ( tenR(2,1) - tenL(2,1) ), ( tenR(1,1) - tenL(1,1) ) );
  x_L = tenL(1:2,1) + D * [ cos(thetaTL), sin(thetaTL) ]';
  x_R = tenR(1:2,1) + D * [ cos(thetaTR), sin(thetaTR) ]';
%}
  x_L = aL(:,1) + aL(:,2) * ( time - ( minus_time_2 ) ) + aL(:,3) * ( time - ( minus_time_2 ) )^2 + aL(:,4) * ( time - minus_time_2 )^3;
  x_R = aR(:,1) + aR(:,2) * ( time - ( minus_time_2 ) ) + aR(:,3) * ( time - ( minus_time_2 ) )^2 + aR(:,4) * ( time - minus_time_2 )^3;
  
  %x_L = aL(:,1) + aL(:,2) * ( time - ( t0 + minus_time ) ) + aL(:,3) * ( time - ( t0 + minus_time ) )^2 + aL(:,4) * ( time - t0 - minus_time )^3;
  %x_R = aR(:,1) + aR(:,2) * ( time - ( t0 + minus_time ) ) + aR(:,3) * ( time - ( t0 + minus_time ) )^2 + aR(:,4) * ( time - t0 - minus_time )^3;

  %{
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j7' 0 ]' );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j3' 0 ]' );
  d_q_joints(3,1) = kakuL;   % �ڕW���p�x
  d_q_joints(7,1) = kakuR;
  %}
  
  d_q_joints = q_joints;   % �ڕW���p�x
  %d_q_joints(3,1) = kakuL;
  
  %d_q_joints(3,1) = -1.2;
  %d_q_joints(7,1) = 1.2;
  
  
  %SV_d.w0 = SV_d.w0 - (((kakuR - d_q_joints(7,1))) / delta_t) - (((kakuL - d_q_joints(3,1))) / delta_t);

 %xeL_des = [ x_L' zeros(1,3) 0 ]';
 %xeR_des = [ x_R' zeros(1,3) 0 ]';
 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);


%{
%%% �ߊl�t�F�[�Y Phase8
aaL = ( POS_eL1(2,1) - POS_eL2(2,1) )/( POS_eL1(1,1) - POS_eL2(1,1) );
bbL = ( POS_eL2(2,1)*POS_eL1(1,1) - POS_eL1(2,1)*POS_eL2(1,1) )/( POS_eL1(1,1) - POS_eL2(1,1) );
aaR = ( POS_eR1(2,1) - POS_eR2(2,1) )/( POS_eR1(1,1) - POS_eR2(1,1) );
bbR = ( POS_eR2(2,1)*POS_eR1(1,1) - POS_eR1(2,1)*POS_eR2(1,1) )/( POS_eR1(1,1) - POS_eR2(1,1) );

if  ten1(1,1) < (tenL(2,1)-bbL)/aaL  &&  (tenR(2,1)-bbR)/aaR < tenR(1,1)  ||  Phase == 8
 %}
    catchtime = catchtime + d_time;
if catchtime >= hokakutime
Phase = 8;
end

    case 8
        %     if     est_W(3,1) > 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
    tenL = ten1;
    tenR = ten3;
%   elseif est_W(3,1) < 0   % �^�[�Q�b�g�p���x�����̂Ƃ�
%     tenL = ten2;
%     tenR = ten4;
%   else                    % �^�[�Q�b�g�p���x��0�̂Ƃ�
%     tenL = ( ten1 + ten2 ) / 2;
%     tenR = ( ten3 + ten4 ) / 2;
%     end

  x_L = aL(:,1) + aL(:,2) * ( time - ( minus_time_2 ) ) + aL(:,3) * ( time - ( minus_time_2 ) )^2 + aL(:,4) * ( time - minus_time_2 )^3;
  x_R = aR(:,1) + aR(:,2) * ( time - ( minus_time_2 ) ) + aR(:,3) * ( time - ( minus_time_2 ) )^2 + aR(:,4) * ( time - minus_time_2 )^3;
  
  %x_L = aL(:,1) + aL(:,2) * ( time - ( t0 + minus_time ) ) + aL(:,3) * ( time - ( t0 + minus_time ) )^2 + aL(:,4) * ( time - t0 - minus_time )^3;
  %x_R = aR(:,1) + aR(:,2) * ( time - ( t0 + minus_time ) ) + aR(:,3) * ( time - ( t0 + minus_time ) )^2 + aR(:,4) * ( time - t0 - minus_time )^3;

  %{
  kakuL = kakudo( [ POS_j3' 0 ]', [ POS_j2_st' 0 ]', [ POS_j7' 0 ]' );
  kakuR = kakudo( [ POS_j7' 0 ]', [ POS_j6_st' 0 ]', [ POS_j3' 0 ]' );
  d_q_joints(3,1) = kakuL;   % �ڕW���p�x
  d_q_joints(7,1) = kakuR;
  %}
  
  d_q_joints = q_joints;   % �ڕW���p�x
  %d_q_joints(3,1) = kakuL;
  
  %d_q_joints(3,1) = -1.2;
  %d_q_joints(7,1) = 1.2;
  
  %SV_d.w0 = SV_d.w0 - (((kakuR - d_q_joints(7,1))) / delta_t) - (((kakuL - d_q_joints(3,1))) / delta_t);

 %xeL_des = [ x_L' zeros(1,3) 0 ]';
 %xeR_des = [ x_R' zeros(1,3) 0 ]';
 xeL_des = [ x_L' zeros(1,3) d_q_joints(3,1) ]';
 xeR_des = [ x_R' zeros(1,3) d_q_joints(7,1) ]';
 veL_des = zeros(6,1);
 veR_des = zeros(6,1);
 aeL_des = zeros(6,1);
 aeR_des = zeros(6,1);

    otherwise
    catchtime = 0;
end

xe_des = [ xeL_des' xeR_des' ]';
ve_des = [ veL_des' veR_des' ]';
ae_des = [ aeL_des' aeR_des' ]';
         

% �֐߈ʒu�ۑ�
POS_j4_tmp = POS_j4;
POS_j8_tmp = POS_j8;

 end