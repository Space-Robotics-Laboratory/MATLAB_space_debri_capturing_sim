function [ contactflag_L, F_NN_L, FR_N_L, FR_T_L, norm_L, tang_L, FR_L, delta_L, deltaVel_L, delta_tmp_L, TB0_L, DB0_L, curPosAP33_L, curPosAP3_delta_L, curPosAP3_tmp_L, curPosBP33_L, PointC_L, PointC_tmp_L ] ...
= Maru_FourTips_LEFT_CONTACT_v01( SV_d, SV_tm, d_time, r_target, r_tip, contact_flag_L, l_min, l_surf_L, tm_geo, Rg_d, POS_eL, delta_tmp_L, curPosAP3_tmp_L, PointC_tmp_L, cof, kkk, ccc )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ���� %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if     contact_flag_L > 0   % �ڐG���Ă�����

% �߂荞�ݗʐݒ�
   delta_L = abs( r_tip - l_surf_L );   % ���l
% �߂荞�ݑ��x�ݒ�
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_L = delta_L;   % �X�V
% �ڐG�ʊp�x
    if tm_geo(1,1) == POS_eL(1,1)   % ��担���S�ƃ^�[�Q�b�g���S������x���W�̂Ƃ�    
      if tm_geo(2,1) > POS_eL(2,1)   % ��担�̐^��Ƀ^�[�Q�b�g������ꍇ
          theta_yaw = pi/2;   % ���ɍ����C�E�Ƀ^�[�Q�b�g�������Ԃ��p�x0�Ƃ��āC�ڐG�ʂ̓�/2
          norm_L = [ cos(-theta_yaw) sin(-theta_yaw) 0 ]';   %�@���P�ʃx�N�g��(�^�[�Q�b�g�O�������C��悪�󂯂�@���͂̌���) [0 -1 0]
      else                                 % ��担�̐^���Ƀ^�[�Q�b�g������ꍇ
          theta_yaw = -pi/2;   % ���ɍ����C�E�Ƀ^�[�Q�b�g�������Ԃ��p�x0�Ƃ��āC�ڐG�ʂ�-��/2
          norm_L = [ cos(-theta_yaw) sin(-theta_yaw) 0 ]';   % [0 1 0]
      end
    else   % ��担���S�ƃ^�[�Q�b�g���S���قȂ�x���W�Ƃ�
          theta_yaw = atan2( ( POS_eL(2,1) - tm_geo(2,1) ), ( POS_eL(1,1) - tm_geo(1,1) ) );   % �t���ځ��p�x���o  ��1'
          norm_L = [ cos(theta_yaw) sin(theta_yaw) 0 ]';
    end

% �������W�n����݂��ڐG�ʒu
    PointC_L = tm_geo + r_target * norm_L;
    curPosAP3_L = PointC_L;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_L = PointC_L;   % ���{�b�g�p�ɕۑ�
    PointC_vel_L = ( PointC_L - PointC_tmp_L ) / d_time;   % ����(���x)
    PointC_tmp_L = PointC_L;   % �X�V
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_L = curPosAP3_L - SV_tm.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_L = rpy2dc( SV_tm.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_LL = TB0_L * curPosAP3_L;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_L = tilde( curPosAP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_L = curPosAP3_LL;   % �X�V
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3 = curPosBP3 - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_LL = DB0_L * curPosBP3_L;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_L = tilde( curPosBP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_L >= 0  ||  r_tip - l_surf_L >= 0  ||  l_min >= ( sqrt( ( tm_geo(1,1) - POS_eL(1,1) )^2 + ( tm_geo(2,1) - POS_eL(2,1) )^2 ) - r_tip )  % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_L = 1;
%       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % �ڐG�p�x�v�Z
%       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
%       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
      kw = kkk;
      cw = ccc;
      F_NN_L = kw * delta_L + cw * deltaVel_L;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
       if F_NN_L < 0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
          contactflag_L = 0;
          F_NN_L = 0;   % �ڐG�� = 0
       end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_L = 0;
      F_NN_L = 0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_L >= 0  ||  r_tip - l_surf_L >= 0  ||  l_min >= ( sqrt( ( tm_geo(1,1) - POS_eL(1,1) )^2 + ( tm_geo(2,1) - POS_eL(2,1) )^2 ) - r_tip )

    r_L = sqrt( ( SV_tm.R0(1,1) - PointC_L(1,1) )^2 + ( SV_tm.R0(2,1) - PointC_L(2,1) )^2 );   % �d�S�ʒu����ڐG�_�܂ł̋���
    tang_L = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_L(2,1) + r_L * SV_tm.w0(3,1) ) )' * norm_L;

  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_L = 0;   % �ڐG�t���O = 0
        tang_L = zeros(3,1);
  end



else   % �ڐG���Ă��Ȃ��Ƃ�(�ꉞ)
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


FR_N_L = F_NN_L * norm_L;   % ���{�b�g�����󂯂�@����
FR_T_L = cof * F_NN_L * tang_L;   % ��悪�󂯂門�C��
FR_L = FR_N_L + FR_T_L;


end