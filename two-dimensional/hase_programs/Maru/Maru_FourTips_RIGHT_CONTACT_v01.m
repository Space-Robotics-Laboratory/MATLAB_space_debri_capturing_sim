function [ contactflag_R, F_NN_R, FR_N_R, FR_T_R, norm_R, tang_R, FR_R, delta_R, deltaVel_R, delta_tmp_R, TB0_R, DB0_R, curPosAP33_R, curPosAP3_delta_R, curPosAP3_tmp_R, curPosBP33_R, PointC_R, PointC_tmp_R ] ...
= Maru_FourTips_RIGHT_CONTACT_v01( SV_d, SV_tm, d_time, r_target, r_tip, contact_flag_R, l_min, l_surf_R, tm_geo, Rg_d, POS_eR, delta_tmp_R, curPosAP3_tmp_R, PointC_tmp_R, cof, kkk, ccc )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ���� %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if     contact_flag_R > 0   % �ڐG���Ă�����

% �߂荞�ݗʐݒ�
   delta_R = abs( r_tip - l_surf_R );   % ���l
% �߂荞�ݑ��x�ݒ�
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_R = delta_R;   % �X�V
% �ڐG�ʊp�x
    if tm_geo(1,1) == POS_eR(1,1)   % ��担���S�ƃ^�[�Q�b�g���S������x���W�̂Ƃ�    
      if tm_geo(2,1) > POS_eR(2,1)   % ��担�̐^��Ƀ^�[�Q�b�g������ꍇ
          theta_yaw = -pi/2;   % ���ɍ����C�E�Ƀ^�[�Q�b�g�������Ԃ��p�x0�Ƃ��āC�ڐG�ʂ̓�/2
          norm_R = [ cos(theta_yaw) sin(theta_yaw) 0 ]';   %�@���P�ʃx�N�g��(�^�[�Q�b�g�O�������C��悪�󂯂�@���͂̌���) [0 -1 0]
      else                                 % ��担�̐^���Ƀ^�[�Q�b�g������ꍇ
          theta_yaw = pi/2;   % ���ɍ����C�E�Ƀ^�[�Q�b�g�������Ԃ��p�x0�Ƃ��āC�ڐG�ʂ�-��/2
          norm_R = [ cos(theta_yaw) sin(theta_yaw) 0 ]';   % [0 1 0]
      end
    else   % ��担���S�ƃ^�[�Q�b�g���S���قȂ�x���W�Ƃ�
          theta_yaw = atan2( ( POS_eR(2,1) - tm_geo(2,1) ), ( POS_eR(1,1) - tm_geo(1,1) ) );   % �t���ځ��p�x���o  ��1'
          norm_R = [ cos(theta_yaw) sin(theta_yaw) 0 ]';
    end

% �������W�n����݂��ڐG�ʒu
    PointC_R = tm_geo + r_target * norm_R;
    curPosAP3_R = PointC_R;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_R = PointC_R;   % ���{�b�g�p�ɕۑ�
    PointC_vel_R = ( PointC_R - PointC_tmp_R ) / d_time;   % ����(���x)
    PointC_tmp_R = PointC_R;   % �X�V
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_R = curPosAP3_R - SV_tm.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_R = rpy2dc( SV_tm.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_RR = TB0_R * curPosAP3_R;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_R = tilde( curPosAP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_R = curPosAP3_RR;   % �X�V
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3 = curPosBP3 - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_RR = DB0_R * curPosBP3_R;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_R = tilde( curPosBP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_R >= 0  ||  r_tip - l_surf_R >= 0  ||  l_min >= ( sqrt( ( tm_geo(1,1) - POS_eR(1,1) )^2 + ( tm_geo(2,1) - POS_eR(2,1) )^2 ) - r_tip )  % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_R = 1;
%       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % �ڐG�p�x�v�Z
%       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
%       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
      kw = kkk;
      cw = ccc;
      F_NN_R = kw * delta_R + cw * deltaVel_R;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
       if F_NN_R < 0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
          contactflag_R = 0;
          F_NN_R = 0;   % �ڐG�� = 0
       end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_R = 0;
      F_NN_R = 0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_R >= 0  ||  r_tip - l_surf_R >= 0  ||  l_min >= ( sqrt( ( tm_geo(1,1) - POS_eR(1,1) )^2 + ( tm_geo(2,1) - POS_eR(2,1) )^2 ) - r_tip )

    r_R = sqrt( ( SV_tm.R0(1,1) - PointC_R(1,1) )^2 + ( SV_tm.R0(2,1) - PointC_R(2,1) )^2 );   % �d�S�ʒu����ڐG�_�܂ł̋���
    tang_R = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_R(2,1) + r_R * SV_tm.w0(3,1) ) )' * norm_R;

  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_R = 0;   % �ڐG�t���O = 0
        tang_R = zeros(3,1);
  end



else   % �ڐG���Ă��Ȃ��Ƃ�(�ꉞ)
     contactflag_R = 0;
     delta_R = 0;
     deltaVel_R = 0;
     F_NN_R = 0;
     curPosAP33_R = zeros(3,3);
     curPosBP33_R = zeros(3,3);
     TB0_R = zeros(3,3);
     DB0_R = zeros(3,3);
     curPosAP3_delta_R = zeros(3,1);
     norm_R = zeros(3,1);
     tang_R = zeros(3,1);
     PointC_R = zeros(3,1);

end


FR_N_R = F_NN_R * norm_R;   % ���{�b�g�����󂯂�@����
FR_T_R = cof * F_NN_R * tang_R;   % ��悪�󂯂門�C��
FR_R = FR_N_R + FR_T_R;


end