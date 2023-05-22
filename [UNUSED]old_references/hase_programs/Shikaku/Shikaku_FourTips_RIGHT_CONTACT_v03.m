function [ contactflag_R, F_NN_R, FR_N_R, FR_T_R, norm_R, tang_R, FR_R, delta_R, deltaVel_R, delta_tmp_R, TB0_R, DB0_R, curPosAP33_R, curPosAP3_delta_R, curPosAP3_tmp_R, curPosBP33_R, PointC_R, l_x_R ] ...
= Shikaku_FourTips_RIGHT_CONTACT_v03( SV_d, SV_ts, d_time, half_side, r_tip, contact_flag_R, contactflag_R, shikaku1, shikaku2, shikaku3, shikaku4, ...
                        l_side_min, l_corner_min, ts_geo, q0, Rg_d, POS_eR, delta_tmp_R, curPosAP3_tmp_R, l1, l2, l3, l4, gamma1, gamma2, gamma3, gamma4, cof_R, kkk, ccc )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% �E�� %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% ��悪�̈�11,22,33,44�ɂ���Ƃ�(�p�ŐڐG���Ă���Ƃ�) %%%%%%%%   % ��担�ƒ��_�Ƃ̐ڐG���C���z�I�ɉ~�Ƃ̐ڐG�Ƃ��čl����

if     contact_flag_R(1,1) > 0  ||  contact_flag_R(2,2) > 0  ||  contact_flag_R(3,3) > 0  ||  contact_flag_R(4,4) > 0

    if     contact_flag_R(1,1) > 0
        shikaku_R = shikaku1;   % �p�̃x�N�g����shikaku_R�Ƃ���   % �E���Ɉ�ԋ߂����_
        gamma_R = gamma1;
        l_tsR0 = l1;
    elseif contact_flag_R(2,2) > 0
        shikaku_R = shikaku2;   % �p�̃x�N�g����shikaku_R�Ƃ���
        gamma_R = gamma2;
        l_tsR0 = l2;
    elseif contact_flag_R(3,3) > 0
        shikaku_R = shikaku3;   % �p�̃x�N�g����shikaku_R�Ƃ���
        gamma_R = gamma3;
        l_tsR0 = l3;
    elseif contact_flag_R(4,4) > 0
        shikaku_R = shikaku4;   % �p�̃x�N�g����shikaku_R�Ƃ���
        gamma_R = gamma4;
        l_tsR0 = l4;
    end

   r_R = sqrt( ( shikaku_R(1,1) - POS_eR(1,1) )^2 + ( shikaku_R(2,1) - POS_eR(2,1) )^2 );   % �^�[�Q�b�g���S�����担���S�܂ł̋���
   l_x_R =  0.0;
% �߂荞�ݗʐݒ�
   delta_R = abs( r_tip - r_R );   % ���l
% �߂荞�ݑ��x�ݒ�
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_R = delta_R;   % �X�V
% �ڐG�ʊp�x
    if shikaku_R(1,1) == POS_eR(1,1)   % ��担���S�ƃ^�[�Q�b�g���S������x���W�̂Ƃ�
      if shikaku_R(2,1) > POS_eR(2,1)   % ��担�̐^��Ƀ^�[�Q�b�g������ꍇ
          theta_R_yaw = -pi/2;   % �E�ɉE���C���Ƀ^�[�Q�b�g�������Ԃ��p�x0�Ƃ��āC�ڐG�ʂ�-��/2
          norm_R = [ cos(-pi/2) sin(-pi/2) 0 ]';   %�@���P�ʃx�N�g��(�^�[�Q�b�g�O�������C��悪�󂯂�@���͂̌���) [0 -1 0]
      else                                 % ��担�̐^���Ƀ^�[�Q�b�g������ꍇ
          theta_R_yaw = pi/2;   % �E�ɉE���C���Ƀ^�[�Q�b�g�������Ԃ��p�x0�Ƃ��āC�ڐG�ʂ̓�/2
          norm_R = [ cos(pi/2) sin(pi/2) 0 ]';   % [0 1 0]
      end
    else   % ��担���S�ƃ^�[�Q�b�g���S���قȂ�x���W�Ƃ�
          theta_R_yaw = atan( ( shikaku_R(2,1) - POS_eR(2,1) ) / ( shikaku_R(1,1) - POS_eR(1,1) ) );   % �t���ځ��p�x���o  ��1'
          norm_R = [ cos(theta_R_yaw) sin(theta_R_yaw) 0 ]';
    end

% �������W�n����݂��ڐG�ʒu
    PointC_R = shikaku_R;
    curPosAP3_R = PointC_R;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_R = PointC_R;   % ���{�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_R = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_RR = TB0_R * curPosAP3_R;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_R = tilde( curPosAP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG��y�����̑��x(�^�[�Q�b�g)
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_R = curPosAP3_RR;   % �X�V
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_RR = DB0_R * curPosBP3_R;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_R = tilde( curPosBP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_R >= 0.0  ||  r_tip - r_R >= 0.0  ||  l_corner_min >= ( sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 ) - r_tip )  % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % �ڐG�p�x�v�Z
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
       if F_NN_R < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
          contactflag_R = 0;
          F_NN_R = 0.0;   % �ڐG�� = 0
       end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_R = 0;
      F_NN_R = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_R >= 0.0  ||  r_tip - r_R >= 0.0  ||  l_corner_min >= ( sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 ) - r_tip )

    if     contact_flag_R(1,1) > 0
        psi_R = abs( pi/2 - abs( q0 + gamma_R - theta_R_yaw ) );
        corner_vel_R = l_tsR0 * SV_ts.w0(3,1);   % �l�p�̒��_�̑��x(�^�[�Q�b�g�d�S���璸�_�܂ł̋����~�^�[�Q�b�g�̊p���x)
    elseif contact_flag_R(2,2) > 0
        psi_R = abs( q0 + gamma_R - theta_R_yaw );
        corner_vel_R = l_tsR0 * SV_ts.w0(3,1);   % �l�p�̒��_�̑��x(�^�[�Q�b�g�d�S���璸�_�܂ł̋����~�^�[�Q�b�g�̊p���x)
    elseif contact_flag_R(3,3) > 0
        psi_R = abs( pi/2 - abs( q0 + gamma_R - theta_R_yaw ) );
        corner_vel_R = l_tsR0 * SV_ts.w0(3,1);   % �l�p�̒��_�̑��x(�^�[�Q�b�g�d�S���璸�_�܂ł̋����~�^�[�Q�b�g�̊p���x)
    elseif contact_flag_R(4,4) > 0
        psi_R = abs( q0 + gamma_R - theta_R_yaw );
        corner_vel_R = l_tsR0 * SV_ts.w0(3,1);   % �l�p�̒��_�̑��x(�^�[�Q�b�g�d�S���璸�_�܂ł̋����~�^�[�Q�b�g�̊p���x)
    end

   if     POS_eR(1,1) < SV_ts.R0(1,1)
       corner_plate_yvel_R = -corner_vel_R * cos( psi_R );% * cos( abs( theta_1_yaw ) );   % �l�p�̒��_�̑��x(�ڐG�ʂɂ�����side_vel�̑��x)�̐ڐG�ʂɂ�����y���� (�p���x�����Ȃ�l�p�̍�����y�����͕�)
   elseif POS_eR(1,1) > SV_ts.R0(1,1)
       corner_plate_yvel_R =  corner_vel_R * cos( psi_R );% * cos( abs( theta_1_yaw ) );   % �l�p�̒��_�̑��x(�ڐG�ʂɂ�����side_vel�̑��x)�̐ڐG�ʂɂ�����y���� (�p���x�����Ȃ�l�p�̉E����y�����͐�)
   end

     if curPosAP3_vel_R(2,1) + corner_plate_yvel_R == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_R = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_R = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_R(2,1) + corner_plate_yvel_R ) )' * norm_R;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_R = 0;   % �ڐG�t���O = 0
        tang_R = zeros(3,1);
  end



%%%%%%%% ��悪�̈�12,23,34,41�ɂ���Ƃ�(�ʂŐڐG���Ă���Ƃ�) %%%%%%%%

elseif contact_flag_R(1,2) > 0  ||  contact_flag_R(2,3) > 0  ||  contact_flag_R(3,4) > 0  ||  contact_flag_R(4,1) > 0


if     contact_flag_R(1,2) > 0

   l_R = sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 );   % �^�[�Q�b�g���S����E��担���S�܂ł̋��� ��
    if     0 < q0  &&  q0 < pi/4   % ���̕�����e���ڐG�̈�ɂ���ĕς��܂�
        if     POS_eR(2,1) > ts_geo(2,1)
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) + q0 );   % �^�[�Q�b�g���S�ʒu�Ǝ�撆�S�ʒu�����ԎΕӂƁC�^�[�Q�b�g���S��ʂ�^�[�Q�b�g�l�p�`�̏㉺�ӂɕ��s�Ȓ���������ӂ�p�����C���p�O�p�`�̉s�p��
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
    else   % q0 = ��/4�̂Ƃ�
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - pi/4 );
    end
   l_x_R = l_R * cos( phi_R );   % �^�[�Q�b�g�d�S���W�ɂ�����E��担���S��x���W�̐�Βl

% �߂荞�ݗʐݒ�
   delta_R = abs( r_tip + half_side - l_x_R );   % ��
% �߂荞�ݑ��x�ݒ�
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_R = delta_R;   % �X�V
% �ڐG�ʊp�x
%    theta_R_yaw = q0 - pi;
   norm_R = [ -cos(q0), -sin(q0), 0 ]';
% �������W�n����݂��ڐG�ʒu
    PointC_R = POS_eR + (r_tip - delta_R) * ( -norm_R );
    curPosAP3_R = PointC_R;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_R = PointC_R;   % ���{�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_R = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_RR = TB0_R * curPosAP3_R;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_R = tilde( curPosAP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_R = curPosAP3_RR;   % �X�V
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_RR = DB0_R * curPosBP3_R;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_R = tilde( curPosBP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_R >= 0.0  ||  r_tip + half_side - l_x_R >= 0.0  ||  l_side_min >= l_x_R   % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % �ڐG�p�x�v�Z
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
    if F_NN_R < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
      contactflag_R = 0;
      F_NN_R = 0.0;   % �ڐG�� = 0
    end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_R = 0;
      F_NN_R = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_R >= 0.0  &&  r_tip + half_side - l_x_R >= 0.0  &&  l_side_min >= l_x_R
     if curPosAP3_vel_R(2,1) == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_R = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_R = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_R(2,1) ) )' * norm_R;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_R = 0;   % �ڐG�t���O = 0
        tang_R = zeros(3,1);
  end


elseif contact_flag_R(2,3) > 0

   l_R = sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 );   % �^�[�Q�b�g���S�����担���S�܂ł̋��� ��
    if     0 < q0  &&  q0 < pi/4   % ���̕�����e���ڐG�̈�ɂ���ĕς��܂�
        if     POS_eR(1,1) < ts_geo(1,1)
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) + q0 );   % �^�[�Q�b�g���S�ʒu�Ǝ�撆�S�ʒu�����ԎΕӂƁC�^�[�Q�b�g���S��ʂ�^�[�Q�b�g�l�p�`�̏㉺�ӂɕ��s�Ȓ���������ӂ�p�����C���p�O�p�`�̉s�p��
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
    else   % q0 = ��/4�̂Ƃ�
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - pi/4 );
    end
   l_x_R = l_R * cos( phi_R );   % �^�[�Q�b�g�d�S���W�ɂ����鍶��担���S��x���W�̐�Βl

% �߂荞�ݗʐݒ�
   delta_R = abs( r_tip + half_side - l_x_R );   % ��
% �߂荞�ݑ��x�ݒ�
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   %����   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_R = delta_R;   % �X�V
% �ڐG�ʊp�x
%    theta_R_yaw = q0 - pi/2;
   norm_R = [ sin(q0), -cos(q0), 0 ]';
% �������W�n����݂��ڐG�ʒu
    PointC_R = POS_eR + (r_tip - delta_R) * ( -norm_R );
    curPosAP3_R = PointC_R;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_R = PointC_R;   % ���{�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_R = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_RR = TB0_R * curPosAP3_R;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_R = tilde( curPosAP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_R = curPosAP3_RR;   % �X�V
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_RR = DB0_R * curPosBP3_R;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_R = tilde( curPosBP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_R >= 0.0  ||  r_tip + half_side - l_x_R >= 0.0  ||  l_side_min >= l_x_R   % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % �ڐG�p�x�v�Z
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
    if F_NN_R < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
      contactflag_R = 0;
      F_NN_R = 0.0;   % �ڐG�� = 0
    end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_R = 0;
      F_NN_R = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_R >= 0.0  &&  r_tip + half_side - l_x_R >= 0.0  &&  l_side_min >= l_x_R
     if curPosAP3_vel_R(1,1) == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_R = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_R = rpy2dc( 0, 0, -pi/2 * sign( curPosAP3_vel_R(1,1) ) )' * norm_R;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_R = 0;   % �ڐG�t���O = 0
        tang_R = zeros(3,1);
  end


elseif contact_flag_R(3,4) > 0

   l_R = sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 );   % �^�[�Q�b�g���S�����担���S�܂ł̋��� ��
    if     0 < q0  &&  q0 < pi/4   % ���̕�����e���ڐG�̈�ɂ���ĕς��܂�
        if     POS_eR(2,1) < ts_geo(2,1)
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) + q0 );   % �^�[�Q�b�g���S�ʒu�Ǝ�撆�S�ʒu�����ԎΕӂƁC�^�[�Q�b�g���S��ʂ�^�[�Q�b�g�l�p�`�̏㉺�ӂɕ��s�Ȓ���������ӂ�p�����C���p�O�p�`�̉s�p��
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
    else   % q0 = ��/4�̂Ƃ�
            phi_R = abs( acos( abs( POS_eR(1,1) - ts_geo(1,1) ) / l_R ) - pi/4 );
    end
   l_x_R = l_R * cos( phi_R );   % �^�[�Q�b�g�d�S���W�ɂ����鍶��担���S��x���W�̐�Βl

% �߂荞�ݗʐݒ�
   delta_R = abs( r_tip + half_side - l_x_R );   % ��
% �߂荞�ݑ��x�ݒ�
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   %����   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_R = delta_R;   % �X�V
% �ڐG�ʊp�x
%    theta_R_yaw = q0;
   norm_R = [ cos(q0), sin(q0), 0 ]';
% �������W�n����݂��ڐG�ʒu
    PointC_R = POS_eR + (r_tip - delta_R) * ( -norm_R );
    curPosAP3_R = PointC_R;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_R = PointC_R;   % ���{�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_R = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_RR = TB0_R * curPosAP3_R;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_R = tilde( curPosAP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_R = curPosAP3_RR;   % �X�V
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_RR = DB0_R * curPosBP3_R;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_R = tilde( curPosBP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_R >= 0.0  ||  r_tip + half_side - l_x_R >= 0.0  ||  l_side_min >= l_x_R   % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % �ڐG�p�x�v�Z
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
    if F_NN_R < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
      contactflag_R = 0;
      F_NN_R = 0.0;   % �ڐG�� = 0
    end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_R = 0;
      F_NN_R = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_R >= 0.0  &&  r_tip + half_side - l_x_R >= 0.0  &&  l_side_min >= l_x_R
     if curPosAP3_vel_R(2,1) == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_R = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_R = rpy2dc( 0, 0, -pi/2 * sign( curPosAP3_vel_R(2,1) ) )' * norm_R;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_R = 0;   % �ڐG�t���O = 0
        tang_R = zeros(3,1);
  end


elseif contact_flag_R(4,1) > 0

   l_R = sqrt( ( ts_geo(1,1) - POS_eR(1,1) )^2 + ( ts_geo(2,1) - POS_eR(2,1) )^2 );   % �^�[�Q�b�g���S�����担���S�܂ł̋��� ��
    if     0 < q0  &&  q0 < pi/4   % ���̕�����e���ڐG�̈�ɂ���ĕς��܂�
        if     POS_eR(1,1) > ts_geo(1,1)
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) + q0 );   % �^�[�Q�b�g���S�ʒu�Ǝ�撆�S�ʒu�����ԎΕӂƁC�^�[�Q�b�g���S��ʂ�^�[�Q�b�g�l�p�`�̏㉺�ӂɕ��s�Ȓ���������ӂ�p�����C���p�O�p�`�̉s�p��
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
    else   % q0 = ��/4�̂Ƃ�
            phi_R = abs( acos( abs( POS_eR(2,1) - ts_geo(2,1) ) / l_R ) - pi/4 );
    end
   l_x_R = l_R * cos( phi_R );   % �^�[�Q�b�g�d�S���W�ɂ����鍶��担���S��x���W�̐�Βl

% �߂荞�ݗʐݒ�
   delta_R = abs( r_tip + half_side - l_x_R );   % ��
% �߂荞�ݑ��x�ݒ�
   deltaVel_R = ( delta_R - delta_tmp_R ) / d_time;   %����   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_R = delta_R;   % �X�V
% �ڐG�ʊp�x
%    theta_R_yaw = q0 + pi/2;
   norm_R = [ -sin(q0), cos(q0), 0 ]';
% �������W�n����݂��ڐG�ʒu
    PointC_R = POS_eR + (r_tip - delta_R) * ( -norm_R );
    curPosBP3_R = PointC_R;   % ���{�b�g�p�ɕۑ�
    curPosAP3_R = PointC_R;   % �^�[�Q�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_R = curPosAP3_R - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_R = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_RR = TB0_R * curPosAP3_R;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_R = tilde( curPosAP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_R = curPosAP3_RR - curPosAP3_tmp_R;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_R = curPosAP3_RR;   % �X�V
    curPosAP3_vel_R = curPosAP3_delta_R / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_R = curPosBP3_R - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_R = curPosBP3_R - Rg_d;
    DB0_R = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_RR = DB0_R * curPosBP3_R;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_R = tilde( curPosBP3_RR );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_R >= 0.0  ||  r_tip + half_side - l_x_R >= 0.0  ||  l_side_min >= l_x_R   % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_R = 1;
%       contact_rad_R = abs( theta_R_yaw - ( Qe_radR(3,1) - pi/2 ) );   % �ڐG�p�x�v�Z
%       kw_R = k_p(1,1) * contact_rad_R^3 + k_p(1,2) * contact_rad_R^2 + k_p(1,3) * contact_rad_R + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
%       cw_R = ( c_p(1,1) * contact_rad_R^3 + c_p(1,2) * contact_rad_R^2 + c_p(1,3) * contact_rad_R + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
      kw_R = kkk;
      cw_R = ccc;
      F_NN_R = kw_R * delta_R + cw_R * deltaVel_R;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
    if F_NN_R < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
      contactflag_R = 0;
      F_NN_R = 0.0;   % �ڐG�� = 0
    end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_R = 0;
      F_NN_R = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_R >= 0.0  &&  r_tip + half_side - l_x_R >= 0.0  &&  l_side_min >= l_x_R
     if curPosAP3_vel_R(1,1) == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_R = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_R = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_R(1,1) ) )' * norm_R;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_R = 0;   % �ڐG�t���O = 0
        tang_R = zeros(3,1);
  end

end


else   % �ڐG���Ă��Ȃ��Ƃ�(�ꉞ)
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

FR_N_R = F_NN_R * norm_R;   % ���{�b�g�����󂯂�@����
FR_T_R = cof_R * F_NN_R * tang_R;   % ��悪�󂯂門�C��
FR_R = FR_N_R + FR_T_R;

end