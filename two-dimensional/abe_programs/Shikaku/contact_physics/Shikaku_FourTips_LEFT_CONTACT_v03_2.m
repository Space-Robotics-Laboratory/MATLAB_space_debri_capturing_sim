function [ contactflag_L, F_NN_L, FR_N_L, FR_T_L, norm_L, tang_L, FR_L, delta_L, deltaVel_L, delta_tmp_L, TB0_L, DB0_L, curPosAP33_L, curPosAP3_delta_L, curPosAP3_tmp_L, curPosBP33_L, PointC_L, l_x_L ] ...
= Shikaku_FourTips_LEFT_CONTACT_v03( LP_d, SV_d, SV_ts, d_time, half_side, r_tip, contact_flag_L, contactflag_L, shikaku1, shikaku2, shikaku3, shikaku4, ...
                        l_side_min, l_corner_min, ts_geo, q0, Rg_d, POS_eL, delta_tmp_L, curPosAP3_tmp_L, l1, l2, l3, l4, gamma1, gamma2, gamma3, gamma4, cof_L, kkk, ccc, jointsL, k_p, c_p )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ���� %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% ��悪�̈�11,22,33,44�ɂ���Ƃ�(�p�ŐڐG���Ă���Ƃ�) %%%%%%%%   % ��担�ƒ��_�Ƃ̐ڐG���C���z�I�ɉ~�Ƃ̐ڐG�Ƃ��čl����

if     contact_flag_L(1,1) > 0  ||  contact_flag_L(2,2) > 0  ||  contact_flag_L(3,3) > 0  ||  contact_flag_L(4,4) > 0

    if     contact_flag_L(1,1) > 0
        shikaku_L = shikaku1;   % �p�̃x�N�g����shikaku_L�Ƃ���   % �����Ɉ�ԋ߂����_
        gamma_L = gamma1;
        l_tsR0 = l1;
    elseif contact_flag_L(2,2) > 0
        shikaku_L = shikaku2;   % �p�̃x�N�g����shikaku_L�Ƃ���
        gamma_L = gamma2;
        l_tsR0 = l2;
    elseif contact_flag_L(3,3) > 0
        shikaku_L = shikaku3;   % �p�̃x�N�g����shikaku_L�Ƃ���
        gamma_L = gamma3;
        l_tsR0 = l3;
    elseif contact_flag_L(4,4) > 0
        shikaku_L = shikaku4;   % �p�̃x�N�g����shikaku_L�Ƃ���
        gamma_L = gamma4;
        l_tsR0 = l4;
    end

   r_L = sqrt( ( shikaku_L(1,1) - POS_eL(1,1) )^2 + ( shikaku_L(2,1) - POS_eL(2,1) )^2 );   % �^�[�Q�b�g���S�����担���S�܂ł̋���
   l_x_L =  0.0;
% �߂荞�ݗʐݒ�
   delta_L = abs( r_tip - r_L );   % ���l
% �߂荞�ݑ��x�ݒ�
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_L = delta_L;   % �X�V
% �ڐG�ʊp�x
    if shikaku_L(1,1) == POS_eL(1,1)   % ��担���S�ƃ^�[�Q�b�g���S������x���W�̂Ƃ�    
      if shikaku_L(2,1) > POS_eL(2,1)   % ��担�̐^��Ƀ^�[�Q�b�g������ꍇ
          theta_L_yaw = pi/2;   % ���ɍ����C�E�Ƀ^�[�Q�b�g�������Ԃ��p�x0�Ƃ��āC�ڐG�ʂ̓�/2
          norm_L = [ cos(-pi/2) sin(-pi/2) 0 ]';   %�@���P�ʃx�N�g��(�^�[�Q�b�g�O�������C��悪�󂯂�@���͂̌���) [0 -1 0]
      else                                 % ��担�̐^���Ƀ^�[�Q�b�g������ꍇ
          theta_L_yaw = -pi/2;   % ���ɍ����C�E�Ƀ^�[�Q�b�g�������Ԃ��p�x0�Ƃ��āC�ڐG�ʂ�-��/2
          norm_L = [ cos(pi/2) sin(pi/2) 0 ]';   % [0 1 0]
      end
    else   % ��担���S�ƃ^�[�Q�b�g���S���قȂ�x���W�Ƃ�
          theta_L_yaw = atan( ( shikaku_L(2,1) - POS_eL(2,1) ) / ( shikaku_L(1,1) - POS_eL(1,1) ) );   % �t���ځ��p�x���o  ��1'
          norm_L = [ cos(theta_L_yaw-pi) sin(theta_L_yaw-pi) 0 ]';
    end

% �������W�n����݂��ڐG�ʒu
    PointC_L = shikaku_L;
    curPosAP3_L = PointC_L;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_L = PointC_L;   % ���{�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_L = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_LL = TB0_L * curPosAP3_L;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_L = tilde( curPosAP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_L = curPosAP3_LL;   % �X�V
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_LL = DB0_L * curPosBP3_L;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_L = tilde( curPosBP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_L >= 0.0  ||  r_tip - r_L >= 0.0  ||  l_corner_min >= ( sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 ) - r_tip )  % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % ����
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % �ڐG�p�x�v�Z
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
      %kw_L = kkk;
      %cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
       if F_NN_L < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
          contactflag_L = 0;
          F_NN_L = 0.0;   % �ڐG�� = 0
       end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_L = 0;
      F_NN_L = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_L >= 0.0  ||  r_tip - r_L >= 0.0  ||  l_corner_min >= ( sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 ) - r_tip )

    if     contact_flag_L(1,1) > 0
        psi_L = abs( pi/2 - abs( q0 + gamma_L - theta_L_yaw ) );
        corner_vel_L = l_tsR0 * SV_ts.w0(3,1);   % �l�p�̒��_�̑��x(�^�[�Q�b�g�d�S���璸�_�܂ł̋����~�^�[�Q�b�g�̊p���x)
    elseif contact_flag_L(2,2) > 0
        psi_L = abs( q0 + gamma_L - theta_L_yaw );
        corner_vel_L = l_tsR0 * SV_ts.w0(3,1);   % �l�p�̒��_�̑��x(�^�[�Q�b�g�d�S���璸�_�܂ł̋����~�^�[�Q�b�g�̊p���x)
    elseif contact_flag_L(3,3) > 0
        psi_L = abs( pi/2 - abs( q0 + gamma_L - theta_L_yaw ) );
        corner_vel_L = l_tsR0 * SV_ts.w0(3,1);   % �l�p�̒��_�̑��x(�^�[�Q�b�g�d�S���璸�_�܂ł̋����~�^�[�Q�b�g�̊p���x)
    elseif contact_flag_L(4,4) > 0
        psi_L = abs( q0 + gamma_L - theta_L_yaw );
        corner_vel_L = l_tsR0 * SV_ts.w0(3,1);   % �l�p�̒��_�̑��x(�^�[�Q�b�g�d�S���璸�_�܂ł̋����~�^�[�Q�b�g�̊p���x)
    end

   if     POS_eL(1,1) < SV_ts.R0(1,1)
       corner_plate_yvel_L = -corner_vel_L * cos( psi_L );% * cos( abs( theta_1_yaw ) );   % �l�p�̒��_�̑��x(�ڐG�ʂɂ�����side_vel�̑��x)�̐ڐG�ʂɂ�����y���� (�p���x�����Ȃ�l�p�̍�����y�����͕�)
   elseif POS_eL(1,1) > SV_ts.R0(1,1)
       corner_plate_yvel_L =  corner_vel_L * cos( psi_L );% * cos( abs( theta_1_yaw ) );   % �l�p�̒��_�̑��x(�ڐG�ʂɂ�����side_vel�̑��x)�̐ڐG�ʂɂ�����y���� (�p���x�����Ȃ�l�p�̉E����y�����͐�)
   end

     if curPosAP3_vel_L(2,1) + corner_plate_yvel_L == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_L = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_L = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_L(2,1) + corner_plate_yvel_L ) )' * norm_L;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_L = 0;   % �ڐG�t���O = 0
        tang_L = zeros(3,1);
  end



%%%%%%%% ��悪�̈�12,23,34,41�ɂ���Ƃ�(�ʂŐڐG���Ă���Ƃ�) %%%%%%%%

elseif contact_flag_L(1,2) > 0  ||  contact_flag_L(2,3) > 0  ||  contact_flag_L(3,4) > 0  ||  contact_flag_L(4,1) > 0


if     contact_flag_L(1,2) > 0

   l_L = sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 );   % �^�[�Q�b�g���S���獶��担���S�܂ł̋��� ��
    if     0 < q0  &&  q0 < pi/4   % ���̕�����e���ڐG�̈�ɂ���ĕς��܂�
        if     POS_eL(2,1) > ts_geo(2,1)
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) + q0 );   % �^�[�Q�b�g���S�ʒu�Ǝ�撆�S�ʒu�����ԎΕӂƁC�^�[�Q�b�g���S��ʂ�^�[�Q�b�g�l�p�`�̏㉺�ӂɕ��s�Ȓ���������ӂ�p�����C���p�O�p�`�̉s�p��
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
    else   % q0 = ��/4�̂Ƃ�
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - pi/4 );
    end
   l_x_L = l_L * cos( phi_L );   % �^�[�Q�b�g�d�S���W�ɂ����鍶��担���S��x���W�̐�Βl

% �߂荞�ݗʐݒ�
   delta_L = abs( r_tip + half_side - l_x_L );   % ��
% �߂荞�ݑ��x�ݒ�
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_L = delta_L;   % �X�V
% �ڐG�ʊp�x
    theta_L_yaw = q0;
   norm_L = [ -cos(q0), -sin(q0), 0 ]';
% �������W�n����݂��ڐG�ʒu
    PointC_L = POS_eL + (r_tip - delta_L) * ( -norm_L );
    curPosAP3_L = PointC_L;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_L = PointC_L;   % ���{�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_L = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_LL = TB0_L * curPosAP3_L;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_L = tilde( curPosAP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_L = curPosAP3_LL;   % �X�V
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_LL = DB0_L * curPosBP3_L;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_L = tilde( curPosBP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_L >= 0.0  ||  r_tip + half_side - l_x_L >= 0.0  ||  l_side_min >= l_x_L   % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % ����
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % �ڐG�p�x�v�Z
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
%      kw_L = kkk;
%      cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
    if F_NN_L < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
      contactflag_L = 0;
      F_NN_L = 0.0;   % �ڐG�� = 0
    end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_L = 0;
      F_NN_L = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_L >= 0.0  &&  r_tip + half_side - l_x_L >= 0.0  &&  l_side_min >= l_x_L
     if curPosAP3_vel_L(2,1) == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_L = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_L = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_L(2,1) ) )' * norm_L;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_L = 0;   % �ڐG�t���O = 0
        tang_L = zeros(3,1);
  end


elseif contact_flag_L(2,3) > 0

   l_L = sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 );   % �^�[�Q�b�g���S�����担���S�܂ł̋��� ��
    if     0 < q0  &&  q0 < pi/4   % ���̕�����e���ڐG�̈�ɂ���ĕς��܂�
        if     POS_eL(1,1) < ts_geo(1,1)
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) + q0 );   % �^�[�Q�b�g���S�ʒu�Ǝ�撆�S�ʒu�����ԎΕӂƁC�^�[�Q�b�g���S��ʂ�^�[�Q�b�g�l�p�`�̏㉺�ӂɕ��s�Ȓ���������ӂ�p�����C���p�O�p�`�̉s�p��
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
    else   % q0 = ��/4�̂Ƃ�
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - pi/4 );
    end
   l_x_L = l_L * cos( phi_L );   % �^�[�Q�b�g�d�S���W�ɂ����鍶��担���S��x���W�̐�Βl

% �߂荞�ݗʐݒ�
   delta_L = abs( r_tip + half_side - l_x_L );   % ��
% �߂荞�ݑ��x�ݒ�
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   %����   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_L = delta_L;   % �X�V
% �ڐG�ʊp�x
    theta_L_yaw = q0 + pi/2;
   norm_L = [ sin(q0), -cos(q0), 0 ]';
% �������W�n����݂��ڐG�ʒu
    PointC_L = POS_eL + (r_tip - delta_L) * ( -norm_L );
    curPosAP3_L = PointC_L;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_L = PointC_L;   % ���{�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_L = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_LL = TB0_L * curPosAP3_L;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_L = tilde( curPosAP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_L = curPosAP3_LL;   % �X�V
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_LL = DB0_L * curPosBP3_L;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_L = tilde( curPosBP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_L >= 0.0  ||  r_tip + half_side - l_x_L >= 0.0  ||  l_side_min >= l_x_L   % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % ����
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % �ڐG�p�x�v�Z
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
%      kw_L = kkk;
%      cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
    if F_NN_L < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
      contactflag_L = 0;
      F_NN_L = 0.0;   % �ڐG�� = 0
    end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_L = 0;
      F_NN_L = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_L >= 0.0  &&  r_tip + half_side - l_x_L >= 0.0  &&  l_side_min >= l_x_L
     if curPosAP3_vel_L(1,1) == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_L = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_L = rpy2dc( 0, 0, -pi/2 * sign( curPosAP3_vel_L(1,1) ) )' * norm_L;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_L = 0;   % �ڐG�t���O = 0
        tang_L = zeros(3,1);
  end


elseif contact_flag_L(3,4) > 0

   l_L = sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 );   % �^�[�Q�b�g���S�����担���S�܂ł̋��� ��
    if     0 < q0  &&  q0 < pi/4   % ���̕�����e���ڐG�̈�ɂ���ĕς��܂�
        if     POS_eL(2,1) < ts_geo(2,1)
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) + q0 );   % �^�[�Q�b�g���S�ʒu�Ǝ�撆�S�ʒu�����ԎΕӂƁC�^�[�Q�b�g���S��ʂ�^�[�Q�b�g�l�p�`�̏㉺�ӂɕ��s�Ȓ���������ӂ�p�����C���p�O�p�`�̉s�p��
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
    else   % q0 = ��/4�̂Ƃ�
            phi_L = abs( acos( abs( POS_eL(1,1) - ts_geo(1,1) ) / l_L ) - pi/4 );
    end
   l_x_L = l_L * cos( phi_L );   % �^�[�Q�b�g�d�S���W�ɂ����鍶��担���S��x���W�̐�Βl

% �߂荞�ݗʐݒ�
   delta_L = abs( r_tip + half_side - l_x_L );   % ��
% �߂荞�ݑ��x�ݒ�
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   %����   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_L = delta_L;   % �X�V
% �ڐG�ʊp�x
    theta_L_yaw = q0 - pi;
   norm_L = [ cos(q0), sin(q0), 0 ]';
% �������W�n����݂��ڐG�ʒu
    PointC_L = POS_eL + (r_tip - delta_L) * ( -norm_L );
    curPosAP3_L = PointC_L;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_L = PointC_L;   % ���{�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_L = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_LL = TB0_L * curPosAP3_L;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_L = tilde( curPosAP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_L = curPosAP3_LL;   % �X�V
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_LL = DB0_L * curPosBP3_L;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_L = tilde( curPosBP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_L >= 0.0  ||  r_tip + half_side - l_x_L >= 0.0  ||  l_side_min >= l_x_L   % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % ����
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % �ڐG�p�x�v�Z
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
%      kw_L = kkk;
%      cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
    if F_NN_L < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
      contactflag_L = 0;
      F_NN_L = 0.0;   % �ڐG�� = 0
    end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_L = 0;
      F_NN_L = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_L >= 0.0  &&  r_tip + half_side - l_x_L >= 0.0  &&  l_side_min >= l_x_L
     if curPosAP3_vel_L(2,1) == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_L = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_L = rpy2dc( 0, 0, -pi/2 * sign( curPosAP3_vel_L(2,1) ) )' * norm_L;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_L = 0;   % �ڐG�t���O = 0
        tang_L = zeros(3,1);
  end


elseif contact_flag_L(4,1) > 0

   l_L = sqrt( ( ts_geo(1,1) - POS_eL(1,1) )^2 + ( ts_geo(2,1) - POS_eL(2,1) )^2 );   % �^�[�Q�b�g���S�����担���S�܂ł̋��� ��
    if     0 < q0  &&  q0 < pi/4   % ���̕�����e���ڐG�̈�ɂ���ĕς��܂�
        if     POS_eL(1,1) > ts_geo(1,1)
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) + q0 );   % �^�[�Q�b�g���S�ʒu�Ǝ�撆�S�ʒu�����ԎΕӂƁC�^�[�Q�b�g���S��ʂ�^�[�Q�b�g�l�p�`�̏㉺�ӂɕ��s�Ȓ���������ӂ�p�����C���p�O�p�`�̉s�p��
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
    else   % q0 = ��/4�̂Ƃ�
            phi_L = abs( acos( abs( POS_eL(2,1) - ts_geo(2,1) ) / l_L ) - pi/4 );
    end
   l_x_L = l_L * cos( phi_L );   % �^�[�Q�b�g�d�S���W�ɂ����鍶��担���S��x���W�̐�Βl

% �߂荞�ݗʐݒ�
   delta_L = abs( r_tip + half_side - l_x_L );   % ��
% �߂荞�ݑ��x�ݒ�
   deltaVel_L = ( delta_L - delta_tmp_L ) / d_time;   %����   % �߂荞�ݑ��x = ( ���݂̂߂荞�ݗ� - 1�X�e�b�v�O�̂߂荞�ݗ� )/��������
   delta_tmp_L = delta_L;   % �X�V
% �ڐG�ʊp�x
    theta_L_yaw = q0 - pi/2;
   norm_L = [ -sin(q0), cos(q0), 0 ]';
% �������W�n����݂��ڐG�ʒu
    PointC_L = POS_eL + (r_tip - delta_L) * ( -norm_L );
    curPosAP3_L = PointC_L;   % �^�[�Q�b�g�p�ɕۑ�
    curPosBP3_L = PointC_L;   % ���{�b�g�p�ɕۑ�
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu
    curPosAP3_L = curPosAP3_L - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
    TB0_L = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������
    curPosAP3_LL = TB0_L * curPosAP3_L;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33_L = tilde( curPosAP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �^�[�Q�b�g�d�S���W�n���猩���ڐG�ʒu�̑��x
    curPosAP3_delta_L = curPosAP3_LL - curPosAP3_tmp_L;   % �ڐG�ʒu�̍��� = ���݂̐ڐG�ʒu - 1�X�e�b�v�O�̐ڐG�ʒu
    curPosAP3_tmp_L = curPosAP3_LL;   % �X�V
    curPosAP3_vel_L = curPosAP3_delta_L / d_time;   % ����(���x)
% ���{�b�g�S�̂̏d�S���W���猩���ʒu
%     curPosBP3_L = curPosBP3_L - SV_d.R0;   % BP���g���Ċ������W���S�����{�b�g�S�̏d�S���W���S�ɑւ���
    curPosBP3_L = curPosBP3_L - Rg_d;
    DB0_L = rpy2dc( SV_d.Q0 );   % ���{�b�g��]�p�̕����]���s������
    curPosBP3_LL = DB0_L * curPosBP3_L;   % �ڐG�ʒu�����{�b�g�d�S���W�ɍ��킹��]
    curPosBP33_L = tilde( curPosBP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
% �ڐG�͂̌v�Z
  if delta_L >= 0.0  ||  r_tip + half_side - l_x_L >= 0.0  ||  l_side_min >= l_x_L   % �߂荞�ݗʂ����̂Ƃ�(�ڐG�����u�Ԃ���߂荞��ł���Ƃ�)
      contactflag_L = 1;
      [ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % ����
      Qe_radL = dc2rpy( ORI_eL' );
       contact_rad_L = abs( theta_L_yaw - ( pi/2 + Qe_radL(3,1) ) );   % �ڐG�p�x�v�Z
       kw_L = k_p(1,1) * contact_rad_L^3 + k_p(1,2) * contact_rad_L^2 + k_p(1,3) * contact_rad_L + k_p(1,4);   % �Ƃ̑�������p���čœK�� k(�����W��) �𓱏o
       cw_L = ( c_p(1,1) * contact_rad_L^3 + c_p(1,2) * contact_rad_L^2 + c_p(1,3) * contact_rad_L + c_p(1,4) );   % cw_1 = 0.0;   % �S�������W�� = 0
%      kw_L = kkk;
%      cw_L = ccc;
      F_NN_L = kw_L * delta_L + cw_L * deltaVel_L;   % �ڐG�ʂɑ΂���@�������̗͂��o�l�E�_���p���f���œ��o ���l? 
    if F_NN_L < 0.0   % �@�������̗͂����̏ꍇ(�z���͔͂������Ă��Ȃ��̂�)
      contactflag_L = 0;
      F_NN_L = 0.0;   % �ڐG�� = 0
    end
  else   % �߂荞�ݗʂ����̂Ƃ�
      contactflag_L = 0;
      F_NN_L = 0.0;   % �ڐG�͔͂������Ȃ� �ڐG�� = 0
  end
% ���C�͂̌v�Z
  if delta_L >= 0.0  &&  r_tip + half_side - l_x_L >= 0.0  &&  l_side_min >= l_x_L
     if curPosAP3_vel_L(1,1) == 0.0   % �ڐG��y�����̑��x(target)�� 0 �̂Ƃ� 
        tang_L = zeros(3,1);
     else   % �ڐG��y�����̑��x(target)�� 0 �ł͂Ȃ��Ƃ� �ڐG��y�����̑��x�̕����ɂ���Ė��C�͂̕������ς���
        tang_L = rpy2dc( 0, 0, pi/2 * sign( curPosAP3_vel_L(1,1) ) )' * norm_L;
     end
  else   % �ڐG���Ă��Ȃ��Ƃ�
        contactflag_L = 0;   % �ڐG�t���O = 0
        tang_L = zeros(3,1);
  end

end


else   % �ڐG���Ă��Ȃ��Ƃ�(�ꉞ)
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

FR_N_L = F_NN_L * norm_L;   % ���{�b�g�����󂯂�@����
FR_T_L = cof_L * F_NN_L * tang_L;   % ��悪�󂯂門�C��
FR_L = FR_N_L + FR_T_L;

end