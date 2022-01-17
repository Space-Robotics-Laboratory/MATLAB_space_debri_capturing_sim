function [ est_P, est_V, est_erP, NorTraj_12, est_Q, est_q, est_W, est_geo, est_half_side, est_r1, est_r2, est_phi1, est_phi2, est_R12, est_Theta12 ] ...
= Maru_Estimation_v01( d_time, time, Obs1, Obs2, Obs1_tmp, Obs2_tmp, NorTraj_12_tmp, est_P_tmp, est_Q_tmp, contactflag_L1, contactflag_L2, contactflag_R1, contactflag_R2, est_phi1, est_phi2, est_r1, est_r2, est_P, est_V, est_erP, NorTraj_12, est_Theta12 )

%%% �����Ȃ�
est_R12 = sqrt( ( Obs1(1,1) - Obs2(1,1) )^2 + ( Obs1(2,1) - Obs2(2,1) )^2 );   % ten1��ten2�̋���
est_half_side = est_R12 / 2;
est_geo = Obs1 + 1/sqrt(2) * rpy2dc( 0, 0, -pi/4 ) * ( Obs2 - Obs1 );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% �d�S�ʒu�̐��� %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if time >= d_time * 3

  if contactflag_L1 == 1 || contactflag_L2 == 1 || contactflag_R1 == 1 || contactflag_R2 == 1

  est_P = Obs1 + rpy2dc( 0, 0, -est_phi1 ) * ( Obs2 - Obs1 ) * est_r1 / est_R12;

  else     % ��ڐG�̏ꍇ�̂݁C�d�S�ʒu����
  %%% �d�S�ʒu�̐���

%%% �O�Ղ̖@���̌X���Ɛؕ�(ten��ten_tmp�̒��_��ʂ�@��)
an_Obs1_traj = -( Obs1(1,1) - Obs1_tmp(1,1) ) / ( Obs1(2,1) - Obs1_tmp(2,1) );
an_Obs2_traj = -( Obs2(1,1) - Obs2_tmp(1,1) ) / ( Obs2(2,1) - Obs2_tmp(2,1) );
bn_Obs1_traj = ( Obs1(2,1) + Obs1_tmp(2,1) ) / 2 - an_Obs1_traj * ( Obs1(1,1) + Obs1_tmp(1,1) ) / 2;
bn_Obs2_traj = ( Obs2(2,1) + Obs2_tmp(2,1) ) / 2 - an_Obs2_traj * ( Obs2(1,1) + Obs2_tmp(1,1) ) / 2;

%%% �@���̌�_�̍��W�����߂� ����1,2�̌�_
NorTraj_12(1,1) = -( bn_Obs2_traj - bn_Obs1_traj ) / ( an_Obs2_traj - an_Obs1_traj );
NorTraj_12(2,1) = ( an_Obs2_traj * bn_Obs1_traj - an_Obs1_traj * bn_Obs2_traj ) / ( an_Obs2_traj - an_Obs1_traj );
NorTraj_12(3,1) = 0;

%%% ���葬�x�C�덷
est_V_dash = ( NorTraj_12 - NorTraj_12_tmp )/d_time;

%%% ten_tmp�����s�ړ���ten_dash_est����]��ten
Obs1_dash_est = Obs1_tmp + est_V_dash * d_time;
Obs2_dash_est = Obs2_tmp + est_V_dash * d_time;


%%% ���̖@���̌X���Ɛؕ�(ten��ten_dash_est�̒��_��ʂ�@��)
a_Obs1_gen_normal = -( Obs1(1,1) - Obs1_dash_est(1,1) ) / ( Obs1(2,1) - Obs1_dash_est(2,1) );
a_Obs2_gen_normal = -( Obs2(1,1) - Obs2_dash_est(1,1) ) / ( Obs2(2,1) - Obs2_dash_est(2,1) );
b_Obs1_gen_normal = ( Obs1(2,1) + Obs1_dash_est(2,1) ) / 2 - a_Obs1_gen_normal * ( Obs1(1,1) + Obs1_dash_est(1,1) ) / 2;
b_Obs2_gen_normal = ( Obs2(2,1) + Obs2_dash_est(2,1) ) / 2 - a_Obs2_gen_normal * ( Obs2(1,1) + Obs2_dash_est(1,1) ) / 2;

%%% ���̖@���̌�_(�~�̐�����W�C�덷)
est_P(1,1) = -( b_Obs2_gen_normal - b_Obs1_gen_normal ) / ( a_Obs2_gen_normal - a_Obs1_gen_normal );
est_P(2,1) = ( a_Obs2_gen_normal * b_Obs1_gen_normal - a_Obs1_gen_normal * b_Obs2_gen_normal ) / ( a_Obs2_gen_normal - a_Obs1_gen_normal );
est_P(3,1) = 0;


%%% ���蔼�a�C����p���x�C����p�x�C���̌덷
est_r1 = sqrt( ( est_P(1,1) - Obs1(1,1) )^2 + ( est_P(2,1) - Obs1(2,1) )^2 );
est_r2 = sqrt( ( est_P(1,1) - Obs2(1,1) )^2 + ( est_P(2,1) - Obs2(2,1) )^2 );


%%% �]���藝���琄��d�S�C�_1,2�̎O�p�`���̊p�x�𐄒�
est_Theta12 = acos( ( est_r1^2 + est_r2^2 - est_R12^2 ) / ( 2 * est_r1 * est_r2 ) );
est_phi1 = acos( ( est_R12^2 + est_r1^2 - est_r2^2 ) / ( 2 * est_r1 * est_R12 ) );
est_phi2 = acos( ( est_R12^2 + est_r2^2 - est_r1^2 ) / ( 2 * est_r2 * est_R12 ) );

%%% �􉽒��S����̏d�S�̋���
est_erP = sqrt( ( est_P(1,1) - est_geo(1,1) )^2 + ( est_P(2,1) - est_geo(2,1) )^2 );

  end
  
end


r = sqrt( ( est_P(1,1) - est_geo(1,1) )^2 + ( est_P(2,1) - est_geo(2,1) )^2 );
if  Obs1_tmp(1,1) == Obs1(1,1) || Obs2_tmp(1,1) == Obs2(1,1) || Obs1_tmp(2,1) == Obs1(2,1) || Obs2_tmp(2,1) == Obs2(2,1) || ( est_half_side * sqrt(2) ) < r
  est_P = est_geo;
end


%%% ���葬�x�C�덷
est_V = ( est_P - est_P_tmp )/d_time;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% �p�x�̐��� %%%%%%%%%%%%%%%%%%%%   % ���atan2()��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% est_Q ��(3,1)�̊p�x�x�N�g�� (�l��-��<Q<=��)
%%% �^�[�Q�b�g�̐���p�x   Obs1, Obs2�������`����C�����̓_�̏ꍇ
% if     Obs1(1,1) < Obs2(1,1)   % Obs1�̕�����
%    if     Obs1(2,1) > Obs2(2,1)   % Obs1�̕�����
%           Q = atan( abs( Obs2(1,1) - Obs1(1,1) ) / abs( Obs2(2,1) - Obs1(2,1) ) );
%    elseif Obs1(2,1) < Obs2(2,1)   % Obs1�̕�����
%           Q = atan( abs( Obs2(1,1) - Obs1(1,1) ) / abs( Obs2(2,1) - Obs1(2,1) ) ) + pi/2;
%    else   % y���W������
%           Q = pi/2;
%    end
% elseif Obs1(1,1) > Obs2(1,1)   % Obs1�̕����E
%    if     Obs1(2,1) > Obs2(2,1)   % Obs1�̕�����
%           Q = -atan( abs( Obs2(1,1) - Obs1(1,1) ) / abs( Obs2(2,1) - Obs1(2,1) ) );
%    elseif Obs1(2,1) < Obs2(2,1)   % Obs1�̕�����
%           Q = -atan( abs( Obs2(1,1) - Obs1(1,1) ) / abs( Obs2(2,1) - Obs1(2,1) ) ) - pi/2;
%    else   % y���W������
%           Q = -pi/2;
%    end
% else   % x���W������
%    if     Obs1(2,1) > Obs2(2,1)   % Obs1�̕�����
%           Q = 0;
%    elseif Obs1(2,1) < Obs2(2,1)   % Obs1�̕�����
%           Q = pi;
%    else   % 2�ϑ��_�_�u���Ă�
%    %%%%%%%%%%%%%%%%%%%%%%%%
%    end
% end
Q = atan2( ( Obs2(2,1) - Obs1(2,1) ), ( Obs2(1,1) - Obs1(1,1) ) );

est_Q(3,1) = Q;


%%% est_q0 �͊p�x�̃X�J���[ (�l��0<=q0<��/2)

if      pi/2 <= Q && Q < pi    % �^�[�Q�b�g����/2�`�΂ɌX���Ƃ�
    est_q = Q - pi/2;
elseif -pi/2 <= Q && Q < 0     % �^�[�Q�b�g��-��/2�`0�ɌX���Ƃ�
    est_q = Q + pi/2;
elseif -pi <= Q && Q < -pi/2   % �^�[�Q�b�g��-�΁`-��/2�ɌX���Ƃ�
    est_q = Q + pi;
else                           % �^�[�Q�b�g��0�`��/2�̂Ƃ��͂��̂܂ܑ��
    est_q = Q;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% �p���x�̐��� %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
est_W = 1/d_time * ( est_Q - est_Q_tmp );



end