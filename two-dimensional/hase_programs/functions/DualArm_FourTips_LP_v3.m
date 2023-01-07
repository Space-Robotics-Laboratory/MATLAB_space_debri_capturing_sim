function LP_d = DualArm_FourTips_LP_v3()   % 2018/8/15 2018�N�V�e�X�g�x�b�h�p�ɏ������� ���J

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% �e�����N�p�����[�^�̒�` %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% �����N�̘A���֌W���`���� %%%%%
% LP�͊e�����N�̏����������L�q���Ă���  LP���f�[�^�œn���Ƃ��ɗl�X�ȃf�[�^��n����悤�ɍ\���̂ɂ��Ă���  BB(i) = �������Ō������Ă��郊���N�ԍ�
LP_d.BB = [ 0 1 2 3    0 5 6 7 ];   % �x�[�X(�����N0)���烊���N1�������Ă���  ���̂Ƃ������N1���猩�����{���Ƃ̓x�[�X�ւ̌����̂��Ƃł���̂�0����ԍ��ɑ������Ă���

% �����N��
LP_d.num_q = length( LP_d.BB );   % length�͍s���Ɨ񐔂̓��̑傫������^����

%%%%% �����N�̘A���֌W���`���� ���̂Q %%%%%
% �Ίp�����͂��ׂ�-1  �e�񂪍����炻�ꂼ�ꃊ���N1,2��\���Ă���  �������Ō������Ă��郊���N�ԍ��ɑΉ�����s��1������
LP_d.SS = [ -1  1  0  0     0  0  0  0;
           0 -1  1  0     0  0  0  0;
           0  0 -1  1     0  0  0  0;
           0  0  0 -1     0  0  0  0;
           0  0  0  0    -1  1  0  0;
           0  0  0  0     0 -1  1  0;
           0  0  0  0     0  0 -1  1;
           0  0  0  0     0  0  0 -1 ];

%%%%% �����N�̘A���֌W���`���� ���̂R %%%%%
% �x�[�X��(�����N0)�ɘA�����Ă��郊���N��1  ����ȊO��0
LP_d.S0 = [ 1 0 0 0    1 0 0 0 ];

%%%%% ��� (�����N��[) �̒�` %%%%%
% ���[�����N�Ȃ��1
LP_d.SE = [ 0 0 0 1    0 0 0 1 ];

%%%%% �֐߂̎�� %%%%%
% ��]�֐߂Ȃ�� 'R'  �����֐߂Ȃ�� 'P'
LP_d.J_type = [ 'R' 'R' 'R' 'R'    'R' 'R' 'R' 'R' ];

%%%%% �x�[�X�d�S����֐�i�ւ̈ʒu�x�N�g�� %%%%%
% ���ژA�����Ă��Ȃ������N�͂��ׂ�0 �x�[�X�̍��W�n����݂�����
bl = 0.00;
LP_d.c0 = [ -0.16+bl 0 0 0    0.16+bl 0 0 0;
           0.10946 0 0 0    0.10946 0 0 0;
           0       0 0 0    0       0 0 0 ];

%%%%% �x�[�X�� (�����N0) �̎��� %%%%%
LP_d.m0 = 7.695;   % = 5.145+2.550;

%%%%% �x�[�X�� (�����N0) �̊������[�����g %%%%% 
LP_d.inertia0 = [ 1e9 0   0;   % 0�̕������J�b�v�����O��(�������?)
                0   1e9 0;   % ������ς�0�Ƃ������Ƃ́A�Œ莲�̎������]����Ƃ�������
                0   0   0.09783069148];
            
nm = 3 / 2;
%%%%% �e�����N�i�x�[�X�ȊO�j�̎��� %%%%%
la_m = 0.570; lb_m = 0.560 * nm / 2; lc_m = 0.560 * nm / 2;
LP_d.m = [ la_m la_m lb_m lc_m    la_m la_m lb_m lc_m ];

%%%%% �����N�n�S�̂̎��� %%%%%
LP_d.mass = sum( LP_d.m ) + LP_d.m0;

%%%%% �e�����N�Ԃ̍��W�n�̉�]�֌W %%%%%
% �����̃����N���W�n����]������  ��̓����N �s��x,y,z����\��  �Ⴆ�Ή��L�̈Ӗ��̓����N1�̓x�[�X�����N(�����N0)�Ɠ���
% tipsangle1 =  pi/6; tipsangle2 = -pi/6; tipsangle3 =  pi/6; tipsangle4 = -pi/6;
LP_d.Qi = [ 0 0 0 0  0 0 0 0;
          0 0 0 0  0 0 0 0;
          0 0 0 0  0 0 0 0 ];
    
%%%%% �e�����N�̊������[�����g %%%%%
la_I = 0.00017771016; lb_I = 0.00067610325 * nm / 2; lc_I = 0.00067610325 * nm / 2;
LP_d.inertia = [1e9 0   0     1e9 0   0     1e9 0   0     1e9 0   0         1e9 0   0     1e9 0   0     1e9 0   0     1e9 0   0;
              0   1e9 0     0   1e9 0     0   1e9 0     0   1e9 0         0   1e9 0     0   1e9 0     0   1e9 0     0   1e9 0;  
              0   0   la_I  0   0   la_I  0   0   lb_I  0   0   lc_I      0   0   la_I  0   0   la_I  0   0   lb_I  0   0   lc_I ];

%%%%% �e�����N�d�S����֐߂ւ̈ʒu�x�N�g�� %%%%%
% ������
n = 8;      % n�̓����N��
LP_d.cc = zeros( 3, n, n );   % 3�~6�~6�̗�e���\��(3�~6�̗�s���6���)

% ��`
t1 = 0.1; 
t2 = 0.03;
t3 = 0.0; 
t4 = -0.01;
te1 = 0.03;

% y�v�f�ȊO�S���[��
LP_d.cc(:,1,1) = [ 0 -0.14641-t1 0 ]';   % �����N1�̏d�S���獪�{���̊֐�(J1)�ւ̈ʒu�x�N�g��
LP_d.cc(:,1,2) = [ 0  0.00459 0 ]';   % �����N1�̏d�S���烊���N2�̊֐߂ւ̈ʒu�x�N�g��
LP_d.cc(:,2,2) = [ 0 -0.14641-t2 0 ]';   % �����N2�̏d�S���烊���N2�̊֐߂ւ̈ʒu�x�N�g��
LP_d.cc(:,2,3) = [ 0  0.00459 0 ]';   % �����N2�̏d�S���烊���N3�̊֐߂ւ̈ʒu�x�N�g��
LP_d.cc(:,3,3) = [ 0 -0.03+t3 0 ]';      % �����N3�̏d�S���烊���N3�̊֐߂ւ̈ʒu�x�N�g��
LP_d.cc(:,3,4) = [ 0  0.03+t4 0 ]';   % �����N3�̏d�S���烊���N4�̊֐߂ւ̈ʒu�x�N�g��
LP_d.cc(:,4,4) = [ 0 -te1 0 ]';       % �����N4�̏d�S���烊���N4�̊֐߂ւ̈ʒu�x�N�g��

LP_d.cc(:,5,5) = [ 0 -0.14641-t1 0 ]';
LP_d.cc(:,5,6) = [ 0  0.00459 0 ]';
LP_d.cc(:,6,6) = [ 0 -0.14641-t2 0 ]';
LP_d.cc(:,6,7) = [ 0  0.00459 0 ]';
LP_d.cc(:,7,7) = [ 0 -0.03+t3 0 ]';
LP_d.cc(:,7,8) = [ 0  0.03+t4 0 ]';
LP_d.cc(:,8,8) = [ 0 -te1 0 ]';

% ���[�����N����[�_�܂ł̈ʒu�x�N�g��  ���[�����N�̏d�S������ւ̈ʒu�x�N�g��(��悪���̏ꍇ�͋��̒��S�܂�)  ���[���������Ȃ������N��0������
% L = 0.07;
% theta = pi/6;
% te2 = L*sin(theta); te3 = L*cos(theta) - te1;

% ��`
te2 = 0;
te3 = 0;

LP_d.ce = [ 0 0 0 -te2    0 0 0 -te2;   % ��惊���N�d�S�ʒu=���ʒu
          0 0 0  te3    0 0 0  te3;     % ���ǃ[���Ȃ񂩂�
          0 0 0  0      0 0 0  0 ];

% ���[�����N�ƒ[�_�̉�]�֌W  ���[�����N�̍��{�Ɛ�[�̍��W�n������ł���Ƃ�������  �L�q�@��Qi�Ɠ���  ���[���������Ȃ������N��0������
LP_d.Qe = [ 0 0 0 0    0 0 0 0;
          0 0 0 0    0 0 0 0;
          0 0 0 0    0 0 0 0 ];

% EOF