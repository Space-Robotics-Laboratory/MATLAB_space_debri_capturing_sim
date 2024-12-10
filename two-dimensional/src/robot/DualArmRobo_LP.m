function LP_d = DualArmRobo_LP(param)   %update uchida

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% �e�����N�p�����[�^�̒�` %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%��{�I��ParamSetting.m���Őݒ肷��%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% �����N�̘A���֌W���`���� %%%%%
% LP�͊e�����N�̏����������L�q���Ă���  LP���f�[�^�œn���Ƃ��ɗl�X�ȃf�[�^��n����悤�ɍ\���̂ɂ��Ă���  BB(i) = �������Ō������Ă��郊���N�ԍ�
LP_d.BB = [ 0 1 2 3    0 5 6 7 ];   % �x�[�X(�����N0)���烊���N1�������Ă���  ���̂Ƃ������N1���猩�����{���Ƃ̓x�[�X�ւ̌����̂��Ƃł���̂�0����ԍ��ɑ������Ă���

% �����N��
%�x�[�X���܂܂Ȃ�
LP_d.num_q = length( LP_d.BB );   % length�͍s���Ɨ񐔂̓��̑傫������^����

%%%%% �����N�̘A���֌W���`���� ���̂Q %%%%%
% �Ίp�����͂��ׂ�-1  �e�񂪍����炻�ꂼ�ꃊ���N1,2��\���Ă���  �������Ō������Ă��郊���N�ԍ��ɑΉ�����s��1������
LP_d.SS =  [  -1  1  0  0     0  0  0  0;
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
zero3 = zeros(3,1);
baseToJoint1 = [-param.robot.baseWidth/2; param.robot.baseDepth/2; 0] - param.robot.comOffset_base;
baseToJoint5 = [ param.robot.baseWidth/2; param.robot.baseDepth/2; 0] - param.robot.comOffset_base;
LP_d.c0 = [ baseToJoint1, zero3, zero3, zero3,   baseToJoint5, zero3, zero3, zero3 ];

%%%%% �x�[�X�� (�����N0) �̎��� %%%%%
LP_d.m0 = param.robot.mass_base;%7.70;5.150+2.550

%%%%% �x�[�X�� (�����N0) �̊������[�����g %%%%% 
LP_d.inertia0 = param.robot.inertia_base;
            
%%%%% �e�����N�i�x�[�X�ȊO�j�̎��� %%%%%
LP_d.m = repmat([param.robot.mass_links, param.robot.mass_endEffector], [1,2]);

%%%%% �����N�n�S�̂̎��� %%%%%
LP_d.mass = sum( LP_d.m ) + LP_d.m0;

%%%%% �e�����N�Ԃ̍��W�n�̉�]�֌W %%%%%
% �����̃����N���W�n����]������  ��̓����N �s��x,y,z����\��.
LP_d.Qi = zeros([3,8]);
    
%%%%% �e�����N�̊������[�����g %%%%%
LP_d.inertia = repmat([param.robot.inertia_links, param.robot.inertia_endEffector], [1,2]);

%%%%% �e�����N�d�S����֐߂ւ̈ʒu�x�N�g�� %%%%%
%���W�̕\���̓����N�̍����̍��W�n����D
% y�v�f�ȊO�S���[��
LP_d.cc = zeros(3,8,8);
centerOfLink2joint_low = [zero3'; -param.robot.length_links * .5; zero3'] - param.robot.comOffset_links;
centerOfLink2joint_up  = [zero3';  param.robot.length_links * .5; zero3'] - param.robot.comOffset_links;
LP_d.cc(:, [1,  10, 19]) = centerOfLink2joint_low;    % [:,n,n] n = 1,2,3 com_link N to joint N
LP_d.cc(:, [37, 46, 55]) = centerOfLink2joint_low;    % [:,n,n] n = 5,6,7 com_link N to joint N
LP_d.cc(:, [9,  18, 27]) = centerOfLink2joint_up;     % [:,n,n-1] n = 2,3,4 com_link N to joint N-1
LP_d.cc(:, [45, 54, 63]) = centerOfLink2joint_up;     % [:,n,n-1] n = 6,7,8 com_link N to joint N-1

length_endEffec = param.robot.endEffector_h * cos(param.robot.endEffector_gamma);
centerOfEE2End   = [0,  length_endEffec * .5, 0]' - param.robot.comOffset_endEffector;
centerOfEE2joint = [0, -length_endEffec * .5, 0]' - param.robot.comOffset_endEffector;
LP_d.cc(:, 4, 4) = centerOfEE2joint;
LP_d.cc(:, 8, 8) = centerOfEE2joint;

% ���[�����N����[�_�܂ł̈ʒu�x�N�g��  ���[�����N�̏d�S������i�Q�̐�[���̒��_�j�ւ̈ʒu�x�N�g��  ���[���������Ȃ������N��0������
LP_d.ce = zeros(3, 8);
LP_d.ce(:, 4) = centerOfEE2End;
LP_d.ce(:, 8) = centerOfEE2End;

% ���[�����N�ƒ[�_�̉�]�֌W  ���[�����N�̍��{�Ɛ�[�̍��W�n������ł���Ƃ�������  �L�q�@��Qi�Ɠ���  ���[���������Ȃ������N��0������
LP_d.Qe = zeros([3,8]);

% EOF