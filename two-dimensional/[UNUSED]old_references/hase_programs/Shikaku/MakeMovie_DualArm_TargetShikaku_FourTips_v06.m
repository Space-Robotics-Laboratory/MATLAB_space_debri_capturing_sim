%%%%%%%%%% �A�j���[�V��������쐬��p for 3�����N�o�r�F�����{�b�g�̐ڐG�V�~�����[�V����(��摬�x����{�x�[�X�t���[) �^�[�Q�b�g�l�p�^
%%%%%%%%%% ���{�b�g�����`�̂ݕ\��version
%%%%%%%%%% DualArm_TargetShikaku_Tracking_Contact_MakeMovie  2018�N1��24��
clc
clear
close all

%%%%%%%%%% �t�H���_�p�X�ݒ� %%%%%%%%%
% % �����̏ꍇ�́�
day = datestr( now, 'yyyy-mmdd' );
% % �����ȊO�̓������w�肷��ꍇ�́�
day = '2019-0118';

% ���[�h�C���Ԑݒ�
movie_d_time = 0.03;
simplemode = 1 ;
kakudaimode = 1 ;
chokakudaimode = 0 ;
mannaka = 0.3;
mannakamode = 0 ;
gridmode = 0 ;

path = 'dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';

% ���Ԏw��
timepath = [ day, '-', '003857' ] ;
% �p�X�w��
datepath = [ 'C:/Users/falsi/Desktop/Matlab/BackUp/', day, '/', timepath, '_' ];
datepath = [ 'C:\Users\falsi\Desktop\�C�m�_��\hase_fig\Case_Simulation_Results\Sim5\', timepath, '_' ];
datpath = [ datepath, path, '/', timepath, '-', 'dat' ];
movpath = [ datepath, path, '/', timepath, '-', 'mov' ];

n_figure = 999;

%%% �e��ݒ�(fig�t�@�C���쐬����ҏW�\) %%%

FSA = 40;   % �������E���x���̃t�H���g��
FST = 60;   % �^�C�g���̃t�H���g��
left =   400;   % �E�B���h�E�̈ʒu��ύX����ۂ̓R�R��������
bottom = 40;
width =  1000;
height = 950;
Position = [ left bottom width height ];

usui_haiiro = [ 0.9 0.9 0.9 ];   % �D���ȐF���w��
koi_haiiro = [ 0.5 0.5 0.5 ];
usui_ao = [ 102/255 153/255 255/255 ];
orange = [ 255/255 217/255 0 ];
green = [ 11/255 102/255 32/255 ];
kuro = [ 0 0 0 ];
shiro = [ 1 1 1 ];

framerate = 40;   % �t���[����?
n = 50;

sen =  2;   % ���̑���
sen0 = 3;
sen1 = 1;
sen2 = 2;
sen3 = 3;
sen4 = 4;

n_figure = n_figure + 1;

%%%%%%%%%% �l�ǂݍ��� %%%%%%%%%%
% �@�\����
mat_01 = dlmread( [ datpath, '/', timepath, '_', '01_Length_etc.dat' ], '\t', 1, 0 );
% ������
mat_11 = dlmread( [ datpath, '/', timepath, '_', '11_Estimation.dat' ], '\t', 1, 0 );
% �� �A�j���[�V�����p
mat_14 = dlmread( [ datpath, '/', timepath, '_', '14_Force_ForMovie.dat' ], '\t', 1, 0 );
% ���{�b�g �A�j���[�V�����p
mat_15 = dlmread( [ datpath, '/', timepath, '_', '15_Robot_ForMovie.dat' ], '\t', 1, 0 ); 
% �^�[�Q�b�g �A�j���[�V�����p
mat_16 = dlmread( [ datpath, '/', timepath, '_', '16_STarget_ForMovie.dat' ], '\t', 1, 0 );
% memo
mat_17 = dlmread( [ datpath, '/', timepath, '_', '17_memo.dat' ], '\t', 1, 0 ); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ����̍쐬 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% {


%%%%%%%%%% �ڐG�E�J�E���g�t���O�ݒ� %%%%%%%%%%
frame_n = 1;             
for_d_angle = linspace( 0, 2 * pi , n );   % 0�`2�΂̋�Ԃ̓��Ԋu�̓_��n�܂ލs�x�N�g����Ԃ�
for_d_angle2 = linspace( 0, pi / 2 , n );   % 0�`2�΂̋�Ԃ̓��Ԋu�̓_��n�܂ލs�x�N�g����Ԃ�
for_pi_half = 0:1/n:pi/2 ;

%%%%%%%%%% �����ݒ� %%%%%%%%%%
r_tip = mat_01(1);   % ���������a
side_target = mat_01(2);   % �^�[�Q�b�g1��
r_motor = mat_01(4);  % ���[�^���a
r_base = mat_01(5)/2;   % �x�[�X�̏d�S�`��p�~
base_tate = mat_01(7); base_yoko = mat_01(8); base_jushin_teihen = mat_01(9);
tekubiyubisaki = mat_01(16); hirakikaku = mat_01(17);

%%%%%%%%%% �ǉ� %%%%%%%%%%
d_pos_j4 = mat_17(:,2:3); d_pos_j8 = mat_17(:,5:6);
xeL = mat_17(:,8:9); xeR = mat_17(:,10:11);
d1 = mat_17(:,12:13); d2 = mat_17(:,15:16); d3 = mat_17(:,18:19); d4 = mat_17(:,21:22);
qL3 = mat_17(:,24); qR7 = mat_17(:,25); qL2 = mat_17(:,26); qR6 = mat_17(:,27);
dqL3 = mat_17(:,28); dqR7 = mat_17(:,29);
phase = mat_17(:,30);


% ���{�b�g
mvtime = mat_15(:,1);   % time �o�ߍ��ݎ���
pos_rb = mat_15(:,2:3);   % �x�[�X�d�S�ʒux,y
pos_j1 = mat_15(:,5:6); pos_j2 = mat_15(:,7:8); pos_j3 = mat_15(:,9:10); pos_j4 = mat_15(:,11:12);
pos_j5 = mat_15(:,13:14); pos_j6 = mat_15(:,15:16); pos_j7 = mat_15(:,17:18); pos_j8 = mat_15(:,19:20);
pos_eL1 = mat_15(:,21:22); pos_eL2 = mat_15(:,24:25); pos_eR1 = mat_15(:,27:28); pos_eR2 = mat_15(:,30:31);
rad_eL1 = mat_15(:,35:36); rad_eL2 = mat_15(:,38:39); rad_eR1 = mat_15(:,41:42); rad_eR2 = mat_15(:,44:45);
ang_bs = mat_15(:,47);                % �x�[�X�d�S�p��
% �^�[�Q�b�g
pos_ts =  mat_16(:,2:3);  % �^�[�Q�b�g�d�S�ʒux,y
shikaku1 = mat_16(:,5:6); shikaku2 = mat_16(:,8:9); shikaku3 = mat_16(:,11:12); shikaku4 = mat_16(:,14:15);   % ���_
ang_tr =  mat_16(:,19);   % �^�[�Q�b�g�d�S�p��
pos_ts_geo =  mat_11(:,24:25);  % �^�[�Q�b�g�􉽒��S�ʒux,y
% ������
obs1_tr = mat_11(:,2:3); obs2_tr = mat_11(:,5:6);   % �ϑ��_
pos_est = mat_11(:,8:9);  % ����d�S�ʒu
% �͏��
nsen = 0.01;
PointC_1_ani = mat_14(:,26:27); PointC_2_ani = mat_14(:,29:30); PointC_3_ani = mat_14(:,32:33); PointC_4_ani = mat_14(:,35:36);
FR_N_1_ani = nsen * mat_14(:,38:39); FR_N_2_ani = nsen * mat_14(:,41:42); FR_N_3_ani = nsen * mat_14(:,44:45); FR_N_4_ani = nsen * mat_14(:,47:48);
F_ri_1_ani = nsen * mat_14(:,50:51); F_ri_2_ani = nsen * mat_14(:,53:54); F_ri_3_ani = nsen * mat_14(:,56:57); F_ri_4_ani = nsen * mat_14(:,59:60);


%%%%%%%%%% �����ݒ� %%%%%%%%%%
% ���{�b�g�x�[�X�d�S�ʒu����O�g�[�_�܂ł̋���   % 2018�V�e�X�g�x�b�h�p�ɏ�������
l_hidariue =    sqrt( ( base_yoko / 2 )^2 + ( base_tate - base_jushin_teihen )^2 );
l_migiue =      sqrt( ( base_yoko / 2 )^2 + ( base_tate - base_jushin_teihen )^2 );
l_hidarishita = sqrt( ( base_yoko / 2 )^2 + ( base_jushin_teihen )^2 );
l_migishita =   sqrt( ( base_yoko / 2 )^2 + ( base_jushin_teihen )^2 );
psi1 = atan( ( base_yoko / 2 ) / ( base_tate - base_jushin_teihen ) );   % �x�[�X�d�S���Wy�������1�̊p�x�ɍ���̐����`���_
psi2 = atan( base_jushin_teihen / ( base_yoko / 2 ) );   % �x�[�X�d�S���Wx�������2�̊p�x�ɍ����̐����`���_
l_target = sqrt( 2 ) * side_target / 2;
lb = sqrt( ( pos_rb(1,1) - pos_j1(1,1) )^2 + ( pos_rb(1,2) - pos_j1(1,2) )^2 );
psi = atan( -( pos_j1(1,1) - pos_rb(1,1) ) / ( pos_j1(1,2) - pos_rb(1,2) ) );


%%%%%%%%%% ���_ %%%%%%%%%%
% �x�[�X�O�g�[�_�ʒu   % 2017�V�e�X�g�x�b�h�p�ɏ�������
t_x = [ shikaku1(:,1) shikaku2(:,1) shikaku3(:,1) shikaku4(:,1) shikaku1(:,1) ]; t_y = [ shikaku1(:,2) shikaku2(:,2) shikaku3(:,2) shikaku4(:,2) shikaku1(:,2) ];
b_x = [ pos_j1(:,1) pos_rb(:,1)+lb*sin(ang_bs-psi) pos_rb(:,1)+lb*sin(ang_bs+psi) pos_j5(:,1) pos_j1(:,1) ];   % ����t�������甽���v���ō���t�����ɖ߂�
b_y = [ pos_j1(:,2) pos_rb(:,2)-lb*cos(ang_bs-psi) pos_rb(:,2)-lb*cos(ang_bs+psi) pos_j5(:,2) pos_j1(:,2) ];
td_x = [ d1(:,1) d2(:,1) d3(:,1) d4(:,1) d1(:,1) ]; td_y = [ d1(:,2) d2(:,2) d3(:,2) d4(:,2) d1(:,2) ];


%%%%%%%%%% �����N���_�ݒ� %%%%%%%%%%
L = tekubiyubisaki; b = sin(hirakikaku);
%%%%% ���r %%%%%
arm_L_x = [ pos_j1(:,1) pos_j2(:,1) pos_j3(:,1) pos_j4(:,1) ]; arm_L_y = [ pos_j1(:,2) pos_j2(:,2) pos_j3(:,2) pos_j4(:,2) ];   % �֐�1,2,3,4�̍��W�ݒ�
hand_L_x = [ pos_eL1(:,1) pos_j4(:,1)-L*b*cos(rad_eL1(:,1)) pos_j4(:,1)+L*b*cos(rad_eL1(:,1)) pos_eL2(:,1) ];   % ����x���W�ݒ�
hand_L_y = [ pos_eL1(:,2) pos_j4(:,2)-L*b*sin(rad_eL1(:,1)) pos_j4(:,2)+L*b*sin(rad_eL1(:,1)) pos_eL2(:,2) ];   % y���W
%%%%% �E�r %%%%%
arm_R_x = [ pos_j5(:,1) pos_j6(:,1) pos_j7(:,1) pos_j8(:,1) ]; arm_R_y = [ pos_j5(:,2) pos_j6(:,2) pos_j7(:,2) pos_j8(:,2) ];   % ���̍��W�ݒ�
hand_R_x = [ pos_eR1(:,1) pos_j8(:,1)-L*b*cos(rad_eR1(:,1)) pos_j8(:,1)+L*b*cos(rad_eR1(:,1)) pos_eR2(:,1) ];   % ����x���W�ݒ�
hand_R_y = [ pos_eR1(:,2) pos_j8(:,2)-L*b*sin(rad_eR1(:,1)) pos_j8(:,2)+L*b*sin(rad_eR1(:,1)) pos_eR2(:,2) ];   % y���W


%%%%%%%%%% �͊m�F %%%%%%%%%%
N_L1x = [ PointC_1_ani(:,1), PointC_1_ani(:,1)+FR_N_1_ani(:,1) ]; N_L1y = [ PointC_1_ani(:,2), PointC_1_ani(:,2)+FR_N_1_ani(:,2) ];
N_L2x = [ PointC_2_ani(:,1), PointC_2_ani(:,1)+FR_N_2_ani(:,1) ]; N_L2y = [ PointC_2_ani(:,2), PointC_2_ani(:,2)+FR_N_2_ani(:,2) ];
N_R1x = [ PointC_3_ani(:,1), PointC_3_ani(:,1)+FR_N_3_ani(:,1) ]; N_R1y = [ PointC_3_ani(:,2), PointC_3_ani(:,2)+FR_N_3_ani(:,2) ];
N_R2x = [ PointC_4_ani(:,1), PointC_4_ani(:,1)+FR_N_4_ani(:,1) ]; N_R2y = [ PointC_4_ani(:,2), PointC_4_ani(:,2)+FR_N_4_ani(:,2) ];
F_L1x = [ PointC_1_ani(:,1), PointC_1_ani(:,1)+F_ri_1_ani(:,1) ]; F_L1y = [ PointC_1_ani(:,2), PointC_1_ani(:,2)+F_ri_1_ani(:,2) ];
F_L2x = [ PointC_2_ani(:,1), PointC_2_ani(:,1)+F_ri_2_ani(:,1) ]; F_L2y = [ PointC_2_ani(:,2), PointC_2_ani(:,2)+F_ri_2_ani(:,2) ];
F_R1x = [ PointC_3_ani(:,1), PointC_3_ani(:,1)+F_ri_3_ani(:,1) ]; F_R1y = [ PointC_3_ani(:,2), PointC_3_ani(:,2)+F_ri_3_ani(:,2) ];
F_R2x = [ PointC_4_ani(:,1), PointC_4_ani(:,1)+F_ri_4_ani(:,1) ]; F_R2y = [ PointC_4_ani(:,2), PointC_4_ani(:,2)+F_ri_4_ani(:,2) ];


%%%%%% �p�x�m�F %%%%%%
fx = .4; fy = .6;
QL2x = [ -fx*ones(size(qL2)), -fx*ones(size(qL2))+.1*cos(qL2) ]; QL2y = [ fy*ones(size(qL2)), fy*ones(size(qL2))+.1*sin(qL2) ];
QR6x = [ fx*ones(size(qR6)), fx*ones(size(qR6))+.1*cos(qR6) ]; QR6y = [ fy*ones(size(qR6)), fy*ones(size(qR6))+.1*sin(qR6) ];
QL3x = [ QL2x(:,2), QL2x(:,2)+.1/2*cos(qL3) ]; QL3y = [ QL2y(:,2), QL2y(:,2)+.1/2*sin(qL3) ];
QR7x = [ QR6x(:,2), QR6x(:,2)+.1/2*cos(qR7) ]; QR7y = [ QR6y(:,2), QR6y(:,2)+.1/2*sin(qR7) ];
dQL3x = [ QL2x(:,2), QL2x(:,2)+.1/2*cos(dqL3) ]; dQL3y = [ QL2y(:,2), QL2y(:,2)+.1/2*sin(dqL3) ];
dQR7x = [ QR6x(:,2), QR6x(:,2)+.1/2*cos(dqR7) ]; dQR7y = [ QR6y(:,2), QR6y(:,2)+.1/2*sin(dqR7) ];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% �����}�`�� %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(n_figure);
hold on; box on; axis equal; % grid on; grid minor;
set( gcf, 'Position', Position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );
set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
if kakudaimode == 0
 if chokakudaimode == 1
 xlim( [ -0.2, 0.2 ] ); ylim( [ 0.2, 0.6 ] );
 else
 xlim( [ -0.5, 0.5 ] ); ylim( [ -0.2, 0.8 ] );
 end
elseif kakudaimode == 1
 if chokakudaimode == 1
 xlim( [ -0.2, 0.2 ] ); ylim( [ 0.2, 0.6 ] );
 else
 xlim( [ -0.4, 0.4 ] ); ylim( [ -0.2, 0.6 ] );
 end
 if mannakamode == 1
 xlim( [ pos_ts_geo(1,1)-mannaka, pos_ts_geo(1,1)+mannaka ] ); ylim( [ pos_ts_geo(1,2)-mannaka, pos_ts_geo(1,2)+mannaka ] );
 end
end
xlabel( 'x [m]' ); ylabel( 'y [m]' );
f = figure(n_figure);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ����ۑ� %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
moviename = [ movpath, '/', timepath, '_DualArm_TargetShikaku_', 'FrameRate', num2str(framerate), '_Mov.avi' ];
if mannakamode == 1
moviename = [ movpath, '/', timepath, '_DualArm_TargetShikaku_', 'FrameRate', num2str(framerate), '_MovOC.avi' ];
end
% writerObj = VideoWriter( [ datepath, path, '/', timepath, '-', 'mov/', 'sim100.avi' ] );
writerObj = VideoWriter( moviename );
set( writerObj, 'FrameRate', framerate, 'Quality', 100 );
open( writerObj );

r0s = r_tip * sin(for_d_angle); r0c = r_tip * cos(for_d_angle);
r1s = r_motor * sin(for_d_angle); r1c = r_motor * cos(for_d_angle);
r2s = r_motor/2 * sin(for_d_angle); r2c = r_motor/2 * cos(for_d_angle);

if simplemode == 0
    % �ǉ�
    d_rb_j4 = fill( d_pos_j4(1,1) + r2s, d_pos_j4(1,2) + r2c, 'r', 'LineWidth', sen1 ); d_rb_j8 = fill( d_pos_j8(1,1) + r2s, d_pos_j8(1,2) + r2c, 'b', 'LineWidth', sen1 );
    d_eL = fill( xeL(1,1) + r2s*2, xeL(1,2) + r2c*2, 'm', 'LineWidth', sen1 ); d_eR = fill( xeR(1,1) + r2s*2, xeR(1,2) + r2c*2, 'c', 'LineWidth', sen1 );
    td  = fill( td_x(1,1:5), td_y(1,1:5), usui_haiiro, 'LineWidth', sen1 );
    dt1 = fill( d1(1,1) + r2s/2, d1(1,2) + r2c/2, 'g', 'LineWidth', sen1 ); dt2 = fill( d2(1,1) + r2s/2, d2(1,2) + r2c/2, 'k', 'LineWidth', sen1 );
    dt3 = fill( d3(1,1) + r2s/2, d3(1,2) + r2c/2, 'k', 'LineWidth', sen1 ); dt4 = fill( d4(1,1) + r2s/2, d4(1,2) + r2c/2, 'k', 'LineWidth', sen1 );
    QLL2 = plot( QL2x(1,:), QL2y(1,:), 'r', 'LineWidth', sen2 ); QRR6 = plot( QR6x(1,:), QR6y(1,:), 'b', 'LineWidth', sen2 );
    QLL3 = plot( QL3x(1,:), QL3y(1,:), 'r', 'LineWidth', sen2 ); QRR7 = plot( QR7x(1,:), QR7y(1,:), 'b', 'LineWidth', sen2 );
    dQLL3 = plot( dQL3x(1,:), dQL3y(1,:), 'm', 'LineWidth', sen2 ); dQRR7 = plot( dQR7x(1,:), dQR7y(1,:), 'c', 'LineWidth', sen2 );
    N_L1 = plot( N_L1x(1,:), N_L1y(1,:), 'c-', 'linewidth', sen2 ); N_L2 = plot( N_L2x(1,:), N_L2y(1,:), 'c-', 'linewidth', sen2 );
    N_R1 = plot( N_R1x(1,:), N_R1y(1,:), 'c-', 'linewidth', sen2 ); N_R2 = plot( N_R2x(1,:), N_R2y(1,:), 'c-', 'linewidth', sen2 );
    F_L1 = plot( F_L1x(1,:), F_L1y(1,:), 'y-', 'linewidth', sen2 ); F_L2 = plot( F_L2x(1,:), F_L2y(1,:), 'y-', 'linewidth', sen2 );
    F_R1 = plot( F_R1x(1,:), F_R1y(1,:), 'y-', 'linewidth', sen2 ); F_R2 = plot( F_R2x(1,:), F_R2y(1,:), 'y-', 'linewidth', sen2 );
    pha = plot( phase(1,1)*0.1/2, 0.8, 'r*', 'linewidth', 10 );
end

% ���{�b�g
    rb_base  = fill( b_x(1,:), b_y(1,:), orange, 'LineWidth', sen2 );   % �x�[�X�l�p�`���_��`��
    rb_arm_L = plot( arm_L_x(1,:), arm_L_y(1,:), 'b-', 'linewidth', sen2 ); rb_arm_R = plot( arm_R_x(1,:), arm_R_y(1,:), 'b-', 'linewidth', sen2 );   % �r�֐߂�`��
    rb_hand_L = plot( hand_L_x(1,:), hand_L_y(1,:), 'b-', 'linewidth', sen2 ); rb_hand_R = plot( hand_R_x(1,:), hand_R_y(1,:), 'b-', 'linewidth', sen2 );   % �r�֐߂�`��
    rb_j1 = fill( pos_j1(1,1) + r1s, pos_j1(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 ); rb_j2 = fill( pos_j2(1,1) + r1s, pos_j2(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 );
    rb_j3 = fill( pos_j3(1,1) + r1s, pos_j3(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 ); rb_j4 = fill( pos_j4(1,1) + r2s, pos_j4(1,2) + r2c, shiro, 'LineWidth', sen1 );
    rb_j5 = fill( pos_j5(1,1) + r1s, pos_j5(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 ); rb_j6 = fill( pos_j6(1,1) + r1s, pos_j6(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 );
    rb_j7 = fill( pos_j7(1,1) + r1s, pos_j7(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 ); rb_j8 = fill( pos_j8(1,1) + r2s, pos_j8(1,2) + r2c, shiro, 'LineWidth', sen1 );   % �֐߂̃��[�^�~��100�̓_�ŕ`��
    % �^�[�Q�b�g
    tg_basesen = fill( t_x(1,:), t_y(1,:), 'r-', 'LineWidth', sen2 );
    % �N���[�W��
 if mannakamode == 1
I = 1;
s11 = pos_eL1 + l_target * [ cos(pi/4*3 + ang_tr ), sin(pi/4*3 + ang_tr ) ];
s21 = pos_eL1 + l_target * [ cos(pi/4*5 + ang_tr ), sin(pi/4*5 + ang_tr ) ];
s31 = pos_eL1 + l_target * [ cos(pi/4*7 + ang_tr ), sin(pi/4*7 + ang_tr ) ];
s41 = pos_eL1 + l_target * [ cos(pi/4*9 + ang_tr ), sin(pi/4*9 + ang_tr ) ];
s12 = pos_eL2 + l_target * [ cos(pi/4*3 + ang_tr ), sin(pi/4*3 + ang_tr ) ];
s22 = pos_eL2 + l_target * [ cos(pi/4*5 + ang_tr ), sin(pi/4*5 + ang_tr ) ];
s32 = pos_eL2 + l_target * [ cos(pi/4*7 + ang_tr ), sin(pi/4*7 + ang_tr ) ];
s42 = pos_eL2 + l_target * [ cos(pi/4*9 + ang_tr ), sin(pi/4*9 + ang_tr ) ];
s13 = pos_eR1 + l_target * [ cos(pi/4*3 + ang_tr ), sin(pi/4*3 + ang_tr ) ];
s23 = pos_eR1 + l_target * [ cos(pi/4*5 + ang_tr ), sin(pi/4*5 + ang_tr ) ];
s33 = pos_eR1 + l_target * [ cos(pi/4*7 + ang_tr ), sin(pi/4*7 + ang_tr ) ];
s43 = pos_eR1 + l_target * [ cos(pi/4*9 + ang_tr ), sin(pi/4*9 + ang_tr ) ];
s14 = pos_eR2 + l_target * [ cos(pi/4*3 + ang_tr ), sin(pi/4*3 + ang_tr ) ];
s24 = pos_eR2 + l_target * [ cos(pi/4*5 + ang_tr ), sin(pi/4*5 + ang_tr ) ];
s34 = pos_eR2 + l_target * [ cos(pi/4*7 + ang_tr ), sin(pi/4*7 + ang_tr ) ];
s44 = pos_eR2 + l_target * [ cos(pi/4*9 + ang_tr ), sin(pi/4*9 + ang_tr ) ];
S1 = fill( [ s11(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s11(I,1)+r_tip*cos(pi/2+ang_tr(I,1)+for_d_angle2), s11(I,1)+r_tip*cos(pi+ang_tr(I,1)), ...
             s21(I,1)+r_tip*cos(pi+ang_tr(I,1)), s21(I,1)+r_tip*cos(pi+ang_tr(I,1)+for_d_angle2), s21(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), ...
             s31(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), s31(I,1)+r_tip*cos(+pi/2*3+ang_tr(I,1)+for_d_angle2), s31(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), ...
             s41(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), s41(I,1)+r_tip*cos(+pi*2+ang_tr(I,1)+for_d_angle2), s41(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s11(I,1)+r_tip*cos(pi/2+ang_tr(I,1)) ], ...
           [ s11(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s11(I,2)+r_tip*sin(pi/2+ang_tr(I,1)+for_d_angle2), s11(I,2)+r_tip*sin(pi+ang_tr(I,1)), ...
             s21(I,2)+r_tip*sin(pi+ang_tr(I,1)), s21(I,2)+r_tip*sin(+pi+ang_tr(I,1)+for_d_angle2), s21(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), ...
             s31(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), s31(I,2)+r_tip*sin(+pi/2*3+ang_tr(I,1)+for_d_angle2), s31(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), ...
             s41(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), s41(I,2)+r_tip*sin(+pi*2+ang_tr(I,1)+for_d_angle2), s41(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s11(I,2)+r_tip*sin(pi/2+ang_tr(I,1)) ], ...
             green, 'LineWidth', 1 );
S2 = fill( [ s12(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s12(I,1)+r_tip*cos(pi/2+ang_tr(I,1)+for_d_angle2), s12(I,1)+r_tip*cos(pi+ang_tr(I,1)), ...
             s22(I,1)+r_tip*cos(pi+ang_tr(I,1)), s22(I,1)+r_tip*cos(pi+ang_tr(I,1)+for_d_angle2), s22(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), ...
             s32(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), s32(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)+for_d_angle2), s32(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), ...
             s42(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), s42(I,1)+r_tip*cos(pi*2+ang_tr(I,1)+for_d_angle2), s42(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s12(I,1)+r_tip*cos(pi/2+ang_tr(I,1)) ], ...
           [ s12(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s12(I,2)+r_tip*sin(pi/2+ang_tr(I,1)+for_d_angle2), s12(I,2)+r_tip*sin(pi+ang_tr(I,1)), ...
             s22(I,2)+r_tip*sin(pi+ang_tr(I,1)), s22(I,2)+r_tip*sin(pi+ang_tr(I,1)+for_d_angle2), s22(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), ...
             s32(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), s32(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)+for_d_angle2), s32(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), ...
             s42(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), s42(I,2)+r_tip*sin(pi*2+ang_tr(I,1)+for_d_angle2), s42(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s12(I,2)+r_tip*sin(pi/2+ang_tr(I,1)) ], ...
             green, 'LineWidth', 1 );
S3 = fill( [ s13(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s13(I,1)+r_tip*cos(pi/2+ang_tr(I,1)+for_d_angle2), s13(I,1)+r_tip*cos(pi+ang_tr(I,1)), ...
             s23(I,1)+r_tip*cos(pi+ang_tr(I,1)), s23(I,1)+r_tip*cos(pi+ang_tr(I,1)+for_d_angle2), s23(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), ...
             s33(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), s33(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)+for_d_angle2), s33(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), ...
             s43(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), s43(I,1)+r_tip*cos(pi*2+ang_tr(I,1)+for_d_angle2), s43(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s13(I,1)+r_tip*cos(pi/2+ang_tr(I,1)) ], ...
           [ s13(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s13(I,2)+r_tip*sin(pi/2+ang_tr(I,1)+for_d_angle2), s13(I,2)+r_tip*sin(pi+ang_tr(I,1)), ...
             s23(I,2)+r_tip*sin(pi+ang_tr(I,1)), s23(I,2)+r_tip*sin(pi+ang_tr(I,1)+for_d_angle2), s23(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), ...
             s33(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), s33(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)+for_d_angle2), s33(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), ...
             s43(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), s43(I,2)+r_tip*sin(pi*2+ang_tr(I,1)+for_d_angle2), s43(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s13(I,2)+r_tip*sin(pi/2+ang_tr(I,1)) ], ...
             green, 'LineWidth', 1 );
S4 = fill( [ s14(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s14(I,1)+r_tip*cos(pi/2+ang_tr(I,1)+for_d_angle2), s14(I,1)+r_tip*cos(pi+ang_tr(I,1)), ...
             s24(I,1)+r_tip*cos(pi+ang_tr(I,1)), s24(I,1)+r_tip*cos(pi+ang_tr(I,1)+for_d_angle2), s24(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), ...
             s34(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), s34(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)+for_d_angle2), s34(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), ...
             s44(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), s44(I,1)+r_tip*cos(pi*2+ang_tr(I,1)+for_d_angle2), s44(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s14(I,1)+r_tip*cos(pi/2+ang_tr(I,1)) ], ...
           [ s14(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s14(I,2)+r_tip*sin(pi/2+ang_tr(I,1)+for_d_angle2), s14(I,2)+r_tip*sin(pi+ang_tr(I,1)), ...
             s24(I,2)+r_tip*sin(pi+ang_tr(I,1)), s24(I,2)+r_tip*sin(pi+ang_tr(I,1)+for_d_angle2), s24(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), ...
             s34(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), s34(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)+for_d_angle2), s34(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), ...
             s44(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), s44(I,2)+r_tip*sin(pi*2+ang_tr(I,1)+for_d_angle2), s44(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s14(I,2)+r_tip*sin(pi/2+ang_tr(I,1)) ], ...
             green, 'LineWidth', 1 );
 end


    % �d�S�̐�`
    rb_cross_1 = fill( [ pos_rb(1,1), ( pos_rb(1,1) + r_base * cos( ang_bs(1,1) + for_pi_half ) ),        pos_rb(1,1), ( pos_rb(1,1) + r_base * cos( ang_bs(1,1) + for_pi_half + pi ) ),     pos_rb(1,1) ], ...
                       [ pos_rb(1,2), ( pos_rb(1,2) + r_base * sin( ang_bs(1,1) + for_pi_half ) ),        pos_rb(1,2), ( pos_rb(1,2) + r_base * sin( ang_bs(1,1) + for_pi_half + pi ) ),     pos_rb(1,2) ], ...
                         kuro, 'LineWidth', sen1 );
    rb_cross_2 = fill( [ pos_rb(1,1), ( pos_rb(1,1) + r_base * cos( ang_bs(1,1) + for_pi_half + pi/2 ) ), pos_rb(1,1), ( pos_rb(1,1) + r_base * cos( ang_bs(1,1) + for_pi_half + 3*pi/2 ) ), pos_rb(1,1) ], ...
                       [ pos_rb(1,2), ( pos_rb(1,2) + r_base * sin( ang_bs(1,1) + for_pi_half + pi/2 ) ), pos_rb(1,2), ( pos_rb(1,2) + r_base * sin( ang_bs(1,1) + for_pi_half + 3*pi/2 ) ), pos_rb(1,2) ], ...
                         shiro, 'LineWidth', sen1 );
    tg_cross_1 = fill( [ pos_ts(1,1), ( pos_ts(1,1) + r_base * cos( ang_tr(1,1) + for_pi_half ) ),        pos_ts(1,1), ( pos_ts(1,1) + r_base * cos( ang_tr(1,1) + for_pi_half + pi ) ),     pos_ts(1,1) ], ...
                       [ pos_ts(1,2), ( pos_ts(1,2) + r_base * sin( ang_tr(1,1) + for_pi_half ) ),        pos_ts(1,2), ( pos_ts(1,2) + r_base * sin( ang_tr(1,1) + for_pi_half + pi ) ),     pos_ts(1,2) ], ...
                         kuro, 'LineWidth', sen1 );
    tg_cross_2 = fill( [ pos_ts(1,1), ( pos_ts(1,1) + r_base * cos( ang_tr(1,1) + for_pi_half + pi/2 ) ), pos_ts(1,1), ( pos_ts(1,1) + r_base * cos( ang_tr(1,1) + for_pi_half + 3*pi/2 ) ), pos_ts(1,1) ], ...
                       [ pos_ts(1,2), ( pos_ts(1,2) + r_base * sin( ang_tr(1,1) + for_pi_half + pi/2 ) ), pos_ts(1,2), ( pos_ts(1,2) + r_base * sin( ang_tr(1,1) + for_pi_half + 3*pi/2 ) ), pos_ts(1,2) ], ...
                         shiro, 'LineWidth', sen1 );
    rb_tip_L1 = fill( pos_eL1(1,1) + r0s, pos_eL1(1,2) + r0c, usui_ao, 'LineWidth', sen1 ); rb_tip_L2 = fill( pos_eL2(1,1) + r0s, pos_eL2(1,2) + r0c, usui_ao, 'LineWidth', sen1 );   % ����担�̉~��100�̓_�ŕ`��
    rb_tip_R1 = fill( pos_eR1(1,1) + r0s, pos_eR1(1,2) + r0c, usui_ao, 'LineWidth', sen1 ); rb_tip_R2 = fill( pos_eR2(1,1) + r0s, pos_eR2(1,2) + r0c, usui_ao, 'LineWidth', sen1 );   % ����担�̉~��100�̓_�ŕ`��
    tg_geo = fill( pos_ts_geo(1,1) + r0c/5, pos_ts_geo(1,2) + r0s/5, 'y-', 'LineWidth', sen2 );

if simplemode == 0
    % ������
    obs1 = fill( obs1_tr(1,1) + r0s/2, obs1_tr(1,2) + r0c/2, 'g', 'LineWidth', sen1 );
    obs2 = fill( obs2_tr(1,1) + r0s/2, obs2_tr(1,2) + r0c/2, kuro, 'LineWidth', sen1 );
    tg_estcom = fill( pos_est(1,1) + r_tip * sin(for_d_angle), pos_est(1,2) + r_tip * cos(for_d_angle), 'y', 'LineWidth', sen1 );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% EraseMode %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set( rb_base, 'EraseMode', 'normal' );
    set( [ rb_arm_L, rb_arm_R ], 'EraseMode', 'normal' );
    set( [ rb_hand_L, rb_hand_R ], 'EraseMode', 'normal' );
    set( [ rb_tip_L1, rb_tip_L2, rb_tip_R1, rb_tip_R2 ], 'EraseMode', 'normal' );
    set( [ rb_j1, rb_j2, rb_j3, rb_j4, rb_j5, rb_j6, rb_j7, rb_j8 ], 'EraseMode', 'normal' );
    set( tg_basesen, 'EraseMode', 'normal' );
    set( [ rb_cross_1, rb_cross_2, tg_cross_1, tg_cross_2 ], 'EraseMode', 'normal' );
if mannakamode == 1;
    set( [ S4, S2, S3, S4 ], 'EraseMode', 'normal' );
end
if simplemode == 0;
    set( [ obs1, obs2 ], 'EraseMode', 'normal' );
    set( tg_estcom, 'EraseMode', 'normal' );
    set( [ N_L1, N_L2, N_R1, N_R2, F_L1, F_L2, F_R1, F_R2 ], 'EraseMode', 'normal' );
    set( [ d_rb_j4, d_rb_j8 ], 'EraseMode', 'normal' );
    set( [ d_eL, d_eR ], 'EraseMode', 'normal' );
    set( [ dt1, dt2, dt3, dt4 ], 'EraseMode', 'normal' );
    set( td, 'EraseMode', 'normal' );
    set( [ QLL2, QRR6, QLL3, QRR7 ], 'EraseMode', 'normal' );
    set( [ dQLL3, dQRR7 ], 'EraseMode', 'normal' );
    set( pha, 'EraseMode', 'normal' );
end

% % �����}��ۑ�
% figure(n_figure)
% title( sprintf( 'Time = %0.3f [s]', mvtime(1) ), 'FontName', 'Times New Roman', 'FontSize', FST );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_1stFigOS.fig' ] );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_1stFigOS.png' ] );

%%%%%%%%%% �}���X�V %%%%%%%%%%
i = 1;
while i <= length( mat_15(:,1) )
mvtime(i)
if mvtime(i) > 10
break
end

    if rem( mvtime(i), movie_d_time ) == 0   % mvtime(i)��movie_d_time�ŏ��Z������̏�]��0�̂Ƃ�(����؂��Ƃ�)

if simplemode == 0
     % �ǉ�
       set( N_L1, 'XData', N_L1x(i,:), 'YData', N_L1y(i,:) ); set( N_L2, 'XData', N_L2x(i,:), 'YData', N_L2y(i,:) );
       set( N_R1, 'XData', N_R1x(i,:), 'YData', N_R1y(i,:) ); set( N_R2, 'XData', N_R2x(i,:), 'YData', N_R2y(i,:) );
       set( F_L1, 'XData', F_L1x(i,:), 'YData', F_L1y(i,:) ); set( F_L2, 'XData', F_L2x(i,:), 'YData', F_L2y(i,:) );
       set( F_R1, 'XData', F_R1x(i,:), 'YData', F_R1y(i,:) ); set( F_R2, 'XData', F_R2x(i,:), 'YData', F_R2y(i,:) );
       set( d_rb_j4, 'XData', d_pos_j4(i,1) + r2s, 'YData', d_pos_j4(i,2) + r2c ); set( d_rb_j8, 'XData', d_pos_j8(i,1) + r2s, 'YData', d_pos_j8(i,2) + r2c );
       set( d_eL, 'XData', xeL(i,1) + r2s*2, 'YData', xeL(i,2) + r2c*2 ); set( d_eR, 'XData', xeR(i,1) + r2s*2, 'YData', xeR(i,2) + r2c*2 );
       set( dt1, 'XData', d1(i,1) + r2s/2, 'YData', d1(i,2) + r2c/2 ); set( dt2, 'XData', d2(i,1) + r2s/2, 'YData', d2(i,2) + r2c/2 );
       set( dt3, 'XData', d3(i,1) + r2s/2, 'YData', d3(i,2) + r2c/2 ); set( dt4, 'XData', d4(i,1) + r2s/2, 'YData', d4(i,2) + r2c/2 );
       set( QLL2, 'XData', QL2x(i,:), 'YData', QL2y(i,:) ); set( QRR6, 'XData', QR6x(i,:), 'YData', QR6y(i,:) );
       set( QLL3, 'XData', QL3x(i,:), 'YData', QL3y(i,:) ); set( QRR7, 'XData', QR7x(i,:), 'YData', QR7y(i,:) );
       set( dQLL3, 'XData', dQL3x(i,:), 'YData', dQL3y(i,:) ); set( dQRR7, 'XData', dQR7x(i,:), 'YData', dQR7y(i,:) );
       set( td, 'XData', td_x(i,:), 'YData', td_y(i,:) );
       set( pha, 'XData', phase(i,1)*0.1/2, 'YData', 0.8 );
end

     % ���{�b�g
       set( rb_base,  'XData', b_x(i,:), 'YData', b_y(i,:) );
       set( rb_arm_L, 'XData', arm_L_x(i,:), 'YData', arm_L_y(i,:) ); set( rb_arm_R, 'XData', arm_R_x(i,:), 'YData', arm_R_y(i,:) );
       set( rb_hand_L, 'XData', hand_L_x(i,:), 'YData', hand_L_y(i,:) ); set( rb_hand_R, 'XData', hand_R_x(i,:), 'YData', hand_R_y(i,:) );
       set( rb_j1, 'XData', pos_j1(i,1) + r1s, 'YData', pos_j1(i,2) + r1c ); set( rb_j2, 'XData', pos_j2(i,1) + r1s, 'YData', pos_j2(i,2) + r1c );
       set( rb_j3, 'XData', pos_j3(i,1) + r1s, 'YData', pos_j3(i,2) + r1c ); set( rb_j4, 'XData', pos_j4(i,1) + r2s, 'YData', pos_j4(i,2) + r2c );
       set( rb_j5, 'XData', pos_j5(i,1) + r1s, 'YData', pos_j5(i,2) + r1c ); set( rb_j6, 'XData', pos_j6(i,1) + r1s, 'YData', pos_j6(i,2) + r1c );
       set( rb_j7, 'XData', pos_j7(i,1) + r1s, 'YData', pos_j7(i,2) + r1c ); set( rb_j8, 'XData', pos_j8(i,1) + r2s, 'YData', pos_j8(i,2) + r2c );
       set( rb_cross_1, 'XData', [ pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half ) ),        pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half + pi ) ),     pos_rb(i,1) ], ...
                        'YData', [ pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half ) ),        pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half + pi ) ),     pos_rb(i,2) ] );
       set( rb_cross_2, 'XData', [ pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half + pi/2 ) ), pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half + 3*pi/2 ) ), pos_rb(i,1) ], ...
                        'YData', [ pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half + pi/2 ) ), pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half + 3*pi/2 ) ), pos_rb(i,2) ] );
     % �^�[�Q�b�g
       set( tg_basesen, 'XData', t_x(i,:), 'YData', t_y(i,:) );
 if mannakamode == 1
     % �N���[�W�� 
       I = i;
       set( S1, 'XData', [ s11(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s11(I,1)+r_tip*cos(pi/2+ang_tr(I,1)+for_d_angle2), s11(I,1)+r_tip*cos(pi+ang_tr(I,1)), ...
             s21(I,1)+r_tip*cos(pi+ang_tr(I,1)), s21(I,1)+r_tip*cos(pi+ang_tr(I,1)+for_d_angle2), s21(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), ...
             s31(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), s31(I,1)+r_tip*cos(+pi/2*3+ang_tr(I,1)+for_d_angle2), s31(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), ...
             s41(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), s41(I,1)+r_tip*cos(+pi*2+ang_tr(I,1)+for_d_angle2), s41(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s11(I,1)+r_tip*cos(pi/2+ang_tr(I,1)) ], ...
                'YData', [ s11(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s11(I,2)+r_tip*sin(pi/2+ang_tr(I,1)+for_d_angle2), s11(I,2)+r_tip*sin(pi+ang_tr(I,1)), ...
             s21(I,2)+r_tip*sin(pi+ang_tr(I,1)), s21(I,2)+r_tip*sin(+pi+ang_tr(I,1)+for_d_angle2), s21(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), ...
             s31(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), s31(I,2)+r_tip*sin(+pi/2*3+ang_tr(I,1)+for_d_angle2), s31(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), ...
             s41(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), s41(I,2)+r_tip*sin(+pi*2+ang_tr(I,1)+for_d_angle2), s41(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s11(I,2)+r_tip*sin(pi/2+ang_tr(I,1)) ] );
       set( S2, 'XData', [ s12(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s12(I,1)+r_tip*cos(pi/2+ang_tr(I,1)+for_d_angle2), s12(I,1)+r_tip*cos(pi+ang_tr(I,1)), ...
             s22(I,1)+r_tip*cos(pi+ang_tr(I,1)), s22(I,1)+r_tip*cos(pi+ang_tr(I,1)+for_d_angle2), s22(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), ...
             s32(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), s32(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)+for_d_angle2), s32(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), ...
             s42(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), s42(I,1)+r_tip*cos(pi*2+ang_tr(I,1)+for_d_angle2), s42(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s12(I,1)+r_tip*cos(pi/2+ang_tr(I,1)) ], ...
                'YData', [ s12(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s12(I,2)+r_tip*sin(pi/2+ang_tr(I,1)+for_d_angle2), s12(I,2)+r_tip*sin(pi+ang_tr(I,1)), ...
             s22(I,2)+r_tip*sin(pi+ang_tr(I,1)), s22(I,2)+r_tip*sin(pi+ang_tr(I,1)+for_d_angle2), s22(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), ...
             s32(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), s32(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)+for_d_angle2), s32(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), ...
             s42(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), s42(I,2)+r_tip*sin(pi*2+ang_tr(I,1)+for_d_angle2), s42(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s12(I,2)+r_tip*sin(pi/2+ang_tr(I,1)) ] );
       set( S3, 'XData', [ s13(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s13(I,1)+r_tip*cos(pi/2+ang_tr(I,1)+for_d_angle2), s13(I,1)+r_tip*cos(pi+ang_tr(I,1)), ...
             s23(I,1)+r_tip*cos(pi+ang_tr(I,1)), s23(I,1)+r_tip*cos(pi+ang_tr(I,1)+for_d_angle2), s23(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), ...
             s33(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), s33(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)+for_d_angle2), s33(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), ...
             s43(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), s43(I,1)+r_tip*cos(pi*2+ang_tr(I,1)+for_d_angle2), s43(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s13(I,1)+r_tip*cos(pi/2+ang_tr(I,1)) ], ...
                'YData', [ s13(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s13(I,2)+r_tip*sin(pi/2+ang_tr(I,1)+for_d_angle2), s13(I,2)+r_tip*sin(pi+ang_tr(I,1)), ...
             s23(I,2)+r_tip*sin(pi+ang_tr(I,1)), s23(I,2)+r_tip*sin(pi+ang_tr(I,1)+for_d_angle2), s23(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), ...
             s33(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), s33(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)+for_d_angle2), s33(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), ...
             s43(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), s43(I,2)+r_tip*sin(pi*2+ang_tr(I,1)+for_d_angle2), s43(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s13(I,2)+r_tip*sin(pi/2+ang_tr(I,1)) ] );
       set( S4, 'XData', [ s14(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s14(I,1)+r_tip*cos(pi/2+ang_tr(I,1)+for_d_angle2), s14(I,1)+r_tip*cos(pi+ang_tr(I,1)), ...
             s24(I,1)+r_tip*cos(pi+ang_tr(I,1)), s24(I,1)+r_tip*cos(pi+ang_tr(I,1)+for_d_angle2), s24(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), ...
             s34(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)), s34(I,1)+r_tip*cos(pi/2*3+ang_tr(I,1)+for_d_angle2), s34(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), ...
             s44(I,1)+r_tip*cos(pi*2+ang_tr(I,1)), s44(I,1)+r_tip*cos(pi*2+ang_tr(I,1)+for_d_angle2), s44(I,1)+r_tip*cos(pi/2+ang_tr(I,1)), s14(I,1)+r_tip*cos(pi/2+ang_tr(I,1)) ], ...
                'YData', [ s14(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s14(I,2)+r_tip*sin(pi/2+ang_tr(I,1)+for_d_angle2), s14(I,2)+r_tip*sin(pi+ang_tr(I,1)), ...
             s24(I,2)+r_tip*sin(pi+ang_tr(I,1)), s24(I,2)+r_tip*sin(pi+ang_tr(I,1)+for_d_angle2), s24(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), ...
             s34(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)), s34(I,2)+r_tip*sin(pi/2*3+ang_tr(I,1)+for_d_angle2), s34(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), ...
             s44(I,2)+r_tip*sin(pi*2+ang_tr(I,1)), s44(I,2)+r_tip*sin(pi*2+ang_tr(I,1)+for_d_angle2), s44(I,2)+r_tip*sin(pi/2+ang_tr(I,1)), s14(I,2)+r_tip*sin(pi/2+ang_tr(I,1)) ] );
 end

       set( tg_cross_1, 'XData', [ pos_ts(i,1), ( pos_ts(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half ) ),        pos_ts(i,1), ( pos_ts(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half + pi ) ),     pos_ts(i,1) ], ...
                        'YData', [ pos_ts(i,2), ( pos_ts(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half ) ),        pos_ts(i,2), ( pos_ts(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half + pi ) ),     pos_ts(i,2) ] );
       set( tg_cross_2, 'XData', [ pos_ts(i,1), ( pos_ts(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half + pi/2 ) ), pos_ts(i,1), ( pos_ts(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half + 3*pi/2 ) ), pos_ts(i,1) ], ...
                        'YData', [ pos_ts(i,2), ( pos_ts(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half + pi/2 ) ), pos_ts(i,2), ( pos_ts(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half + 3*pi/2 ) ), pos_ts(i,2) ] );
       set( rb_tip_L1, 'XData', pos_eL1(i,1) + r0s, 'YData', pos_eL1(i,2) + r0c ); set( rb_tip_L2, 'XData', pos_eL2(i,1) + r0s, 'YData', pos_eL2(i,2) + r0c );
       set( rb_tip_R1, 'XData', pos_eR1(i,1) + r0s, 'YData', pos_eR1(i,2) + r0c ); set( rb_tip_R2, 'XData', pos_eR2(i,1) + r0s, 'YData', pos_eR2(i,2) + r0c );
       set( tg_geo, 'XData', pos_ts_geo(i,1) + r0c/5, 'YData', pos_ts_geo(i,2) + r0s/5 );

if simplemode == 0
     % ������
       set( obs1, 'XData', obs1_tr(i,1) + r0s/2, 'YData', obs1_tr(i,2) + r0c/2 );
       set( obs2, 'XData', obs2_tr(i,1) + r0s/2, 'YData', obs2_tr(i,2) + r0c/2 );
       set( tg_estcom, 'XData', pos_est(i,1) + r_tip * sin(for_d_angle), 'YData', pos_est(i,2) + r_tip * cos(for_d_angle) );
end

       title( sprintf( 'Time = %0.3f [s]', mat_15(i,1) ), 'FontName', 'Times New Roman', 'FontSize', FST );
       if mannakamode == 1
          xlim( [ pos_ts_geo(i,1)-mannaka, pos_ts_geo(i,1)+mannaka ] ); ylim( [ pos_ts_geo(i,2)-mannaka, pos_ts_geo(i,2)+mannaka ] );
       end
       F( frame_n ) = getframe( n_figure );
       hold off;
       drawnow;
       writeVideo( writerObj, F( frame_n ) );
       frame_n = frame_n + 1;

    end 

%{
    if rem( mvtime(i), movie_d_time * 10 ) == 0 && mvtime(i) >= 0 && mvtime(i) <= 1.4    % mvtime(i)��movie_d_time�ŏ��Z������̏�]��0�̂Ƃ�(����؂��Ƃ�)
        % �}��ۑ�
        figure(n_figure)
        title( sprintf( 'Time = %0.3f [s]', mvtime(i) ), 'FontName', 'Times New Roman', 'FontSize', FST );
        saveas( figure(n_figure), [ movpath, '\', timepath, '_DualArm_TargetShikaku_TrackingContact_', num2str(mvtime(i)*100), 'Fig.fig' ] );
        saveas( figure(n_figure), [ movpath, '\', timepath, '_DualArm_TargetShikaku_TrackingContact_', num2str(mvtime(i)*100), 'Fig.png' ] );
    end
%}

    i = i + 1;

end

% % �Ō�̐}��ۑ�
% figure(n_figure)
% title( sprintf( 'Time = %0.3f [s]', mvtime(end) ), 'FontName', 'Times New Roman', 'FontSize', FST );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_LastFig4.fig' ] );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_LastFig4.png' ] );
% n_figure = n_figure + 1;

close( writerObj );

%}

%}   % ����쐬�I��
close all