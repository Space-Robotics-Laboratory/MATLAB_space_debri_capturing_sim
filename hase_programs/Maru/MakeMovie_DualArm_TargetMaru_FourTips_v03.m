%%%%%%%%%% アニメーション動画作成専用 for 3リンク双腕宇宙ロボットの接触シミュレーション(手先速度制御＋ベースフリー) ターゲット四角型
%%%%%%%%%% ロボット正方形のみ表示version
%%%%%%%%%% DualArm_TargetShikaku_Tracking_Contact_MakeMovie  2018年1月24日
clc
clear
close all

%%%%%%%%%% フォルダパス設定 %%%%%%%%%
% % 今日の場合は↓
day = datestr( now, 'yyyy-mmdd' );
% % 今日以外の日時を指定する場合は↓
day = '2019-0203';

path = 'dt=0.001,et=20,Xtm=0,Ytm=0,Vxtm=0,Vytm=0,THtm=0,Wtm=1,ErL=0,ErTH=45';

% 図モード設定
movie_d_time = 0.03;
simplemode = 1 ;
kakudaimode =  0;
chokakudaimode = 0 ;
mannakamode = 1;
mannaka = 0.3;
gridmode = 0 ;

% 時間指定
timepath = [ day, '-', '204515' ] ;
% パス指定
datepath = [ 'C:/Users/falsi/Desktop/Matlab/BackUp/', day, '/', timepath, '_' ];
datepath = [ 'C:\Users\falsi\Desktop\最終審査\figs\v2\SimM\', timepath, '_' ];
datpath = [ datepath, path, '/', timepath, '-', 'dat' ];
movpath = [ datepath, path, '/', timepath, '-', 'mov' ];

n_figure = 999;

%%% 各種設定(figファイル作成後も編集可能) %%%

FSA = 40;   % メモリ・ラベルのフォント数
FST = 60;   % タイトルのフォント数
left =   400;   % ウィンドウの位置を変更する際はココをいじる
bottom = 40;
width =  1000;
height = 950;
Position = [ left bottom width height ];

usui_haiiro = [ 0.9 0.9 0.9 ];   % 好きな色を指定
koi_haiiro = [ 0.5 0.5 0.5 ];
usui_ao = [ 102/255 153/255 255/255 ];
orange = [ 255/255 217/255 0 ];
green = [ 11/255 102/255 32/255 ];
kuro = [ 0 0 0 ];
shiro = [ 1 1 1 ];

framerate = 40;   % フレーム数?
n = 50;

sen =  2;   % 線の太さ
sen0 = 3;
sen1 = 1;
sen2 = 2;
sen3 = 3;
sen4 = 4;

n_figure = n_figure + 1;

%%%%%%%%%% 値読み込み %%%%%%%%%%
% 機構長さ
mat_01 = dlmread( [ datpath, '/', timepath, '_', '01_Length_etc.dat' ], '\t', 1, 0 );
% 推定情報
mat_11 = dlmread( [ datpath, '/', timepath, '_', '11_Estimation.dat' ], '\t', 1, 0 );
% 力 アニメーション用
mat_14 = dlmread( [ datpath, '/', timepath, '_', '14_Force_ForMovie.dat' ], '\t', 1, 0 );
% ロボット アニメーション用
mat_15 = dlmread( [ datpath, '/', timepath, '_', '15_Robot_ForMovie.dat' ], '\t', 1, 0 ); 
% ターゲット アニメーション用
mat_16 = dlmread( [ datpath, '/', timepath, '_', '16_MTarget_ForMovie.dat' ], '\t', 1, 0 );
% memo
mat_17 = dlmread( [ datpath, '/', timepath, '_', '17_memo.dat' ], '\t', 1, 0 ); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 動画の作成 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% {

%%%%%%%%%% 接触・カウントフラグ設定 %%%%%%%%%%
frame_n = 1;
for_d_angle = linspace( 0, 2 * pi , n );   % 0～2πの区間の等間隔の点をn個含む行ベクトルを返す(0.02π[deg]間隔)
for_pi_half = 0:1/n:pi/2 ;

%%%%%%%%%% 長さ設定 %%%%%%%%%%
r_tip = mat_01(1);   % 金属球半径
d_target = mat_01(2);   % ターゲット1辺
r_target = d_target / 2;
r_motor = mat_01(4);  % モータ半径
r_base = mat_01(5)/2;   % ベースの重心描画用円
base_tate = mat_01(7); base_yoko = mat_01(8); base_jushin_teihen = mat_01(9);
tekubiyubisaki = mat_01(16); hirakikaku = mat_01(17);

%%%%%%%%%% 追加 %%%%%%%%%%
d_pos_j4 = mat_17(:,2:3); d_pos_j8 = mat_17(:,5:6);
xeL = mat_17(:,8:9); xeR = mat_17(:,10:11);
d1 = mat_17(:,12:13); d2 = mat_17(:,15:16); d3 = mat_17(:,18:19); d4 = mat_17(:,21:22);
qL3 = mat_17(:,24); qR7 = mat_17(:,25); qL2 = mat_17(:,26); qR6 = mat_17(:,27);
dqL3 = mat_17(:,28); dqR7 = mat_17(:,29);
phase = mat_17(:,30);
d_geo = mat_17(:,34:36);

% ロボット
mvtime = mat_15(:,1);   % time 経過刻み時間
pos_rb = mat_15(:,2:3);   % ベース重心位置x,y
pos_j1 = mat_15(:,5:6); pos_j2 = mat_15(:,7:8); pos_j3 = mat_15(:,9:10); pos_j4 = mat_15(:,11:12);
pos_j5 = mat_15(:,13:14); pos_j6 = mat_15(:,15:16); pos_j7 = mat_15(:,17:18); pos_j8 = mat_15(:,19:20);
pos_eL1 = mat_15(:,21:22); pos_eL2 = mat_15(:,24:25); pos_eR1 = mat_15(:,27:28); pos_eR2 = mat_15(:,30:31);
rad_eL1 = mat_15(:,35:36); rad_eL2 = mat_15(:,38:39); rad_eR1 = mat_15(:,41:42); rad_eR2 = mat_15(:,44:45);
ang_bs = mat_15(:,47);                % ベース重心姿勢
% ターゲット
pos_tm =  mat_16(:,2:3);  % ターゲット重心位置x,y
pos_tm_geo =  mat_11(:,21:23);  % ターゲット幾何中心位置x,y
% shikaku1 = mat_16(:,5:6); shikaku2 = mat_16(:,8:9); shikaku3 = mat_16(:,11:12); shikaku4 = mat_16(:,14:15);   % 頂点
ang_tr =  mat_16(:,19);   % ターゲット重心姿勢
% 推定情報
obs1_tr = mat_11(:,2:3); obs2_tr = mat_11(:,5:6);   % 観測点
pos_est = mat_11(:,8:9);  % 推定重心位置
% 力情報
nsen = 0.01;
PointC_1_ani = mat_14(:,26:27); PointC_2_ani = mat_14(:,29:30); PointC_3_ani = mat_14(:,32:33); PointC_4_ani = mat_14(:,35:36);
FR_N_1_ani = nsen * mat_14(:,38:39); FR_N_2_ani = nsen * mat_14(:,41:42); FR_N_3_ani = nsen * mat_14(:,44:45); FR_N_4_ani = nsen * mat_14(:,47:48);
F_ri_1_ani = nsen * mat_14(:,50:51); F_ri_2_ani = nsen * mat_14(:,53:54); F_ri_3_ani = nsen * mat_14(:,56:57); F_ri_4_ani = nsen * mat_14(:,59:60);


%%%%%%%%%% 距離設定 %%%%%%%%%%
% ロボットベース重心位置から外枠端点までの距離   % 2018新テストベッド用に書き換え
l_hidariue =    sqrt( ( base_yoko / 2 )^2 + ( base_tate - base_jushin_teihen )^2 );
l_migiue =      sqrt( ( base_yoko / 2 )^2 + ( base_tate - base_jushin_teihen )^2 );
l_hidarishita = sqrt( ( base_yoko / 2 )^2 + ( base_jushin_teihen )^2 );
l_migishita =   sqrt( ( base_yoko / 2 )^2 + ( base_jushin_teihen )^2 );
psi1 = atan( ( base_yoko / 2 ) / ( base_tate - base_jushin_teihen ) );   % ベース重心座標y軸からψ1の角度に左上の正方形頂点
psi2 = atan( base_jushin_teihen / ( base_yoko / 2 ) );   % ベース重心座標x軸からψ2の角度に左下の正方形頂点
% l_target = sqrt( 2 ) * d_target / 2;
lb = sqrt( ( pos_rb(1,1) - pos_j1(1,1) )^2 + ( pos_rb(1,2) - pos_j1(1,2) )^2 );
psi = atan( -( pos_j1(1,1) - pos_rb(1,1) ) / ( pos_j1(1,2) - pos_rb(1,2) ) );

%%%%%%%%%% 頂点 %%%%%%%%%%
% ベース外枠端点位置   % 2017新テストベッド用に書き換え
% t_x = [ shikaku1(:,1) shikaku2(:,1) shikaku3(:,1) shikaku4(:,1) shikaku1(:,1) ]; t_y = [ shikaku1(:,2) shikaku2(:,2) shikaku3(:,2) shikaku4(:,2) shikaku1(:,2) ];
b_x = [ pos_j1(:,1) pos_rb(:,1)+lb*sin(ang_bs-psi) pos_rb(:,1)+lb*sin(ang_bs+psi) pos_j5(:,1) pos_j1(:,1) ];   % 左手付け根から反時計回りで左手付け根に戻る
b_y = [ pos_j1(:,2) pos_rb(:,2)-lb*cos(ang_bs-psi) pos_rb(:,2)-lb*cos(ang_bs+psi) pos_j5(:,2) pos_j1(:,2) ];
td_x = [ d1(:,1) d2(:,1) d3(:,1) d4(:,1) d1(:,1) ]; td_y = [ d1(:,2) d2(:,2) d3(:,2) d4(:,2) d1(:,2) ];

%%%%%%%%%% リンク頂点設定 %%%%%%%%%%
L = tekubiyubisaki; b = sin(hirakikaku);
%%%%% 左腕 %%%%%
arm_L_x = [ pos_j1(:,1) pos_j2(:,1) pos_j3(:,1) pos_j4(:,1) ]; arm_L_y = [ pos_j1(:,2) pos_j2(:,2) pos_j3(:,2) pos_j4(:,2) ];   % 関節1,2,3,4の座標設定
hand_L_x = [ pos_eL1(:,1) pos_j4(:,1)-L*b*cos(rad_eL1(:,1)) pos_j4(:,1)+L*b*cos(rad_eL1(:,1)) pos_eL2(:,1) ];   % 手先のx座標設定
hand_L_y = [ pos_eL1(:,2) pos_j4(:,2)-L*b*sin(rad_eL1(:,1)) pos_j4(:,2)+L*b*sin(rad_eL1(:,1)) pos_eL2(:,2) ];   % y座標
%%%%% 右腕 %%%%%
arm_R_x = [ pos_j5(:,1) pos_j6(:,1) pos_j7(:,1) pos_j8(:,1) ]; arm_R_y = [ pos_j5(:,2) pos_j6(:,2) pos_j7(:,2) pos_j8(:,2) ];   % 手先の座標設定
hand_R_x = [ pos_eR1(:,1) pos_j8(:,1)-L*b*cos(rad_eR1(:,1)) pos_j8(:,1)+L*b*cos(rad_eR1(:,1)) pos_eR2(:,1) ];   % 手先のx座標設定
hand_R_y = [ pos_eR1(:,2) pos_j8(:,2)-L*b*sin(rad_eR1(:,1)) pos_j8(:,2)+L*b*sin(rad_eR1(:,1)) pos_eR2(:,2) ];   % y座標

%%%%%%%%%% 力確認 %%%%%%%%%%
N_L1x = [ PointC_1_ani(:,1), PointC_1_ani(:,1)+FR_N_1_ani(:,1) ]; N_L1y = [ PointC_1_ani(:,2), PointC_1_ani(:,2)+FR_N_1_ani(:,2) ];
N_L2x = [ PointC_2_ani(:,1), PointC_2_ani(:,1)+FR_N_2_ani(:,1) ]; N_L2y = [ PointC_2_ani(:,2), PointC_2_ani(:,2)+FR_N_2_ani(:,2) ];
N_R1x = [ PointC_3_ani(:,1), PointC_3_ani(:,1)+FR_N_3_ani(:,1) ]; N_R1y = [ PointC_3_ani(:,2), PointC_3_ani(:,2)+FR_N_3_ani(:,2) ];
N_R2x = [ PointC_4_ani(:,1), PointC_4_ani(:,1)+FR_N_4_ani(:,1) ]; N_R2y = [ PointC_4_ani(:,2), PointC_4_ani(:,2)+FR_N_4_ani(:,2) ];
F_L1x = [ PointC_1_ani(:,1), PointC_1_ani(:,1)+F_ri_1_ani(:,1) ]; F_L1y = [ PointC_1_ani(:,2), PointC_1_ani(:,2)+F_ri_1_ani(:,2) ];
F_L2x = [ PointC_2_ani(:,1), PointC_2_ani(:,1)+F_ri_2_ani(:,1) ]; F_L2y = [ PointC_2_ani(:,2), PointC_2_ani(:,2)+F_ri_2_ani(:,2) ];
F_R1x = [ PointC_3_ani(:,1), PointC_3_ani(:,1)+F_ri_3_ani(:,1) ]; F_R1y = [ PointC_3_ani(:,2), PointC_3_ani(:,2)+F_ri_3_ani(:,2) ];
F_R2x = [ PointC_4_ani(:,1), PointC_4_ani(:,1)+F_ri_4_ani(:,1) ]; F_R2y = [ PointC_4_ani(:,2), PointC_4_ani(:,2)+F_ri_4_ani(:,2) ];

%%%%%% 角度確認 %%%%%%
fx = .4; fy = .6;
QL2x = [ -fx*ones(size(qL2)), -fx*ones(size(qL2))+.1*cos(qL2) ]; QL2y = [ fy*ones(size(qL2)), fy*ones(size(qL2))+.1*sin(qL2) ];
QR6x = [ fx*ones(size(qR6)), fx*ones(size(qR6))+.1*cos(qR6) ]; QR6y = [ fy*ones(size(qR6)), fy*ones(size(qR6))+.1*sin(qR6) ];
QL3x = [ QL2x(:,2), QL2x(:,2)+.1/2*cos(qL3) ]; QL3y = [ QL2y(:,2), QL2y(:,2)+.1/2*sin(qL3) ];
QR7x = [ QR6x(:,2), QR6x(:,2)+.1/2*cos(qR7) ]; QR7y = [ QR6y(:,2), QR6y(:,2)+.1/2*sin(qR7) ];
dQL3x = [ QL2x(:,2), QL2x(:,2)+.1/2*cos(dqL3) ]; dQL3y = [ QL2y(:,2), QL2y(:,2)+.1/2*sin(dqL3) ];
dQR7x = [ QR6x(:,2), QR6x(:,2)+.1/2*cos(dqR7) ]; dQR7y = [ QR6y(:,2), QR6y(:,2)+.1/2*sin(dqR7) ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 初期図描画 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(n_figure);
hold on; box on; axis equal;
if gridmode == 1
grid on; grid minor;
end
set( gcf, 'Position', Position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );
set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA*1. );
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
 xlim( [ pos_tm_geo(1,1)-mannaka, pos_tm_geo(1,1)+mannaka ] ); ylim( [ pos_tm_geo(1,2)-mannaka, pos_tm_geo(1,2)+mannaka ] );
 end
end
xlabel( 'x [m]' ); ylabel( 'y [m]' );
f = figure(n_figure);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 動画保存 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if simplemode == 1 && kakudaimode == 1
%   if gridmode == 1
%      moviename = [ movpath, '/', timepath, '_DualArm_TargetMaru_', 'FrameRate', num2str(framerate), '_MovSimpleKakudaiGridOC.avi' ];
%   else
%      moviename = [ movpath, '/', timepath, '_DualArm_TargetMaru_', 'FrameRate', num2str(framerate), '_MovSimpleKakudaiOC.avi' ];
%   end
% else
%      moviename = [ movpath, '/', timepath, '_DualArm_TargetMaru_', 'FrameRate', num2str(framerate), '_MovOC.avi' ];
% end
moviename = [ movpath, '/', timepath, '_DualArm_TargetMaru_', 'FrameRate', num2str(framerate), '_Mov.avi' ];
if mannakamode == 1
moviename = [ movpath, '/', timepath, '_DualArm_TargetMaru_', 'FrameRate', num2str(framerate), '_MovOC.avi' ];
end

writerObj = VideoWriter( moviename );
set( writerObj, 'FrameRate', framerate, 'Quality', 100 );
open( writerObj );

r0c = r_tip * cos(for_d_angle); r0s = r_tip * sin(for_d_angle); 
r1c = r_motor * cos(for_d_angle); r1s = r_motor * sin(for_d_angle); 
r2c = r_motor/2 * cos(for_d_angle); r2s = r_motor/2 * sin(for_d_angle); 

if simplemode == 0
    % 追加
    d_rb_j4 = fill( d_pos_j4(1,1) + r2c, d_pos_j4(1,2) + r2s, 'r', 'LineWidth', sen1 ); d_rb_j8 = fill( d_pos_j8(1,1) + r2c, d_pos_j8(1,2) + r2s, 'b', 'LineWidth', sen1 );
    d_eL = fill( xeL(1,1) + r2c*2, xeL(1,2) + r2s*2, 'm', 'LineWidth', sen1 ); d_eR = fill( xeR(1,1) + r2c*2, xeR(1,2) + r2s*2, 'c', 'LineWidth', sen1 );
    td  = fill( td_x(1,1:5), td_y(1,1:5), usui_haiiro, 'LineWidth', sen1 );
    dt1 = fill( d1(1,1) + r2c/2, d1(1,2) + r2s/2, 'g', 'LineWidth', sen1 ); dt2 = fill( d2(1,1) + r2c/2, d2(1,2) + r2s/2, 'k', 'LineWidth', sen1 );
    dt3 = fill( d3(1,1) + r2c/2, d3(1,2) + r2s/2, 'k', 'LineWidth', sen1 ); dt4 = fill( d4(1,1) + r2c/2, d4(1,2) + r2s/2, 'k', 'LineWidth', sen1 );
    QLL2 = plot( QL2x(1,:), QL2y(1,:), 'r', 'LineWidth', sen2 ); QRR6 = plot( QR6x(1,:), QR6y(1,:), 'b', 'LineWidth', sen2 );
    QLL3 = plot( QL3x(1,:), QL3y(1,:), 'r', 'LineWidth', sen2 ); QRR7 = plot( QR7x(1,:), QR7y(1,:), 'b', 'LineWidth', sen2 );
    dQLL3 = plot( dQL3x(1,:), dQL3y(1,:), 'm', 'LineWidth', sen2 ); dQRR7 = plot( dQR7x(1,:), dQR7y(1,:), 'c', 'LineWidth', sen2 );
    N_L1 = plot( N_L1x(1,:), N_L1y(1,:), 'c-', 'linewidth', sen2 ); N_L2 = plot( N_L2x(1,:), N_L2y(1,:), 'c-', 'linewidth', sen2 );
    N_R1 = plot( N_R1x(1,:), N_R1y(1,:), 'c-', 'linewidth', sen2 ); N_R2 = plot( N_R2x(1,:), N_R2y(1,:), 'c-', 'linewidth', sen2 );
    F_L1 = plot( F_L1x(1,:), F_L1y(1,:), 'y-', 'linewidth', sen2 ); F_L2 = plot( F_L2x(1,:), F_L2y(1,:), 'y-', 'linewidth', sen2 );
    F_R1 = plot( F_R1x(1,:), F_R1y(1,:), 'y-', 'linewidth', sen2 ); F_R2 = plot( F_R2x(1,:), F_R2y(1,:), 'y-', 'linewidth', sen2 );
    pha = plot( phase(1,1)*0.1/2, 0.8, 'r*', 'linewidth', 10 );
end

% ロボット
    rb_base  = fill( b_x(1,:), b_y(1,:), orange, 'LineWidth', sen2 );   % ベース四角形頂点を描画
    rb_arm_L = plot( arm_L_x(1,:), arm_L_y(1,:), 'b-', 'linewidth', sen2 ); rb_arm_R = plot( arm_R_x(1,:), arm_R_y(1,:), 'b-', 'linewidth', sen2 );   % 腕関節を描画
    rb_hand_L = plot( hand_L_x(1,:), hand_L_y(1,:), 'b-', 'linewidth', sen2 ); rb_hand_R = plot( hand_R_x(1,:), hand_R_y(1,:), 'b-', 'linewidth', sen2 );   % 腕関節を描画
    rb_j1 = fill( pos_j1(1,1) + r1c, pos_j1(1,2) + r1s, koi_haiiro, 'LineWidth', sen1 ); rb_j2 = fill( pos_j2(1,1) + r1c, pos_j2(1,2) + r1s, koi_haiiro, 'LineWidth', sen1 );
    rb_j3 = fill( pos_j3(1,1) + r1c, pos_j3(1,2) + r1s, koi_haiiro, 'LineWidth', sen1 ); rb_j4 = fill( pos_j4(1,1) + r2c, pos_j4(1,2) + r2s, shiro, 'LineWidth', sen1 );
    rb_j5 = fill( pos_j5(1,1) + r1c, pos_j5(1,2) + r1s, koi_haiiro, 'LineWidth', sen1 ); rb_j6 = fill( pos_j6(1,1) + r1c, pos_j6(1,2) + r1s, koi_haiiro, 'LineWidth', sen1 );
    rb_j7 = fill( pos_j7(1,1) + r1c, pos_j7(1,2) + r1s, koi_haiiro, 'LineWidth', sen1 ); rb_j8 = fill( pos_j8(1,1) + r2c, pos_j8(1,2) + r2s, shiro, 'LineWidth', sen1 );   % 関節のモータ円を100個の点で描画
    % ターゲット
%     tg_basesen = fill( t_x(1,:), t_y(1,:), 'r-', 'LineWidth', sen2 );
    tg_en = fill( pos_tm_geo(1,1) + r_target * cos(for_d_angle), pos_tm_geo(1,2) + r_target * sin(for_d_angle), 'r-', 'LineWidth', sen2 );
 if mannakamode == 1
    % クロージャ
    OC_t1 = fill( pos_eL1(1,1) + (r_target+r_tip) * cos(for_d_angle), pos_eL1(1,2) + (r_target+r_tip) * sin(for_d_angle), green, 'LineWidth', 1 );
    OC_t2 = fill( pos_eL2(1,1) + (r_target+r_tip) * cos(for_d_angle), pos_eL2(1,2) + (r_target+r_tip) * sin(for_d_angle), green, 'LineWidth', 1 );
    OC_t3 = fill( pos_eR1(1,1) + (r_target+r_tip) * cos(for_d_angle), pos_eR1(1,2) + (r_target+r_tip) * sin(for_d_angle), green, 'LineWidth', 1 );
    OC_t4 = fill( pos_eR2(1,1) + (r_target+r_tip) * cos(for_d_angle), pos_eR2(1,2) + (r_target+r_tip) * sin(for_d_angle), green, 'LineWidth', 1 );
 end

    % 重心の扇形
    rb_cross_1 = fill( [ pos_rb(1,1), ( pos_rb(1,1) + r_base * cos( ang_bs(1,1) + for_pi_half ) ),        pos_rb(1,1), ( pos_rb(1,1) + r_base * cos( ang_bs(1,1) + for_pi_half + pi ) ),     pos_rb(1,1) ], ...
                       [ pos_rb(1,2), ( pos_rb(1,2) + r_base * sin( ang_bs(1,1) + for_pi_half ) ),        pos_rb(1,2), ( pos_rb(1,2) + r_base * sin( ang_bs(1,1) + for_pi_half + pi ) ),     pos_rb(1,2) ], ...
                         kuro, 'LineWidth', sen1 );
    rb_cross_2 = fill( [ pos_rb(1,1), ( pos_rb(1,1) + r_base * cos( ang_bs(1,1) + for_pi_half + pi/2 ) ), pos_rb(1,1), ( pos_rb(1,1) + r_base * cos( ang_bs(1,1) + for_pi_half + 3*pi/2 ) ), pos_rb(1,1) ], ...
                       [ pos_rb(1,2), ( pos_rb(1,2) + r_base * sin( ang_bs(1,1) + for_pi_half + pi/2 ) ), pos_rb(1,2), ( pos_rb(1,2) + r_base * sin( ang_bs(1,1) + for_pi_half + 3*pi/2 ) ), pos_rb(1,2) ], ...
                         shiro, 'LineWidth', sen1 );
    tg_cross_1 = fill( [ pos_tm(1,1), ( pos_tm(1,1) + r_base * cos( ang_tr(1,1) + for_pi_half ) ),        pos_tm(1,1), ( pos_tm(1,1) + r_base * cos( ang_tr(1,1) + for_pi_half + pi ) ),     pos_tm(1,1) ], ...
                       [ pos_tm(1,2), ( pos_tm(1,2) + r_base * sin( ang_tr(1,1) + for_pi_half ) ),        pos_tm(1,2), ( pos_tm(1,2) + r_base * sin( ang_tr(1,1) + for_pi_half + pi ) ),     pos_tm(1,2) ], ...
                         kuro, 'LineWidth', sen1 );
    tg_cross_2 = fill( [ pos_tm(1,1), ( pos_tm(1,1) + r_base * cos( ang_tr(1,1) + for_pi_half + pi/2 ) ), pos_tm(1,1), ( pos_tm(1,1) + r_base * cos( ang_tr(1,1) + for_pi_half + 3*pi/2 ) ), pos_tm(1,1) ], ...
                       [ pos_tm(1,2), ( pos_tm(1,2) + r_base * sin( ang_tr(1,1) + for_pi_half + pi/2 ) ), pos_tm(1,2), ( pos_tm(1,2) + r_base * sin( ang_tr(1,1) + for_pi_half + 3*pi/2 ) ), pos_tm(1,2) ], ...
                         shiro, 'LineWidth', sen1 );
    rb_tip_L1 = fill( pos_eL1(1,1) + r0c, pos_eL1(1,2) + r0s, usui_ao, 'LineWidth', sen1 ); rb_tip_L2 = fill( pos_eL2(1,1) + r0c, pos_eL2(1,2) + r0s, usui_ao, 'LineWidth', sen1 );   % 左手先球の円を100個の点で描画
    rb_tip_R1 = fill( pos_eR1(1,1) + r0c, pos_eR1(1,2) + r0s, usui_ao, 'LineWidth', sen1 ); rb_tip_R2 = fill( pos_eR2(1,1) + r0c, pos_eR2(1,2) + r0s, usui_ao, 'LineWidth', sen1 );   % 左手先球の円を100個の点で描画
    tg_geo = fill( pos_tm_geo(1,1) + r0c/2, pos_tm_geo(1,2) + r0s/2, 'y-', 'LineWidth', sen2 );


if simplemode == 0
    % 推定情報
    obs1 = fill( obs1_tr(1,1) + r0c/2, obs1_tr(1,2) + r0s/2, 'g', 'LineWidth', sen1 );
    obs2 = fill( obs2_tr(1,1) + r0c/2, obs2_tr(1,2) + r0s/2, kuro, 'LineWidth', sen1 );
    tg_estcom = fill( pos_est(1,1) + r_tip * sin(for_d_angle), pos_est(1,2) + r_tip * cos(for_d_angle), 'y', 'LineWidth', sen1 );
    d_en_geo = fill( d_geo(1,1) + r0c, d_geo(1,2) + r0s, 'g-', 'LineWidth', sen1 );
    tg_en_geo = fill( pos_tm_geo(1,1) + r0c/2, pos_tm_geo(1,2) + r0s/2, 'w-', 'LineWidth', sen1 );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% EraseMode %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set( rb_base, 'EraseMode', 'normal' );
    set( [ rb_arm_L, rb_arm_R ], 'EraseMode', 'normal' );
    set( [ rb_hand_L, rb_hand_R ], 'EraseMode', 'normal' );
    set( [ rb_tip_L1, rb_tip_L2, rb_tip_R1, rb_tip_R2 ], 'EraseMode', 'normal' );
    set( [ rb_j1, rb_j2, rb_j3, rb_j4, rb_j5, rb_j6, rb_j7, rb_j8 ], 'EraseMode', 'normal' );
%     set( tg_basesen, 'EraseMode', 'normal' );
    set( tg_en, 'EraseMode', 'normal' );
    set( tg_geo, 'EraseMode', 'normal' );
    set( [ rb_cross_1, rb_cross_2, tg_cross_1, tg_cross_2 ], 'EraseMode', 'normal' );
if simplemode == 0;
    set( tg_en_geo, 'EraseMode', 'normal' );
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

% % 初期図を保存
% figure(n_figure)
% title( 'Time = 0.000 [s]', 'FontName', 'Times New Roman', 'FontSize', FST );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetMaru_1stFig.fig' ] );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetMaru_1stFig.png' ] );

%%%%%%%%%% 図を更新 %%%%%%%%%%
i = 1;
while i <= length( mat_15(:,1) )

if mvtime(i) > 10
break
end

    if rem( mvtime(i), movie_d_time ) == 0   % mvtime(i)をmovie_d_timeで除算した後の剰余が0のとき(割り切れるとき)

if simplemode == 0
     % 追加
       set( N_L1, 'XData', N_L1x(i,:), 'YData', N_L1y(i,:) ); set( N_L2, 'XData', N_L2x(i,:), 'YData', N_L2y(i,:) );
       set( N_R1, 'XData', N_R1x(i,:), 'YData', N_R1y(i,:) ); set( N_R2, 'XData', N_R2x(i,:), 'YData', N_R2y(i,:) );
       set( F_L1, 'XData', F_L1x(i,:), 'YData', F_L1y(i,:) ); set( F_L2, 'XData', F_L2x(i,:), 'YData', F_L2y(i,:) );
       set( F_R1, 'XData', F_R1x(i,:), 'YData', F_R1y(i,:) ); set( F_R2, 'XData', F_R2x(i,:), 'YData', F_R2y(i,:) );
       set( d_rb_j4, 'XData', d_pos_j4(i,1) + r2c, 'YData', d_pos_j4(i,2) + r2s ); set( d_rb_j8, 'XData', d_pos_j8(i,1) + r2c, 'YData', d_pos_j8(i,2) + r2s );
       set( d_eL, 'XData', xeL(i,1) + r2c*2, 'YData', xeL(i,2) + r2s*2 ); set( d_eR, 'XData', xeR(i,1) + r2c*2, 'YData', xeR(i,2) + r2s*2 );
       set( dt1, 'XData', d1(i,1) + r2c/2, 'YData', d1(i,2) + r2s/2 ); set( dt2, 'XData', d2(i,1) + r2c/2, 'YData', d2(i,2) + r2s/2 );
       set( dt3, 'XData', d3(i,1) + r2c/2, 'YData', d3(i,2) + r2s/2 ); set( dt4, 'XData', d4(i,1) + r2c/2, 'YData', d4(i,2) + r2s/2 );
       set( QLL2, 'XData', QL2x(i,:), 'YData', QL2y(i,:) ); set( QRR6, 'XData', QR6x(i,:), 'YData', QR6y(i,:) );
       set( QLL3, 'XData', QL3x(i,:), 'YData', QL3y(i,:) ); set( QRR7, 'XData', QR7x(i,:), 'YData', QR7y(i,:) );
       set( dQLL3, 'XData', dQL3x(i,:), 'YData', dQL3y(i,:) ); set( dQRR7, 'XData', dQR7x(i,:), 'YData', dQR7y(i,:) );
       set( td, 'XData', td_x(i,:), 'YData', td_y(i,:) );
       set( pha, 'XData', phase(i,1)*0.1/2, 'YData', 0.8 );
end

     % ロボット
       set( rb_base,  'XData', b_x(i,:), 'YData', b_y(i,:) );
       set( rb_arm_L, 'XData', arm_L_x(i,:), 'YData', arm_L_y(i,:) ); set( rb_arm_R, 'XData', arm_R_x(i,:), 'YData', arm_R_y(i,:) );
       set( rb_hand_L, 'XData', hand_L_x(i,:), 'YData', hand_L_y(i,:) ); set( rb_hand_R, 'XData', hand_R_x(i,:), 'YData', hand_R_y(i,:) );
       set( rb_j1, 'XData', pos_j1(i,1) + r1c, 'YData', pos_j1(i,2) + r1s ); set( rb_j2, 'XData', pos_j2(i,1) + r1c, 'YData', pos_j2(i,2) + r1s );
       set( rb_j3, 'XData', pos_j3(i,1) + r1c, 'YData', pos_j3(i,2) + r1s ); set( rb_j4, 'XData', pos_j4(i,1) + r2c, 'YData', pos_j4(i,2) + r2s );
       set( rb_j5, 'XData', pos_j5(i,1) + r1c, 'YData', pos_j5(i,2) + r1s ); set( rb_j6, 'XData', pos_j6(i,1) + r1c, 'YData', pos_j6(i,2) + r1s );
       set( rb_j7, 'XData', pos_j7(i,1) + r1c, 'YData', pos_j7(i,2) + r1s ); set( rb_j8, 'XData', pos_j8(i,1) + r2c, 'YData', pos_j8(i,2) + r2s );
       set( rb_cross_1, 'XData', [ pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half ) ),        pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half + pi ) ),     pos_rb(i,1) ], ...
                        'YData', [ pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half ) ),        pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half + pi ) ),     pos_rb(i,2) ] );
       set( rb_cross_2, 'XData', [ pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half + pi/2 ) ), pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half + 3*pi/2 ) ), pos_rb(i,1) ], ...
                        'YData', [ pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half + pi/2 ) ), pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half + 3*pi/2 ) ), pos_rb(i,2) ] );
     % ターゲット
%        set( tg_basesen, 'XData', t_x(i,:), 'YData', t_y(i,:) );
       set( tg_en, 'XData', pos_tm_geo(i,1) + r_target * cos(for_d_angle), 'YData', pos_tm_geo(i,2) + r_target * sin(for_d_angle) );
 if mannakamode == 1
    % クロージャ
       set( OC_t1, 'XData', pos_eL1(i,1) + (r_target+r_tip) * cos(for_d_angle), 'YData', pos_eL1(i,2) + (r_target+r_tip) * sin(for_d_angle) );
       set( OC_t2, 'XData', pos_eL2(i,1) + (r_target+r_tip) * cos(for_d_angle), 'YData', pos_eL2(i,2) + (r_target+r_tip) * sin(for_d_angle) );
       set( OC_t3, 'XData', pos_eR1(i,1) + (r_target+r_tip) * cos(for_d_angle), 'YData', pos_eR1(i,2) + (r_target+r_tip) * sin(for_d_angle) );
       set( OC_t4, 'XData', pos_eR2(i,1) + (r_target+r_tip) * cos(for_d_angle), 'YData', pos_eR2(i,2) + (r_target+r_tip) * sin(for_d_angle) );
 end

       set( tg_cross_1, 'XData', [ pos_tm(i,1), ( pos_tm(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half ) ),        pos_tm(i,1), ( pos_tm(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half + pi ) ),     pos_tm(i,1) ], ...
                        'YData', [ pos_tm(i,2), ( pos_tm(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half ) ),        pos_tm(i,2), ( pos_tm(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half + pi ) ),     pos_tm(i,2) ] );
       set( tg_cross_2, 'XData', [ pos_tm(i,1), ( pos_tm(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half + pi/2 ) ), pos_tm(i,1), ( pos_tm(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half + 3*pi/2 ) ), pos_tm(i,1) ], ...
                        'YData', [ pos_tm(i,2), ( pos_tm(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half + pi/2 ) ), pos_tm(i,2), ( pos_tm(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half + 3*pi/2 ) ), pos_tm(i,2) ] );
       set( rb_tip_L1, 'XData', pos_eL1(i,1) + r0c, 'YData', pos_eL1(i,2) + r0s ); set( rb_tip_L2, 'XData', pos_eL2(i,1) + r0c, 'YData', pos_eL2(i,2) + r0s );
       set( rb_tip_R1, 'XData', pos_eR1(i,1) + r0c, 'YData', pos_eR1(i,2) + r0s ); set( rb_tip_R2, 'XData', pos_eR2(i,1) + r0c, 'YData', pos_eR2(i,2) + r0s );
       set( tg_geo, 'XData', pos_tm_geo(i,1) + r0c/2, 'YData', pos_tm_geo(i,2) + r0s/2 );

if simplemode == 0
     % 推定情報
       set( obs1, 'XData', obs1_tr(i,1) + r0c/2, 'YData', obs1_tr(i,2) + r0s/2 );
       set( obs2, 'XData', obs2_tr(i,1) + r0c/2, 'YData', obs2_tr(i,2) + r0s/2 );
       set( tg_estcom, 'XData', pos_est(i,1) + r_tip * sin(for_d_angle), 'YData', pos_est(i,2) + r_tip * cos(for_d_angle) );
       set( d_en_geo, 'XData', d_geo(i,1) + r0c, 'YData', d_geo(i,2) + r0s );
       set( tg_en_geo, 'XData', pos_tm_geo(i,1) + r0c/2, 'YData', pos_tm_geo(i,2) + r0s/2 );
end

       title( sprintf( 'Time = %0.3f [s]', mat_15(i,1) ), 'FontName', 'Times New Roman', 'FontSize', FST );
       if mannakamode == 1
          xlim( [ pos_tm_geo(i,1)-mannaka, pos_tm_geo(i,1)+mannaka ] ); ylim( [ pos_tm_geo(i,2)-mannaka, pos_tm_geo(i,2)+mannaka ] );
       end
       F( frame_n ) = getframe( n_figure );
       hold off;
       drawnow;
       writeVideo( writerObj, F( frame_n ) );
       frame_n = frame_n + 1;

    end 

    i = i + 1;
%{
    if rem( mvtime(i), movie_d_time * 5 ) == 0   % mvtime(i)をmovie_d_timeで除算した後の剰余が0のとき(割り切れるとき)
        % 図を保存
        figure(n_figure)
        title( sprintf( 'Time = %0.3f [s]', mvtime(i,1) ), 'FontName', 'Times New Roman', 'FontSize', FST );
        saveas( figure(n_figure), [ movpath, '\', timepath, '_DualArm_TargetMaru_TrackingContact_', num2str(mvtime(i)*100), 'Figure.fig' ] );
        saveas( figure(n_figure), [ movpath, '\', timepath, '_DualArm_TargetMaru_TrackingContact_', num2str(mvtime(i)*100), 'Figure.png' ] );
    end
%}
end

% % 最後の図を保存
% figure(n_figure)
% title( sprintf( 'Time = %0.3f [s]', mat_15(end,1) ), 'FontName', 'Times New Roman', 'FontSize', FST );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetMaru_LastFig.fig' ] );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetMaru_LastFig.png' ] );
% % n_figure = n_figure + 1;

close( writerObj );

%}

%}   % 動画作成終了
close all