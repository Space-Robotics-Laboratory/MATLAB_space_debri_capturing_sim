%%%%%%%%%% SpaceDyn サンプルプログラム  3リンク双腕宇宙ロボットの接触シミュレーション(手先速度制御＋ベースフリー) ターゲット四角型
%%%%%%%%%% 結果表示(グラフ描画，動画作成)あり
%%%%%%%%%% DualArm_TargetShikaku_Tracking_Contact  2018年8月6日 長谷作成
%%%%%%%%%% 変数名 : ～1 → leftarm, ～2 → rightarm
%%%%%%%%%% DesiredHandMotion_v05.m 編集 2021年11月1日 阿部光輝
%%%%%%%%%% 時間によるif文でPhaseを定義していたが, switch文に変更
%%%%%%%%%% Phase4:接触後キープPhase, Phase5:目標手先位置の計算, Phase6:手先制御, Phase7,8:キープ
clc
clear all
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% global 変数の定義 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 全ての関数及びメインルーチン内で共通で使用される変数
global d_time
global Gravity
global Ez
Ez = [ 0 0 1 ]';
d_time = 0.001; % シミュレーション1step当たりの時間
Gravity = [ 0 0 0 ]'; % 重力（地球重力は Gravity = [0 0 -9.8]）

for ts_X_i =    0;      % tsはターゲットを意味する
for ts_Y_i =    0;
for ts_Vx_i =   0;
for ts_Vy_i =   0;
for ts_Deg_i =  0;
for ts_W_i =   2;%:2:3;
for Theta_h_i = 0;
for Kh_i = 0;
for Vh_ini_i = 0;
for ts_Error_Length_i = 0;%:0.25:0.5%;   % 3      % 正方形幾何中心から，正方形対角線の半分の何割の距離に重心があるか0～1
for ts_Error_Theta_i = 45%:45:360;%220:20:360;%20:20:180%360%;   % 5

zeta = [ d_time, 10, Theta_h_i, ts_X_i, ts_Y_i, ts_W_i, Vh_ini_i, Kh_i, ts_Deg_i, ts_Error_Length_i, ts_Error_Theta_i, ts_Vx_i, ts_Vy_i ]';

    qqq = 0;%-pi/7;
% 分解加速度制御ゲイン
    Kp = 20000;%10000;   % 20000;
    Kd = 500;%1000;   % 500;
%     Kp_2 = 1;3;   % 0.8
%     Kd_2 = 0.3;   % 0.3

minus_time = 2;10*d_time;
minus_time_2 = 0;
minus_time_3 = 0;
catchlimit = 20;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ストップウォッチON %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% シミュレーションにかかる時間の計測
startT = clock();
startCPUT = cputime;
endtime = zeta(2,1);

    conma = ',';
    d_Time_name = [ 'dt=', num2str( zeta(1,1) ) ];      % 刻み時間はzetaの第一引数d_time
    EndTime_name = [ 'et=', num2str( zeta(2,1) ) ];     % シミュレーション終了時間
    tsX_name = [ 'Xts=', num2str( ts_X_i ) ];           % ターゲットのx座標   
    tsY_name = [ 'Yts=', num2str( ts_Y_i ) ];           % ターゲットのy座標
    tsVx_name = [ 'Vxts=', num2str( ts_Vx_i ) ];        % ターゲットのx速度
    tsVy_name = [ 'Vyts=', num2str( ts_Vy_i ) ];        % ターゲットのy速度
    tsDeg_name = [ 'THts=', num2str( ts_Deg_i ) ];      % ターゲットの回転角
    tsW_name = [ 'Wts=', num2str( ts_W_i ) ];           % ターゲットの角速度
    tsErLe_name = [ 'ErL=', num2str( ts_Error_Length_i ) ]; % 正方形幾何中心から，正方形対角線の半分の何割の距離に重心があるか0～1
    tsErTH_name = [ 'ErTH=', num2str( ts_Error_Theta_i ) ]; % 何か分からんがとりあえず45度
%     Theta_h_name = [ 'THh=', num2str( Theta_h_i ) ];
%     Kh_name = [ 'Kh=', num2str( Kh_i ) ];
%     Vh_ini_name = [ 'Vh=', num2str( Vh_ini_i ) ];

zetaname = [ d_Time_name, conma, EndTime_name, conma, tsX_name, conma, tsY_name, conma, ...
             tsVx_name, conma, tsVy_name, conma, tsDeg_name, conma, tsW_name, conma, ...
             tsErLe_name, conma, tsErTH_name ]; % 生成されるファイルの名前

u = '.';
s =  '%s\t';
se = '%s\r\n';
g =  '%g\t';
ge = '%g\n';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ロボット・ターゲットLP設定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ロボットリンクパラメータ
LP_d = DualArm_FourTips_LP_v3();   
SV_d = DualArm_FourTips_SV_v3( LP_d );
% ターゲットリンクパラメータ
LP_ts = TargetShikaku_FourTips_LP();
SV_ts = TargetShikaku_FourTips_SV( LP_ts );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% フォルダパス指定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% フォルダ作成
timepath = datestr( now, 'yyyy-mmdd-HHMMSS' );  % yはyear，mはmonth,dはday,Hはhour,Mはminute,Sはsecond.それぞれの文字数分出力する
datepath = [ 'C:/Users/baske/OneDrive/デスクトップ/' datestr( now, 'yyyy-mmdd' ) '/' timepath, '_' ];  % データ保存のディレクトリを作る
% path = [ datepath, '-Shikaku_', zetaname ];
path = [ datepath, zetaname ];          % パスを引く
datfile = [ path, '/', timepath, '-', 'dat' ];      % datファイルの名前指定
figfile = [ path, '/', timepath, '-', 'fig' ];      % グラフファイルの名前指定
pngfile = [ path, '/', timepath, '-', 'png' ];      % 画像ファイルの名前指定
movfile = [ path, '/', timepath, '-', 'mov' ];      % 動画ファイルの名前指定
mkdir( datfile ); mkdir( figfile ); mkdir( pngfile ); mkdir( movfile ); % それぞれのディレクトリを作る命令

% パス設定
datpath = [ path, '/', timepath, '-', 'dat' ];      % パスを作る
figpath = [ path, '/', timepath, '-', 'fig' ];
pngpath = [ path, '/', timepath, '-', 'png' ];
movpath = [ path, '/', timepath, '-', 'mov' ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 係数設定 %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 剛性係数，粘性係数設定
kw_1 = 1000;900; % 980.0;       % 剛性係数.誰が決めたんやこれ
cw_1 = 20; % 8.7;              % 粘性係数.これも誰が決めた?
% cw = 100.7;
kw_2 =  0.0;                    % 剛性係数が0とは?
cw_2 =  0.0;                    % 粘性係数が0とは?
kkk = kw_1;
ccc = cw_1;
% 摩擦係数固定値
cof = 0.3;   % 摩擦係数0.1で固定?

% 図番号
n_figure = 999;

% 剛性係数，粘性係数，摩擦係数可変値   % 石井さん摩擦データから
% なんでこいつら要素数5なん？
theta_contact = [ deg2rad(0)  deg2rad(10)  deg2rad(20)  deg2rad(30)  deg2rad(40) ]; % degreeをradianに変換
kk = [ 900  920  980  1057  1230 ];
cc = [ 6.0  6.8  8.7  10.9  15.5 ];
myu = [ 0.0005  0.112  0.185  0.238  0.298 ];
k_p = polyfit( theta_contact, kk, 3 );   % 最小二乗法 kのデータに対して最小二乗的に最適な近似となる3次多項式k_p(θ)の係数を返す
c_p = polyfit( theta_contact, cc, 3 );   % polyfitはmathworksが提供している関数.スムージングをかける
myu_p = polyfit( theta_contact, myu, 3 );

theta_h = deg2rad( zeta(3,1) );   % 追従角度Theta_h_i　degをradに変換

w_dtots = zeros(3,1);
J_3_G_tmp = zeros(12,8);
d_ten1 = zeros(3,1);
d_ten2 = zeros(3,1);
d_ten3 = zeros(3,1);
d_ten4 = zeros(3,1);
% スプライン曲線係数
% ロボットの軌跡を近似的に算出するのがスプライン曲線
aL = zeros(2,4);
aR = zeros(2,4);
x_L = zeros(2,1);
x_R = zeros(2,1);
d_QL_3 = pi/2;
d_QR_7 = pi/2;
delta_t = 0;
stepn = 5;
t0 = d_time * stepn;
t1 = 0;
% q_joints_kotei = SV_d.q;
flag_kotei = 0;
Phase = 1;
flagphase5 = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 長さパラメータ設定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r_tip = 0.01; %チェイサ先端球半径 1cm
DHD = 0.113;%ターゲット対角の長さの半分 11.3cm
side_target = 0.15;  %ターゲット1辺 15cm 
half_side = side_target / 2;    % 1辺の半分
base_yoko = 0.32;   % ベース四角形横 32cm
base_tate = 0.15;   % ベース四角形縦 15cm
base_jushin_teihen = 0.09754;   % ベース重心から底辺までの長さ 9.754cm
base_arm_yoko_1 = abs( LP_d.c0(1,1) );  %abs()は絶対値を返す関数
base_arm_yoko_2 = abs( LP_d.c0(1,4) );
base_arm_tate_1 = abs( LP_d.c0(2,1) );
base_arm_tate_2 = abs( LP_d.c0(2,4) );
r_motor = 0.033 / 2;   % モータ半径 0.033/2cm
r_base = 0.03;   % ベースの重心描画用円
r_airtank = 0.096 / 2;   % エアタンク半径 0.096/2cm
base_airtank_yoko = base_yoko/2 + 0.108 - r_airtank/2;   % ベース重心からエアタンク中心までの横距離 エアタンクの高さが10.8cm
base_airtank_tate = base_jushin_teihen - base_tate/2;   % ベース重心からエアタンク中心までの縦距離

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 初期値設定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% ロボット初期値設定 %%%%%%%%%%
% ベースからnum_eで指定された手先までを結ぶ関節(リンク)を求める  1アーム多リンクなら1, リンクの数を表す
num_eL = 1;
jointsL = j_num( LP_d, num_eL );   %左手のジョイント数
num_eR = 2;
jointsR = j_num( LP_d, num_eR );   %右手のジョイント数

% ベースの初期位置・姿勢・速度・角速度
SV_d.R0 = [ 0 0 0 ]';           % 初期位置
SV_d.Q0 = [ 0 0 deg2rad( 0 ) ]';% 初期姿勢
SV_d.A0 = rpy2dc( SV_d.Q0 )';   % 初期姿勢から方向余弦行列を算出
SV_d.v0 = [ 0 0 0 ]';           % 初期並進速度
SV_d.w0 = [ 0 0 0 ]';           % 初期角速度
SV_d = calc_aa( LP_d, SV_d );   % 各リンクの座標返還行列(方向余弦行列)の計算(リンクi->慣性座標系)
SV_d = calc_pos( LP_d, SV_d );  % 各リンク重心位置の計算
% そっかSVは構造体的なやつやから計算してほしい部分だけ計算してくれるんか
% てかSV_dとかになってないんやけど.SVのまんまやねんけど
% ->変更しました

% % ロボットの初期関節角度を設定
q_L = [ pi/3+qqq  -pi*4/9-qqq  -pi*7/18  0 ]';    % qqq = 0
q_R = -q_L;   % 右側
SV_d.q = [ q_L' q_R' ]';    % なんでこんなに転置ばっかすんの?
q_joints_kotei = zeros(8,1);    % 関節角度を一時的に格納しておく行列...かな?
%kotei1 = pi/16;                 % これなんですの?
%kotei2 = pi/6;                  % これもなんですの?
%q_joints_kotei(1:3,1) = [ pi/3-kotei1  -pi*4/9+kotei1-kotei2  -pi*7/18+kotei2 ]';  %[ pi/3+(-pi/7*1.2) -pi/2-(-pi/7*1.2) -pi/3 ]';わっかんね
%q_joints_kotei(5:7,1) = -q_joints_kotei(1:3,1); % q_joint_koteiの4行目と8行目がゼロのまんまじゃね?こいつらは受動関節やからか

%%%%%%%%%% 初期手先位置・姿勢の計算 %%%%%%%%%%
% 順動力学計算
SV_d =  f_dyn_rk2( LP_d, SV_d );    % ロボットに関する順動力学
SV_ts = f_dyn_rk2( LP_ts, SV_ts );  % ターゲットに関する順動力学
% 各リンクの座標変換行列(方向余弦行列)の計算（リンクi → 慣性座標系）
 SV_d = calc_aa( LP_d, SV_d );   % これさっきもやったやん(193行目)
% 各リンク重心の位置
 SV_d = calc_pos( LP_d, SV_d );
% 全関節位置と姿勢の計算
[ POS_j_L, ORI_j_L ] = f_kin_j( LP_d, SV_d, jointsL );   % 左手 jointsLは左手の関節数
[ POS_j_R, ORI_j_R ] = f_kin_j( LP_d, SV_d, jointsR );   % 右手
% 各関節の位置 x,y
POS_j1 = POS_j_L( 1:2, 1 );   % 関節1の位置代入
POS_j2 = POS_j_L( 1:2, 2 );   
POS_j3 = POS_j_L( 1:2, 3 );
POS_j4 = POS_j_L( 1:2, 4 );
POS_j4_tmp = POS_j4;    
d_POS_j4 = [ POS_j4' 0 ]';
POS_j5 = POS_j_R( 1:2, 1 );
POS_j6 = POS_j_R( 1:2, 2 );
POS_j7 = POS_j_R( 1:2, 3 );
POS_j8 = POS_j_R( 1:2, 4 );
POS_j8_tmp = POS_j8;
d_POS_j8 = [ POS_j8' 0 ]';

% 手先位置姿勢の計算と手先角度のオイラー角表現
L = 0.07;   %
theta = pi/5;
D = abs( L * ( cos( theta ) - sin( theta ) ) + r_tip * sqrt(2) );   % 手先が丁度ターゲットに触れるような長さを計算
D_0 = abs( L * cos( theta ) + r_tip );
[ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % 左手
Qe_radL1 = dc2rpy( ORI_eL' );   %左側
Qe_radL2 = dc2rpy( ORI_eL' );   %左側
POS_eL1 = [ POS_j4' 0 ]' + rpy2dc([ 0 0  theta ])' * ( POS_eL - [ POS_j4' 0 ]' ) * L / norm( LP_d.cc( :, 4, 4 ) );
POS_eL2 = [ POS_j4' 0 ]' + rpy2dc([ 0 0 -theta ])' * ( POS_eL - [ POS_j4' 0 ]' ) * L / norm( LP_d.cc( :, 4, 4 ) );
[ POS_eR, ORI_eR ] = f_kin_e( LP_d, SV_d, jointsR );   % 右手
Qe_radR1 = dc2rpy( ORI_eR' );   %右側
Qe_radR2 = dc2rpy( ORI_eR' );   %右側
POS_eR1 = [ POS_j8' 0 ]' + rpy2dc([ 0 0  theta ])' * ( POS_eR - [ POS_j8' 0 ]' ) * L / norm( LP_d.cc( :, 8, 8 ) );
POS_eR2 = [ POS_j8' 0 ]' + rpy2dc([ 0 0 -theta ])' * ( POS_eR - [ POS_j8' 0 ]' ) * L / norm( LP_d.cc( :, 8, 8 ) );
%%%%%%%%%% ターゲット初期値設定 %%%%%%%%%%
% ターゲットの姿勢・速度・角速度
SV_ts.v0 = [ zeta(12,1) zeta(13,1) 0 ]';   % ターゲット初期並進速度
SV_ts.w0 = [ 0 0 zeta(6,1) ]';   % ターゲット初期角速度
SV_ts_Q0_0 = [ 0 0 deg2rad( zeta(9,1) ) ]';   % time=0のときのターゲットの姿勢
SV_ts.Q0 = SV_ts_Q0_0 - [ 0 0 minus_time * SV_ts.w0(3,1) ]';   % time=-minus_timeのときのターゲット姿勢
SV_ts.A0 = rpy2dc( SV_ts.Q0 )'; % そのときの方向余弦行列

% ターゲットの初期位置   ts_dammy_R0を正方形中心とする．
ts_geo_0 = [ zeta(4,1) ( POS_eL1(2,1) + POS_eL2(2,1) ) / 2 + zeta(5,1) 0 ]';
%ts_geo_0 = [ zeta(4,1) POS_eL2(2,1) 0 ]';
%error_tsR0 = half_side * sqrt(2) * zeta(10,1);   % 円の中心がSV_ts.R0から error_R0[m] の位置にある
%theta_tsR0 = deg2rad( zeta(11,1) );   % 円の中心がSV_ts.R0から theta_R0[deg] の位置にある
%SV_ts_R0_0(1,1) = ts_geo_0(1,1) - error_tsR0 * cos( theta_tsR0 + SV_ts_Q0_0(3,1) );   % time=0のときの重心位置
SV_ts_R0_0(1,1) = ts_geo_0(1,1);
%SV_ts_R0_0(2,1) = ts_geo_0(2,1) - error_tsR0 * sin( theta_tsR0 + SV_ts_Q0_0(3,1) );
SV_ts_R0_0(2,1) = ts_geo_0(2,1);
SV_ts_R0_0(3,1) = ts_geo_0(3,1);
SV_ts.R0 = SV_ts_R0_0 - minus_time * SV_ts.v0;   % time=-minus_timeのときの初期重心位置
%ts_geo(1,1) = SV_ts.R0(1,1) + error_tsR0 * cos( theta_tsR0 + SV_ts.Q0(3,1) );
ts_geo(1,1) = SV_ts.R0(1,1);
%ts_geo(2,1) = SV_ts.R0(2,1) + error_tsR0 * sin( theta_tsR0 + SV_ts.Q0(3,1) );
ts_geo(2,1) = SV_ts.R0(2,1);
ts_geo(3,1) = SV_ts.R0(3,1);

% ターゲットの初期観測点位置
r_Obs1 = half_side * 5/5; r_Obs2 = half_side * 5/5;
LeftUp = 3*pi/4;   LeftDown = -3*pi/4;
RightUp = pi/4;    RightDown = -pi/4;
kansoku1 = LeftUp; 
kansoku2 = LeftDown;

Obs1 = ts_geo + sqrt(2) * r_Obs1 * [ cos( kansoku1 + SV_ts.Q0(3,1) ) sin( kansoku1 + SV_ts.Q0(3,1) ) 0 ]';   % 初期shikaku1と同じ点
Obs2 = ts_geo + sqrt(2) * r_Obs2 * [ cos( kansoku2 + SV_ts.Q0(3,1) ) sin( kansoku2 + SV_ts.Q0(3,1) ) 0 ]';   % 初期shikaku2と同じ点
Obs1_tmp = zeros(3,1); Obs2_tmp = zeros(3,1);

% 推定
est_P = ts_geo;
est_V = zeros(3,1);
est_Q = zeros(3,1);
est_Q_ini = zeros(3,1);
est_W = zeros(3,1);
est_r1 = 0; est_r2 = 0;
est_phi1 = 0; est_phi2 = 0;
est_R12 = 0;
est_Theta12 = 0;

est_geo = Obs1 + sqrt(1/2) * rpy2dc( 0, 0, -pi/4 ) * ( Obs2 - Obs1 );
est_half_side = sqrt( ( Obs1(1,1) - Obs2(1,1) )^2 + ( Obs1(2,1) - Obs2(2,1) )^2 ) / 2;
est_q = atan( abs( Obs1(2,1) - Obs2(2,1) ) / abs( Obs1(2,1) - Obs2(2,1) ) );
est_Q_ini(3,1) = est_q;
est_erP = zeros(3,1);

% %%% 軌跡の接線の傾き
% at_ten1_traj = ( Obs1(2,1) - Obs1_tmp(2,1) ) / ( Obs1(1,1) - Obs1_tmp(1,1) );
% at_ten2_traj = ( Obs2(2,1) - Obs2_tmp(2,1) ) / ( Obs2(1,1) - Obs2_tmp(1,1) );
% %%% 軌跡の法線の傾きと切片(tenとten_tmpの中点を通る法線)
% an_ten1_traj = -1 / at_ten1_traj;
% an_ten2_traj = -1 / at_ten2_traj;
% bn_ten1_traj = ( Obs1(2,1) + Obs1_tmp(2,1) ) / 2 - an_ten1_traj * ( Obs1(1,1) + Obs1_tmp(1,1) ) / 2;
% bn_ten2_traj = ( Obs2(2,1) + Obs2_tmp(2,1) ) / 2 - an_ten2_traj * ( Obs2(1,1) + Obs2_tmp(1,1) ) / 2;
% %%% 法線の交点の座標を求める 直線1,2の交点
% NorTraj_12(1,1) = -( bn_ten2_traj - bn_ten1_traj ) / ( an_ten2_traj - an_ten1_traj );
% NorTraj_12(2,1) = ( an_ten2_traj * bn_ten1_traj - an_ten1_traj * bn_ten2_traj ) / ( an_ten2_traj - an_ten1_traj );
% NorTraj_12_tmp = zeros(3,1);

%%% 軌跡の接線の傾き
at_ten1_traj = 0; at_ten2_traj = 0;
%%% 軌跡の法線の傾きと切片(tenとten_tmpの中点を通る法線)
an_ten1_traj = 0; an_ten2_traj = 0; bn_ten1_traj = 0; bn_ten2_traj = 0;
%%% 法線の交点の座標を求める 直線1,2の交点
NorTraj_12 = zeros(3,1); NorTraj_12_tmp = zeros(3,1);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%% 質量の設定 %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % ロボット全質量
% w_d = LP_d.mass;
% 
% % ターゲット質量(推定値)
% mtm_est = LP_ts.mass - 10; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 変数初期化 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 法線接線単位ベクトル初期化
norm_L1 = zeros(3,1); norm_L2 = zeros(3,1); norm_R1 = zeros(3,1); norm_R2 = zeros(3,1);
tang_L1 = zeros(3,1);tang_L2 = zeros(3,1); tang_R1 = zeros(3,1); tang_R2 = zeros(3,1);

% 接触位置初期化
PointC_L1 = zeros(3,1); PointC_L2 = zeros(3,1); PointC_R1 = zeros(3,1); PointC_R2 = zeros(3,1);
curPosAP3_L1 = zeros(3,1); curPosAP3_L2 = zeros(3,1); curPosAP3_R1 = zeros(3,1); curPosAP3_R2 = zeros(3,1);
curPosBP3_L1 = zeros(3,1); curPosBP3_L2 = zeros(3,1); curPosBP3_R1 = zeros(3,1); curPosBP3_R2 = zeros(3,1);
curPosAP3_tmp_L1 = zeros(3,1); curPosAP3_tmp_L2 = zeros(3,1); curPosAP3_tmp_R1 = zeros(3,1); curPosAP3_tmp_R2 = zeros(3,1);
curPosAP3_vel_L1 = zeros(3,1); curPosAP3_vel_L2 = zeros(3,1); curPosAP3_vel_R1 = zeros(3,1); curPosAP3_vel_R2 = zeros(3,1);

% 方向余弦行列初期化
TB0_L1_dash = zeros(3,3); TB0_L2_dash = zeros(3,3); TB0_R1_dash = zeros(3,3); TB0_R2_dash = zeros(3,3);

% めり込み量初期化
delta_tmp_L1 = 0; delta_tmp_L2 = 0; delta_tmp_R1 = 0; delta_tmp_R2 = 0;

%接触力初期化
F_NN_L1 = 0; F_NN_L2 = 0; F_NN_R1 = 0; F_NN_R2 = 0;
FR_N_L1 = zeros(3,1); FR_N_L2 = zeros(3,1); FR_N_R1 = zeros(3,1); FR_N_R2 = zeros(3,1);
FR_T_L1 = zeros(3,1); FR_T_L2 = zeros(3,1); FR_T_R1 = zeros(3,1); FR_T_R2 = zeros(3,1);
FR_L1 = zeros(3,1); FR_L2 = zeros(3,1); FR_R1 = zeros(3,1); FR_R2 = zeros(3,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 関節制御で使うパラメータの定義 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vh0 = zeta(7,1);   % 初期の手先速度(両側から挟み込む)
veL_x0 = +vh0;     % 左手先x成分 
veR_x0 = -vh0;     % 右手先x成分
veL_y0 = 0;        % 左手先y成分
veR_y0 = 0;        % 右手先y成分

veL_x = veL_x0;
veR_x = veR_x0;
veL_y = veL_y0;
veR_y = veR_y0;
veL_des = [ veL_x veL_y 0 0 0 0 ]';
veR_des = [ veR_x veR_y 0 0 0 0 ]';
ve_des =  [ veL_des' veR_des' ]';

% 関節指令値
q_des =   SV_d.q;   % 指令値というか，目標値
qd_des =  SV_d.qd;
qdd_des = SV_d.qdd;

% 手先速度調整ゲイン
kh = zeta(8,1);   % 1.5;
kh_L = kh;
kh_R = kh;

at_dash = 0;
bt_dash = 0;
bt_L = 0;
bt_R = 0;
ahL = 0;
ahR = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 接触・カウントフラグ設定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 確実に接触が起こらない条件 (ロボット重心・ターゲット重心間距離がl_hen_min or l_kaku_minより大きい時，接触は起こらない) 
l_side_min = r_tip + half_side;   % 接触する瞬間のロボットアーム手先球中心～ターゲット中心までの距離 = 手先半径 + ターゲットの一辺/2
l_corner_min = r_tip + half_side * sqrt(2);   % 接触する瞬間のロボットアーム手先球中心～ターゲット中心までの距離 = 手先半径 + ターゲットの対角/2

% 接触中のループ数カウント
contact_1_i = 0;
contact_2_i = 0;
countflag_1 = 1;
countflag_2 = 1;

% カウント
count = 1;
% 捕獲カウント
catchtime = 0;

% 接触フラグ 接触1 非接触0
contactflag_L1 = 0;   % 接触フラグ
contactflag_L2 = 0;
contactflag_R1 = 0;
contactflag_R2 = 0;
flag_1_1 = 0;   % ターゲットが左向きに移動フラグ，右向きに移動フラグ，接触せず左向きに移動フラグ，接触せず右向きに移動フラグ
flag_2_1 = 0;
flag_1_2 = 0;
flag_2_2 = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 結果書き込み用datファイルの定義 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 機構長さ
fo_01_Length_etc = fopen( [ datpath, '/', timepath, '_', '01_Length_etc.dat' ], 'w' );   % アニメ用長さ，ロボット・ターゲットの機構的長さ
% ロボット重心 運動
fo_02_RobotCoM_Motion = fopen( [ datpath, '/', timepath, '_', '02_RobotCoM_Motion.dat' ], 'w' );   % ロボット重心の運動に関する変数　位置，速度，加速度，姿勢，角速度，各加速度
% ロボット手先 運動
fo_03_RobotEE_Motion = fopen( [ datpath, '/', timepath, '_', '03_RobotEE_Motion.dat' ], 'w' );   % ロボット手先の運動に関する変数　位置，姿勢，接触位置(ロボット重心座標系)
% ロボットリンク 運動
fo_04_RobotLink_Motion = fopen( [ datpath, '/', timepath, '_', '04_RobotLink_Motion.dat' ], 'w' );   % ロボットリンクの運動に関する変数　位置，関節角度，目標手先速度
% ターゲット重心 運動
fo_05_STargetCoM_Motion = fopen( [ datpath, '/', timepath, '_', '05_STargetCoM_Motion.dat' ], 'w' );   % ターゲットの運動に関する変数　位置，速度，加速度，姿勢，角速度，各加速度
% ターゲット手先 運動
fo_06_STargetEE_Motion = fopen( [ datpath, '/', timepath, '_', '06_STargetEE_Motion.dat' ], 'w' );   % ターゲット手先の運動に関する変数　四角頂点位置，接触位置(ターゲット重心座標系)，接触位置速度
% ロボット 力学
fo_07_Robot_Kinetics = fopen( [ datpath, '/', timepath, '_', '07_Robot_Kinetics.dat' ], 'w' );   % ロボットの力学に関する変数　(ロボットが受ける) 法線力(1~4)，接線力(1~4)，接触力(1~4)，，接触トルク，目標トルク
% ターゲット 力学
fo_08_STarget_Kinetics = fopen( [ datpath, '/', timepath, '_', '08_STarget_Kinetics.dat' ], 'w' );   % ターゲットの力学に関する変数　(ターゲットが受ける) 外力，外トルク
% 接触フラグ
fo_09_ContactFlag = fopen( [ datpath, '/', timepath, '_', '09_ContactFlag.dat' ], 'w' );   % 接触判定(1~4)
% 接触フラグ行列
fo_10_ContactFlagMatrix = fopen( [ datpath, '/', timepath, '_', '10_ContactFlagMatrix.dat' ], 'w' );   % ターゲットの接触領域判定
% 推定情報
fo_11_Estimation = fopen( [ datpath, '/', timepath, '_', '11_Estimation.dat' ], 'w' );   % 推定変数　観測点(1,2)，重心位置，重心速度，姿勢，姿勢(0~pi/2)，角速度，幾何中心，長さ，角度，
% めり込み
fo_12_Length_ForMovie = fopen( [ datpath, '/', timepath, '_', '12_Length_ForMovie.dat' ], 'w' );   % l_x_H(1~4), delta_H(1~4), deltaVel_H(1~4)
% 運動量
fo_13_Momentum_ForMovie = fopen( [ datpath, '/', timepath, '_', '13_Momentum_ForMovie.dat' ], 'w' );   % ロボットの運動量，角運動量，ターゲットの運動量，角運動量，系全体の運動量，角運動量
% 力 アニメーション用
fo_14_Force_ForMovie = fopen( [ datpath, '/', timepath, '_', '14_Force_ForMovie.dat' ], 'w' );   % 法線単位ベクトル，接線単位ベクトル，法線力，接線力
% ロボット アニメーション用
fo_15_Robot_ForMovie = fopen( [ datpath, '/', timepath, '_', '15_Robot_ForMovie.dat' ], 'w' );   % 重心位置，関節位置，手先位置，重心姿勢
% ターゲット アニメーション用
fo_16_STarget_ForMovie = fopen( [ datpath, '/', timepath, '_', '16_STarget_ForMovie.dat' ], 'w' );   % 重心位置，四角頂点位置，重心姿勢
% ターゲット アニメーション用
fo_17_memo = fopen( [ datpath, '/', timepath, '_', '17_memo.dat' ], 'w' );   % 重心位置，四角頂点位置，重心姿勢

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% datファイルの1行目に各変数の名前を書き込み %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 機構長さ   % アニメ用長さ，ロボット・ターゲットの機構的長さ
fprintf( fo_01_Length_etc, [ repmat( s, 1, 16 ), se ], ...   % =17コ
 'r_tip', 'side_target', 'half_side', 'r_motor', 'r_base', 'r_airtank', ...   % 6
 'base_tate', 'base_yoko', 'base_jushin_teihen', 'base_arm_yoko_1', 'base_arm_yoko_2', 'base_arm_tate_1', 'base_arm_tate_2', 'base_airtank_yoko', 'base_airtank_tate', 'tekubi-yubisaki', 'hirakikaku' );   % 11
% ロボット重心 運動   % ロボット重心の運動に関する変数　位置，速度，加速度，姿勢，角速度，角加速度
fprintf( fo_02_RobotCoM_Motion, [ repmat( s, 1, 18 ), se ], ...   % =19
 'time [1]', 'SV_d.R0 [3]',u,u, 'SV_d.v0 [3]',u,u, 'SV_d.vd0 [3]',u,u, 'SV_d.Q0 [3]',u,u, 'SV_d.w0 [3]',u,u, 'SV_d.wd0 [3]',u,u );   % 19
% ロボット手先 運動   % ロボット手先の運動に関する変数　位置，姿勢，接触位置(ロボット重心座標系)
fprintf( fo_03_RobotEE_Motion, [ repmat( s, 1, 36 ), se ], ...   % =37
 'time [1]', 'POS_eL1 [3]',u,u, 'POS_eL2 [3]',u,u, 'POS_eR1 [3]',u,u, 'POS_eR2 [3]',u,u, ...   % 13
             'Qe_degL1 [3]',u,u, 'Qe_degL2 [3]',u,u, 'Qe_degR1 [3]',u,u, 'Qe_degR2 [3]',u,u, ...   % 12
             'curPosBP3_L1 [3]',u,u, 'curPosBP3_L2 [3]',u,u, 'curPosBP3_R1 [3]',u,u, 'curPosBP3_R2 [3]',u,u );   % 12
% ロボットリンク 運動   % ロボットリンクの運動に関する変数　位置，関節角度，目標手先速度
fprintf( fo_04_RobotLink_Motion, [ repmat( s, 1, 36 ), se ], ...   % =37
 'time [1]', 'POS_j1 [2]',u, 'POS_j2 [2]',u, 'POS_j3 [2]',u, 'POS_j4 [2]',u, 'POS_j5 [2]',u, 'POS_j6 [2]',u, 'POS_j7 [2]',u, 'POS_j8 [2]',u, ...   % 17
             'SV_d.q [8]',u,u,u,u,u,u,u, 've1_des [6]',u,u,u,u,u, 've2_des [6]',u,u,u,u,u );   % 20
% ターゲット重心 運動   % ターゲットの運動に関する変数　位置，速度，加速度，姿勢，角速度，角加速度
fprintf( fo_05_STargetCoM_Motion, [ repmat( s, 1, 18 ), se ], ...   % =19
 'time [1]', 'SV_ts.R0 [3]',u,u, 'SV_ts.v0 [3]',u,u, 'SV_ts.vd0 [3]',u,u, 'SV_ts.Q0 [3]',u,u, 'SV_ts.w0 [3]',u,u, 'SV_ts.wd0 [3]',u,u );   % 19
% ターゲット手先 運動   % ターゲット手先の運動に関する変数　四角頂点位置，接触位置(ターゲット重心座標系)，接触位置速度
fprintf( fo_06_STargetEE_Motion, [ repmat( s, 1, 36 ), se ], ...   % =37
 'time [1]', 'shikaku1 [3]',u,u, 'shikaku2 [3]',u,u, 'shikaku3 [3]',u,u, 'shikaku4 [3]',u,u, ...   % 13
             'curPosAP3_L1 [3]',u,u, 'curPosAP3_L2 [3]',u,u, 'curPosAP3_R1 [3]',u,u, 'curPosAP3_R2 [3]',u,u, ...   % 12
             'curPosAP3_vel_L1 [3]',u,u, 'curPosAP3_vel_L2 [3]',u,u, 'curPosAP3_vel_R1 [3]',u,u, 'curPosAP3_vel_R2 [3]',u,u );   % 12
% ロボット 力学      % ロボットの力学に関する変数　(ロボットが受ける) 法線力(1~4)，接線力(1~4)，接触力(1~4)，，接触トルク，目標トルク
fprintf( fo_07_Robot_Kinetics, [ repmat( s, 1, 56 ), se ], ...   % =57
 'time [1]', 'FR_N_L1 [3]',u,u, 'FR_N_L2 [3]',u,u, 'FR_N_R1 [3]',u,u, 'FR_N_R2 [3]',u,u, 'FR_T_L1 [3]',u,u, 'FR_T_L2 [3]',u,u, 'FR_T_R1 [3]',u,u, 'FR_T_R2 [3]',u,u, ...   % 25
             'FR_L1 [3]',u,u, 'FR_L2 [3]',u,u, 'FR_R1 [3]',u,u, 'FR_R2 [3]',u,u, 'T_d3_L1 [3]',u,u, 'T_d3_L2 [3]',u,u, 'T_d3_R1 [3]',u,u, 'T_d3_R2 [3]',u,u, ...   % 24
             'SV_d.tau [8]',u,u,u,u,u,u,u );   % 8
% ターゲット 力学      % ターゲットの力学に関する変数　(ターゲットが受ける) 外力，外トルク
fprintf( fo_08_STarget_Kinetics, [ repmat( s, 1, 6 ), se ], ...   % =7
 'time [1]', 'SV_ts.F0 [3]',u,u, 'SV_ts.T0 [3]',u,u );   % 7
% 接触フラグ   % 接触判定(1~4)
fprintf( fo_09_ContactFlag, [ repmat( s, 1, 9 ), se ], ...   % =10
 'time', 'contactflag_L1', 'contactflag_L2', 'contactflag_R1', 'contactflag_R2', 'flag_1_1', 'flag_2_1', 'flag_1_2', 'flag_2_2', 'catchcount' );   % 10
% 捕獲フラグ   % 捕獲のためのフラグ
fprintf( fo_10_ContactFlagMatrix, [ repmat( s, 1, 37 ), se ], ...   % =38
 'time', 'q0', ...   % 2
         'contact_flag_L1(1,1)', 'contact_flag_L1(2,2)', 'contact_flag_L1(3,3)', 'contact_flag_L1(4,4)', 'contact_flag_L1(1,2)', 'contact_flag_L1(2,3)', 'contact_flag_L1(3,4)', 'contact_flag_L1(4,1)', 'contact_flag_L1(5,5)', ...   % 9
         'contact_flag_L2(1,1)', 'contact_flag_L2(2,2)', 'contact_flag_L2(3,3)', 'contact_flag_L2(4,4)', 'contact_flag_L2(1,2)', 'contact_flag_L2(2,3)', 'contact_flag_L2(3,4)', 'contact_flag_L2(4,1)', 'contact_flag_L2(5,5)', ...   % 9
         'contact_flag_R1(1,1)', 'contact_flag_R1(2,2)', 'contact_flag_R1(3,3)', 'contact_flag_R1(4,4)', 'contact_flag_R1(1,2)', 'contact_flag_R1(2,3)', 'contact_flag_R1(3,4)', 'contact_flag_R1(4,1)', 'contact_flag_R1(5,5)', ...   % 9
         'contact_flag_R2(1,1)', 'contact_flag_R2(2,2)', 'contact_flag_R2(3,3)', 'contact_flag_R2(4,4)', 'contact_flag_R2(1,2)', 'contact_flag_R2(2,3)', 'contact_flag_R2(3,4)', 'contact_flag_R2(4,1)', 'contact_flag_R2(5,5)' );   % 9
% 推定情報   % 推定変数　観測点(1,2)，重心位置，重心速度，姿勢，姿勢(0~pi/2)，角速度，幾何中心，長さ，角度
fprintf( fo_11_Estimation, [ repmat( s, 1, 31 ), se ], ....   % 32
 'time [1]', 'Obs1 [3]',u,u, 'Obs2 [3]',u,u, 'est_P [3]',u,u, 'est_V [3]',u,u, 'est_Q [3]',u,u, 'est_q [1]', 'est_W [3]',u,u, ...   % 20
             'est_geo_center [3]',u,u, 'ts_geo [3]',u,u, 'est_r1 [1]', 'est_r2 [1]', 'est_phi1 [1]', 'est_phi2 [1]', 'est_R12 [1]', 'est_Theta12 [1]' );   % 12
% めり込み   % l_x_H(1~4), delta_H(1~4), deltaVel_H(1~4)
fprintf( fo_12_Length_ForMovie, [ repmat( s, 1, 12 ), se ], ...   % =13
 'time', 'l_x_L1', 'l_x_L2', 'l_x_R1', 'l_x_R2', 'delta_L1', 'delta_L2', 'delta_R1', 'delta_R2', 'deltaVel_L1', 'deltaVel_L2', 'deltaVel_R1', 'deltaVel_R2' );   % 13
% 運動量   % ロボットの運動量，角運動量，ターゲットの運動量，角運動量，系全体の運動量，角運動量
fprintf( fo_13_Momentum_ForMovie, [ repmat( s, 1, 36 ), se ], ...   % =37
 'time [1]', 'PP_d [6]',u,u,u,u,u, 'PP_ts [6]',u,u,u,u,u, 'PP [6]',u,u,u,u,u, 'PP_d_2 [6]',u,u,u,u,u, 'PP_ts_2 [6]',u,u,u,u,u, 'PP_2 [6]',u,u,u,u,u );   % 37
% 力 アニメーション用   % 法線単位ベクトル，接線単位ベクトル，法線力，接線力
fprintf( fo_14_Force_ForMovie, [ repmat( s, 1, 60 ), se ], ...   % =61
 'time [1]', 'norm_L1 [3]',u,u, 'norm_L2 [3]',u,u, 'norm_R1 [3]',u,u, 'norm_R2 [3]',u,u, ...   % 13
             'tang_L1 [3]',u,u, 'tang_L2 [3]',u,u, 'tang_R1 [3]',u,u, 'tang_R2 [3]',u,u, ...   % 12
             'PointC_L1 [3]',u,u, 'PointC_L2 [3]',u,u, 'PointC_R1 [3]',u,u, 'PointC_R2 [3]',u,u, ...   % 12
             'FR_N_L1 [3]',u,u, 'FR_N_L2 [3]',u,u, 'FR_N_R1 [3]',u,u, 'FR_N_R2 [3]',u,u, ...   % 12
             'FR_T_L1 [3]',u,u, 'FR_T_L2 [3]',u,u, 'FR_T_R1 [3]',u,u, 'FR_T_R2 [3]',u,u );   % 12
% ロボット アニメーション用    % 重心位置，関節位置，手先位置，重心姿勢
fprintf( fo_15_Robot_ForMovie, [ repmat( s, 1, 46 ), se ], ...   % =47
 'time [1]', 'SV_d.R0 [3]',u,u, ...   % 4
             'POS_j1 [2]',u, 'POS_j2 [2]',u, 'POS_j3 [2]',u, 'POS_j4 [2]',u, 'POS_j5 [2]',u, 'POS_j6 [2]',u, 'POS_j7 [2]',u, 'POS_j8 [2]',u, ...   % 16
             'POS_eL1 [3]',u,u, 'POS_eL2 [3]',u,u, 'POS_eR1 [3]',u,u, 'POS_eR2 [3]',u,u, ...   % 12
             'Qe_radL1 [3]',u,u, 'Qe_radL2 [3]',u,u, 'Qe_radR1 [3]',u,u, 'Qe_radR2 [3]',u,u, ...   % 12
             'SV_d.Q0 [3]',u,u );   % 3
% ターゲット アニメーション用    % 重心位置，関節位置，手先位置，重心姿勢
fprintf( fo_16_STarget_ForMovie, [ repmat( s, 1, 18 ), se ], ...   % =19
 'time [1]', 'SV_ts.R0 [3]',u,u, 'shikaku1 [3]',u,u, 'shikaku2 [3]',u,u, 'shikaku3 [3]',u,u, 'shikaku4 [3]',u,u, 'SV_ts.Q0 [3]',u,u );   % 19

% dPOS
fprintf( fo_17_memo, [ repmat( s, 1, 31 ), se ], ...   % =32
 'time [1]', 'd_POSL [3]',u,u, 'dPOSR [3]',u,u,'xeL_des(1:2)',u, 'xeR_des(1:2)',u, ...
             'd_ten1',u,u, 'd_ten2',u,u, 'd_ten3',u,u, 'd_ten4',u,u, 'QL3', 'QR7', 'QL2', 'QR6', 'd_QL3', 'd_QR7', 'Phase', 'Catchtime', 'flagphase5' );   % 32


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% シミュレーションループスタート %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for time = 0 : d_time : ( endtime + minus_time )

display_time = ( time - minus_time );
[ time display_time, Phase, catchtime ]'   % 現在どのステップを計算しているかをコマンドウィンドウに出力
delta_t_2 = 0.7; %Phase6における手先速度制御時間
delta_t_3 = 0.7;

     t1 = t0 + minus_time + delta_t; %固定値
     minus_time_2 = t1 + 0.5; %固定値
%{
if ((time - minus_time_2) > delta_t_2)
     minus_time_3 = minus_time_2 + delta_t_2;
end
%}
% 円のダミー中心位置をターゲット重心位置から計算
%ts_geo(1,1) = SV_ts.R0(1,1) + error_tsR0 * cos( theta_tsR0 + SV_ts.Q0(3,1) );
ts_geo(1,1) = SV_ts.R0(1,1);
%ts_geo(2,1) = SV_ts.R0(2,1) + error_tsR0 * sin( theta_tsR0 + SV_ts.Q0(3,1) );
ts_geo(2,1) = SV_ts.R0(2,1);
ts_geo(3,1) = SV_ts.R0(3,1);

% 常に左上→左下→右下→右上が1→2→3→4になるように設定
if      pi/2 <= SV_ts.Q0(3,1) && SV_ts.Q0(3,1) < pi    % ターゲットがπ/2～πに傾くとき
    q0 = SV_ts.Q0(3,1) - pi/2;
elseif -pi/2 <= SV_ts.Q0(3,1) && SV_ts.Q0(3,1) < 0     % ターゲットが-π/2～0に傾くとき
    q0 = SV_ts.Q0(3,1) + pi/2;
elseif -pi <= SV_ts.Q0(3,1) && SV_ts.Q0(3,1) < -pi/2   % ターゲットが-π～-π/2に傾くとき
    q0 = SV_ts.Q0(3,1) + pi;
else                                                   % ターゲットが0～π/2のときはそのまま代入
    q0 = SV_ts.Q0(3,1);    
end

[ shikaku1, shikaku2, shikaku3, shikaku4, shikaku1_2, shikaku2_3, shikaku3_4, shikaku4_1, shikaku2_1, shikaku3_2, shikaku4_3, shikaku1_4, ...
  l1, l2, l3, l4, g1, g2, g3, g4, contact_flag_L1, contact_flag_L2, contact_flag_R1, contact_flag_R2 ] ...
= Shikaku_FourTips_CONTACT_FLAG( SV_ts, q0, side_target, r_tip, ts_geo, POS_eL1, POS_eL2, POS_eR1, POS_eR2 );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 接触定義 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calc_Rgで重心位置決定
Rg_d = calc_Rg( LP_d, SV_d );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 接触力計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% 左側1,2 %%%%%%%%%%
[ contactflag_L1, F_NN_L1, FR_N_L1, FR_T_L1, norm_L1, tang_L1, FR_L1, delta_L1, deltaVel_L1, delta_tmp_L1, TB0_L1, DB0_L1, curPosAP33_L1, curPosAP3_delta_L1, curPosAP3_tmp_L1, curPosBP33_L1, PointC_L1, l_x_L1 ] ...
= Shikaku_FourTips_LEFT_CONTACT_v03( SV_d, SV_ts, d_time, half_side, r_tip, contact_flag_L1, contactflag_L1, shikaku1, shikaku2, shikaku3, shikaku4, ...
                                      l_side_min, l_corner_min, ts_geo, q0, Rg_d, POS_eL1, delta_tmp_L1, curPosAP3_tmp_L1, l1, l2, l3, l4, g1, g2, g3, g4, cof, kkk, ccc );
[ contactflag_L2, F_NN_L2, FR_N_L2, FR_T_L2, norm_L2, tang_L2, FR_L2, delta_L2, deltaVel_L2, delta_tmp_L2, TB0_L2, DB0_L2, curPosAP33_L2, curPosAP3_delta_L2, curPosAP3_tmp_L2, curPosBP33_L2, PointC_L2, l_x_L2 ] ...
= Shikaku_FourTips_LEFT_CONTACT_v03( SV_d, SV_ts, d_time, half_side, r_tip, contact_flag_L2, contactflag_L2, shikaku1, shikaku2, shikaku3, shikaku4, ...
                                      l_side_min, l_corner_min, ts_geo, q0, Rg_d, POS_eL2, delta_tmp_L2, curPosAP3_tmp_L2, l1, l2, l3, l4, g1, g2, g3, g4, cof, kkk, ccc );
%%%%%%%%%% 右側3,4 %%%%%%%%%%
[ contactflag_R1, F_NN_R1, FR_N_R1, FR_T_R1, norm_R1, tang_R1, FR_R1, delta_R1, deltaVel_R1, delta_tmp_R1, TB0_R1, DB0_R1, curPosAP33_R1, curPosAP3_delta_R1, curPosAP3_tmp_R1, curPosBP33_R1, PointC_R1, l_x_R1 ] ...
= Shikaku_FourTips_RIGHT_CONTACT_v03( SV_d, SV_ts, d_time, half_side, r_tip, contact_flag_R1, contactflag_R1, shikaku1, shikaku2, shikaku3, shikaku4, ...
                                      l_side_min, l_corner_min, ts_geo, q0, Rg_d, POS_eR1, delta_tmp_R1, curPosAP3_tmp_R1, l1, l2, l3, l4, g1, g2, g3, g4, cof, kkk, ccc );
[ contactflag_R2, F_NN_R2, FR_N_R2, FR_T_R2, norm_R2, tang_R2, FR_R2, delta_R2, deltaVel_R2, delta_tmp_R2, TB0_R2, DB0_R2, curPosAP33_R2, curPosAP3_delta_R2, curPosAP3_tmp_R2, curPosBP33_R2, PointC_R2, l_x_R2 ] ...
= Shikaku_FourTips_RIGHT_CONTACT_v03( SV_d, SV_ts, d_time, half_side, r_tip, contact_flag_R2, contactflag_R2, shikaku1, shikaku2, shikaku3, shikaku4, ...
                                      l_side_min, l_corner_min, ts_geo, q0, Rg_d, POS_eR2, delta_tmp_R2, curPosAP3_tmp_R2, l1, l2, l3, l4, g1, g2, g3, g4, cof, kkk, ccc );


% if contactflag_L1 == 1 && contactflag_L2 == 1 && contactflag_R1 == 1 && contactflag_R2 == 1
%    catchcount = catchcount + d_time;
% else
%    catchcount = 0;
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 接触トルクの計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% 左側1,2 %%%%%%%%%%
T_t3_L1 = curPosAP33_L1 * TB0_L1 * ( -FR_L1 );   % curPosAP33(交代行列) * A0'(ターゲットの方向余弦行列) * -F3(慣性座標系から見たターゲットが受ける接触力);   zeros(3,1);
T_t3_L2 = curPosAP33_L2 * TB0_L2 * ( -FR_L2 );
T_d3_L1 = cross( ( PointC_L1 - POS_eL ), FR_L1 );   % zeros(3,1); curPosBP33_L1 * DB0_L1 * (  FR_L1 );   % curPosBP33 * B0' * (F3);
T_d3_L2 = cross( ( PointC_L2 - POS_eL ), FR_L2 );   % zeros(3,1); curPosBP33_L2 * DB0_L2 * (  FR_L2 );
%%%%%%%%%% 右側3,4 %%%%%%%%%%
T_t3_R1 = curPosAP33_R1 * TB0_R1 * ( -FR_R1 );
T_t3_R2 = curPosAP33_R2 * TB0_R2 * ( -FR_R2 );
T_d3_R1 = cross( ( PointC_R1 - POS_eR ), FR_R1 );   % zeros(3,1); curPosBP33_R1 * DB0_R1 * (  FR_R1 );
T_d3_R2 = cross( ( PointC_R2 - POS_eR ), FR_R2 );   % zeros(3,1); curPosBP33_R2 * DB0_R2 * (  FR_R2 );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ターゲット，ロボットが受ける力とトルクの計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SV_ts.F0 = -( FR_L1 + FR_L2 + FR_R1 + FR_R2 );      % ターゲットが受ける力(左右からの合力) F3_1(左手,慣性右方向) + F3_2(右手,慣性左方向)
SV_ts.T0 = T_t3_L1 + T_t3_L2 + T_t3_R1 + T_t3_R2;    % ターゲットが受けるトルク(左右からの合トルク)
FR_L = FR_L1 + FR_L2;
FR_R = FR_R1 + FR_R2;
T_d3_L = T_d3_L1 + T_d3_L2;
T_d3_R = T_d3_R1 + T_d3_R2;
SV_d.Fe(:,4) = FR_L;    % ロボット(左)が受ける力 (左向きにする)
SV_d.Te(:,4) = T_d3_L;   % ロボット(左)が受けるトルク
SV_d.Fe(:,8) = FR_R;    % ロボット(右)が受ける力 (右向きにする)
SV_d.Te(:,8) = T_d3_R;   % ロボット(右)が受けるトルク

Fh_L = [ FR_L' T_d3_L' ]';   % 力・トルクをまとめる
Fh_R = [ FR_R' T_d3_R' ]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 観測点の保存・更新 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta = 0.000000;
%%% 観測点の定義
% 保存
Obs1_tmp = Obs1;
Obs2_tmp = Obs2;
NorTraj_12_tmp = NorTraj_12;
est_P_tmp = est_P;
est_Q_tmp = est_Q;
% 更新   % ここは未知情報を代入してよい
Obs1 = ts_geo + sqrt(2) * r_Obs1 * [ cos( kansoku1 + SV_ts.Q0(3,1) ), sin( kansoku1 + SV_ts.Q0(3,1) ), 0 ]' + [ randi(10,1) * delta, randi(10,1) * delta, 0 ]';   % 初期shikaku1と同じ点を観測
Obs2 = ts_geo + sqrt(2) * r_Obs2 * [ cos( kansoku2 + SV_ts.Q0(3,1) ), sin( kansoku2 + SV_ts.Q0(3,1) ), 0 ]' + [ randi(10,1) * delta, randi(10,1) * delta, 0 ]';   % 初期shikaku2と同じ点を観測

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 重心位置の推定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[ est_P, est_V, est_erP, NorTraj_12, est_Q, est_q, est_W, est_geo, est_half_side, est_r1, est_r2, est_phi1, est_phi2, est_R12, est_Theta12 ] ...
= Estimation( d_time, time, Obs1, Obs2, Obs1_tmp, Obs2_tmp, NorTraj_12_tmp, est_P_tmp, est_Q_tmp, contactflag_L1, contactflag_L2, contactflag_R1, contactflag_R2, est_phi1, est_phi2, est_r1, est_r2, est_P, est_V, est_erP, NorTraj_12, est_Theta12 );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ヤコビ行列計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 一般化ヤコビ行列計算 手先
    JL = calc_gj( LP_d, SV_d, num_eL );   % 一般化ヤコビ行列(左手)
    JR = calc_gj( LP_d, SV_d, num_eR );   % 一般化ヤコビ行列(右手)
%     J_1 = [ JL' JR' ]';   % 一般化ヤコビ行列をまとめる
%     Je_1 = pinv( J_1 );             % ヤコビの逆行列

% ヤコビ行列計算 手首
    HH = calc_hh( LP_d, SV_d );
    Hs = HH(1:6, 1:6);

      JmL = calc_je( LP_d, SV_d, [ 1 2 3 ] );
      [ pL, oL ] = f_kin_j( LP_d, SV_d, [ 1 2 3 ] );   % 手先だからcalc_gjでも
      JsL = [   eye( 3, 3 ) -tilde( pL(:,3)-SV_d.R0 );
              zeros( 3, 3 )               eye( 3, 3 ) ];
      HmL = HH(1:6, 7:6 + LP_d.num_q);
    GJL = JmL - JsL * Hs \ HmL;
      QL_3 = pi/2 + dc2rpy( oL(:,end-2:end)' );   % 左手首リンクの角度   初期角がy軸方向に伸びている状態なので，+π/2をしている
      QL_2 = pi/2 + dc2rpy( oL(:,end-5:end-3)' );
%       QL_3 = dc2rpy( oL(:,end-2:end)' );   % 左手首リンクの角度   初期角がy軸方向に伸びている状態なので，+π/2をしている
%       QL_2 = dc2rpy( oL(:,end-5:end-3)' );


      JmR = calc_je( LP_d, SV_d, [ 5 6 7 ] );
      [ pR, oR ] = f_kin_j( LP_d, SV_d, [ 5 6 7 ] );
      JsR = [   eye( 3, 3 ) -tilde( pR(:,3)-SV_d.R0 );
              zeros( 3, 3 )               eye( 3, 3 ) ];
      HmR = HH( 1:6, 7:6 + LP_d.num_q );
    GJR = JmR - JsR * Hs \ HmR;
      QR_7 = pi/2 + dc2rpy( oR(:,end-2:end)' );   % 右手首リンクの角度
      QR_6 = pi/2 + dc2rpy( oR(:,end-5:end-3)' );
%       QR_7 = dc2rpy( oR(:,end-2:end)' );   % 右手首リンクの角度
%       QR_6 = dc2rpy( oR(:,end-5:end-3)' );

    J_3_G = [ GJL' GJR' ]';
    Je_3_G = pinv( J_3_G );
    Jd_3_G = ( J_3_G - J_3_G_tmp ) / d_time;
    J_3_G_tmp = J_3_G;


hokakutime = 0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 目標手先速度の計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[ xe_des, ve_des, ae_des, d_ten1, d_ten2, d_ten3, d_ten4, POS_j4_tmp, POS_j8_tmp, d_POS_j4, d_POS_j8, VEL_j4, VEL_j8, ...
  d_QL_3, d_QR_7, aL, aR, q_joints, q_wrist, qd_joints, qd_wrist, delta_t, t0, Phase, catchtime ] ...
= DesiredHandMotion_v05( d_time, time, minus_time, minus_time_2, minus_time_3, SV_d, SV_ts, Obs1, Obs2, est_P, est_V, est_Q, est_q, est_W, est_geo, ...
                         POS_j2, POS_j6, POS_j3, POS_j7, POS_j4, POS_j8, POS_j4_tmp, POS_j8_tmp, d_POS_j4, d_POS_j8,...
                         D, D_0, d_QL_3, d_QR_7, aL, aR, d_ten1, d_ten2, d_ten3, d_ten4, delta_t, delta_t_2, delta_t_3, t0, ...
                         contactflag_L1, contactflag_L2, contactflag_R1, contactflag_R2, catchtime, POS_eL1, POS_eL2, POS_eR1, POS_eR2, hokakutime, DHD, Phase, t1 );

Kd_ichi = Kd/2;
Kd_kaku = Kd/12;
Kp_ichi = Kp*2;
Kp_kaku = Kp/30;
% Kp_wrist = 1;
% Kd_wrist = 0.3;
Kp_wrist = 0.001;
Kd_wrist = 0.03;


% if time == round( (t0+delta_t)/d_time )*d_time
% flag_kotei = 1;
% q_joints_kotei = q_joints;
% end

%%% 手先位置制御のためのφddの設定 %%%
qdd = Je_3_G * ...
      ( ae_des ...
      + Kd_ichi * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_j4', zeros(1,4), VEL_j8', zeros(1,4) ]' ) ...
      + Kp_ichi * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_j4', zeros(1,4), POS_j8', zeros(1,4) ]' ) ...
      + Kd_kaku * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(3,1), zeros(1,5), qd_joints(7,1) ]' ) ...
      + Kp_kaku * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(3,1), zeros(1,5),  q_joints(7,1) ]' ) ...
      + Jd_3_G * qd_joints );
qdd_2 = Je_3_G * ...
      ( ae_des ...
      + Kd_kaku * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(3,1), zeros(1,5), qd_joints(7,1) ]' ) ...
      + Kp_kaku * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(3,1), zeros(1,5),  q_joints(7,1) ]' ) ...
      + Jd_3_G * qd_joints );
  
  qdd_3 = Je_3_G * ([ VEL_j4', zeros(1,4), VEL_j8', zeros(1,4) ]' );
  

% 関数calc_asuta呼び出し
    [ H_asuta, C_asuta ] = calc_asuta( LP_d, SV_d );

% 目標トルク
 if     Phase == 4 % && flagphase5 ~= 1    % time >= round( (t0+delta_t)/d_time )*d_time   % flag_kotei == 1
    q_joints_kotei = q_joints;

 KL = kakudo( [ POS_j3' 0 ]', [ ( 2 * POS_j3 - POS_j2 )' 0 ]', SV_ts.R0 );
 KR = kakudo( [ POS_j7' 0 ]', [ ( 2 * POS_j7 - POS_j6 )' 0 ]', SV_ts.R0 );
     SV_d.tau =  - JL' * Fh_L - JR' * Fh_R ... H_asuta * qdd_2 + C_asuta - JL' * Fh_L - JR' * Fh_R ... 
 ...             + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
 ...             + 3 * ( [ q_joints_kotei(1:3)', 0, q_joints_kotei(5:7)', 0 ]' - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
              + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
              + 3 * ( [ q_joints_kotei(1:2)', KL, 0, q_joints_kotei(5:6)', KR, 0 ]' ...
                    - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
              + Kd_wrist * ( zeros(8,1) - qd_wrist ) + Kp_wrist * ( zeros(8,1) - q_wrist );
        
% 目標トルク
elseif     Phase == 5 % && flagphase5 ~= 1    % time >= round( (t0+delta_t)/d_time )*d_time   % flag_kotei == 1
 
     q_joints_kotei = q_joints;

 KL = kakudo( [ POS_j3' 0 ]', [ ( 2 * POS_j3 - POS_j2 )' 0 ]', SV_ts.R0 );
 KR = kakudo( [ POS_j7' 0 ]', [ ( 2 * POS_j7 - POS_j6 )' 0 ]', SV_ts.R0 );
     SV_d.tau =  - JL' * Fh_L - JR' * Fh_R ... H_asuta * qdd_2 + C_asuta - JL' * Fh_L - JR' * Fh_R ... 
 ...             + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
 ...             + 3 * ( [ q_joints_kotei(1:3)', 0, q_joints_kotei(5:7)', 0 ]' - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
              + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
              + 3 * ( [ q_joints_kotei(1:2)', KL, 0, q_joints_kotei(5:6)', KR, 0 ]' ...
                    - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
              + Kd_wrist * ( zeros(8,1) - qd_wrist ) + Kp_wrist * ( zeros(8,1) - q_wrist );
    %{
    SV_d.tau = H_asuta * qdd_2 + C_asuta - JL' * Fh_L - JR' * Fh_R ... - JL' * Fh_L - JR' * Fh_R ... 
...             + 1 * ( zeros(8,1) - [ qd_joints(1:2)', zeros(1,2), qd_joints(5:6)', zeros(1,2) ]' ) ...
...             + 2 * ( [ q_joints_kotei(1:2)', zeros(1,2), q_joints_kotei(5:6)', zeros(1,2) ]' - [ q_joints(1:2)', zeros(1,2), q_joints(5:6)', zeros(1,2) ]' ) ...
...             + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
...             + 2 * ( [ q_joints_kotei(1:3)', 0, q_joints_kotei(5:7)', 0 ]' - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
             + Kd_wrist * ( zeros(8,1) - qd_wrist ) + Kp_wrist * ( zeros(8,1) - q_wrist );
      %}
         % 目標トルク
elseif     Phase == 6 % && flagphase5 ~= 1    % time >= round( (t0+delta_t)/d_time )*d_time   % flag_kotei == 1
    q_joints_kotei = q_joints;
    
        
    SV_d.tau = H_asuta * qdd_3 + C_asuta - JL' * Fh_L - JR' * Fh_R ... 
...             + 1 * ( zeros(8,1) - [ qd_joints(1:2)', zeros(1,2), qd_joints(5:6)', zeros(1,2) ]' ) ...
...             + 2 * ( [ q_joints_kotei(1:2)', zeros(1,2), q_joints_kotei(5:6)', zeros(1,2) ]' - [ q_joints(1:2)', zeros(1,2), q_joints(5:6)', zeros(1,2) ]' ) ...
...             + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
...             + 2 * ( [ q_joints_kotei(1:3)', 0, q_joints_kotei(5:7)', 0 ]' - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
             + Kd_wrist * ( zeros(8,1) - qd_wrist ) + Kp_wrist * ( zeros(8,1) - q_wrist );
 
% 目標トルク
elseif     Phase == 7 && flagphase5 ~= 1    % time >= round( (t0+delta_t)/d_time )*d_time   % flag_kotei == 1
    q_joints_kotei = q_joints;
       
 
 KL = kakudo( [ POS_j3' 0 ]', [ ( 2 * POS_j3 - POS_j2 )' 0 ]', [ POS_j7' 0 ]' );
 KR = kakudo( [ POS_j7' 0 ]', [ ( 2 * POS_j7 - POS_j6 )' 0 ]', [ POS_j3' 0 ]' );
     SV_d.tau = - JL' * Fh_L - JR' * Fh_R ... H_asuta * qdd_2 + C_asuta - JL' * Fh_L - JR' * Fh_R ... 
 ...             + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
 ...             + 3 * ( [ q_joints_kotei(1:3)', 0, q_joints_kotei(5:7)', 0 ]' - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
              + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
              + 3 * ( [ q_joints_kotei(1:2)', KL, 0, q_joints_kotei(5:6)', KR, 0 ]' ...
                    - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
              + Kd_wrist * ( zeros(8,1) - qd_wrist ) + Kp_wrist * ( zeros(8,1) - q_wrist );
    
elseif Phase == 8 || flagphase5 == 1
%   if catchtime >= hokakutime
     if catchtime > ( hokakutime - d_time )  &&  catchtime < ( hokakutime + d_time )
        flagphase5 = 1;
        q_joints_kotei = q_joints;
     end

KL = kakudo( [ POS_j3' 0 ]', [ ( 2 * POS_j3 - POS_j2 )' 0 ]', [ POS_j7' 0 ]' );
KR = kakudo( [ POS_j7' 0 ]', [ ( 2 * POS_j7 - POS_j6 )' 0 ]', [ POS_j3' 0 ]' );
    SV_d.tau = - JL' * Fh_L - JR' * Fh_R ... H_asuta * qdd_2 + C_asuta - JL' * Fh_L - JR' * Fh_R ... 
...             + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
...             + 3 * ( [ q_joints_kotei(1:3)', 0, q_joints_kotei(5:7)', 0 ]' - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
             + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
             + 3 * ( [ q_joints_kotei(1:2)', KL, 0, q_joints_kotei(5:6)', KR, 0 ]' ...
                   - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
             + Kd_wrist * ( zeros(8,1) - qd_wrist ) + Kp_wrist * ( zeros(8,1) - q_wrist );

else   % Phase = 1,2,3
 
    SV_d.tau = H_asuta * qdd + C_asuta - JL' * Fh_L - JR' * Fh_R ...
             + Kd_wrist * ( zeros(8,1) - qd_wrist ) + Kp_wrist * ( zeros(8,1) - q_wrist );
      
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 順動力学の計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 運動方程式より次の時間のベースの加速度及び関節角加速度を計算し，それを積分することによりでベースの位置，速度，関節角，関節角速度を求める．
% 運動方程式中の速度非線形項を計算するために，再帰ニュートンオイラー法による逆動力学も内部で解いている．
SV_d =  f_dyn_rk2( LP_d, SV_d );
SV_ts = f_dyn_rk2( LP_ts, SV_ts );

%%%%%%%%%% 手先位置姿勢の計算(順運動学) %%%%%%%%%%
% 各リンクの座標変換行列(方向余弦行列)の計算（リンクi → 慣性座標系）
SV_d = calc_aa( LP_d, SV_d );
% 各リンク重心の位置
SV_d = calc_pos( LP_d, SV_d );
% 全関節位置姿勢の計算
[ POS_j_L, ORI_j_L ] = f_kin_j( LP_d, SV_d, jointsL );   % 左手1
[ POS_j_R, ORI_j_R ] = f_kin_j( LP_d, SV_d, jointsR );   % 右手3
% 各関節の位置 x,y
POS_j1 = POS_j_L( 1:2, 1 );   % 関節1の位置代入
POS_j2 = POS_j_L( 1:2, 2 );
POS_j3 = POS_j_L( 1:2, 3 );
POS_j4 = POS_j_L( 1:2, 4 );
POS_j5 = POS_j_R( 1:2, 1 );
POS_j6 = POS_j_R( 1:2, 2 );
POS_j7 = POS_j_R( 1:2, 3 );
POS_j8 = POS_j_R( 1:2, 4 );
% 手先位置姿勢の計算
[ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   % 左手
Qe_radL1 = dc2rpy( ORI_eL' );   %左側
Qe_radL2 = dc2rpy( ORI_eL' );   %左側
POS_eL1 = [ POS_j4' 0 ]' + rpy2dc([ 0 0  theta ])' * ( POS_eL - [ POS_j4' 0 ]' ) * L / norm( LP_d.cc( :, 4, 4 ) );
POS_eL2 = [ POS_j4' 0 ]' + rpy2dc([ 0 0 -theta ])' * ( POS_eL - [ POS_j4' 0 ]' ) * L / norm( LP_d.cc( :, 4, 4 ) );
[ POS_eR, ORI_eR ] = f_kin_e( LP_d, SV_d, jointsR );   % 右手
Qe_radR1 = dc2rpy( ORI_eR' );   %右側
Qe_radR2 = dc2rpy( ORI_eR' );   %右側
POS_eR1 = [ POS_j8' 0 ]' + rpy2dc([ 0 0  theta ])' * ( POS_eR - [ POS_j8' 0 ]' ) * L / norm( LP_d.cc( :, 8, 8 ) );
POS_eR2 = [ POS_j8' 0 ]' + rpy2dc([ 0 0 -theta ])' * ( POS_eR - [ POS_j8' 0 ]' ) * L / norm( LP_d.cc( :, 8, 8 ) );
% ベース角度のオイラー角表現
SV_d.Q0 = dc2rpy( SV_d.A0' );       
% 角度をradからdegへ変換
Q0_deg_d = rad2deg( SV_d.Q0 );   % ベースの姿勢[deg]
Qe_degL1 = rad2deg( Qe_radL1 );   % 左手先1の姿勢[deg]
Qe_degL2 = rad2deg( Qe_radL2 );   % 左手先2の姿勢[deg]
Qe_degR1 = rad2deg( Qe_radR1 );   % 右手先3の姿勢[deg]
Qe_degR2 = rad2deg( Qe_radR2 );   % 右手先4の姿勢[deg]
q_deg = rad2deg( SV_d.q );   % 関節の姿勢[deg]
% ターゲット姿勢計算
SV_ts.Q0 = dc2rpy( SV_ts.A0' );   % ターゲットの方向余弦行列を角度に変換
Q0_deg_t = rad2deg( SV_ts.Q0 );

%ベースから見たターゲットの角速度
w_dtots = SV_ts.w0 - SV_d.w0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 全運動量の計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ロボット
  HH_d =  calc_hh( LP_d, SV_d );
  Hb_d =  HH_d( 1:6, 1:6 );
  Hbm_d = HH_d( 1:6, 7:end );
  PP_d =  Hb_d * [ SV_d.v0' SV_d.w0' ]' + Hbm_d * SV_d.qd;  % ベース重心周り

% ターゲット
  HH_ts = calc_hh( LP_ts, SV_ts );
  Hb_ts = HH_ts( 1:6, 1:6 );
  PP_ts = Hb_ts * [ SV_ts.v0' SV_ts.w0' ]';

% 系全体
%   R_d = SV_d.R0 - Rg_d;
%   R_ts = SV_ts.R0 - Rg_d;
  R = SV_ts.R0 - SV_d.R0;
  PP( 1:3, 1 ) = PP_d( 1:3, 1 ) + PP_ts( 1:3, 1 );
  PP( 4:5, 1 ) = zeros( 2, 1 );
  PP( 6, 1 ) = PP_d( 6, 1 ) + PP_ts( 6, 1 ) + LP_ts.mass * ( R( 1, 1 ) * SV_ts.v0( 2, 1 ) - R( 2, 1 ) * SV_ts.v0( 1, 1 ) );

% 各リンク質量
 link_mass = LP_d.m( 1, 1:8 );
% 各リンク慣性モーメント
 link_inertia = [ LP_d.inertia( :, 1:3 ) LP_d.inertia( :, 4:6 ) LP_d.inertia( :, 7:9 ) LP_d.inertia( :, 10:12 ) LP_d.inertia( :, 13:15 ) LP_d.inertia( :, 16:18 ) LP_d.inertia( :, 19:21 ) LP_d.inertia( :, 22:24 ) ];
% 各リンク重心位置
 link_position = SV_d.RR( :, 1:8 );
% 各リンク重心位置の歪対称行列
 link_tilde = [ tilde( link_position( :, 1 ) ) tilde( link_position( :, 2 ) ) tilde( link_position( :, 3 ) ) tilde( link_position( :, 4 ) ) tilde( link_position( :, 5 ) ) tilde( link_position( :, 6 ) ) tilde( link_position( :, 7 ) ) tilde( link_position( :, 8 ) ) ];
% 各リンク重心速度
 link_velocity = SV_d.vv( :, 1:8 );
% 各リンク重心角速度
 link_angularvelocity = SV_d.ww( :, 1:8 );
% ロボットの並進運動量 P_d，角運動量 L_d
  P_d = LP_d.m0 * SV_d.v0;
  P_d = P_d + link_mass(1,1) * link_velocity(:,1);
  P_d = P_d + link_mass(1,2) * link_velocity(:,2);
  P_d = P_d + link_mass(1,3) * link_velocity(:,3);
  P_d = P_d + link_mass(1,4) * link_velocity(:,4);
  P_d = P_d + link_mass(1,5) * link_velocity(:,5);
  P_d = P_d + link_mass(1,6) * link_velocity(:,6);
  P_d = P_d + link_mass(1,7) * link_velocity(:,7);
  P_d = P_d + link_mass(1,8) * link_velocity(:,8);
  L_d = LP_d.inertia0 * SV_d.w0 + tilde( SV_d.R0 ) * LP_d.m0 * SV_d.v0;
  L_d = L_d + link_inertia(:,1:3)   * link_angularvelocity(:,1) + link_tilde(:,1:3)   * link_mass(1,1) * link_velocity(:,1);
  L_d = L_d + link_inertia(:,4:6)   * link_angularvelocity(:,2) + link_tilde(:,4:6)   * link_mass(1,2) * link_velocity(:,2);
  L_d = L_d + link_inertia(:,7:9)   * link_angularvelocity(:,3) + link_tilde(:,7:9)   * link_mass(1,3) * link_velocity(:,3);
  L_d = L_d + link_inertia(:,10:12) * link_angularvelocity(:,4) + link_tilde(:,10:12) * link_mass(1,4) * link_velocity(:,4);
  L_d = L_d + link_inertia(:,13:15) * link_angularvelocity(:,5) + link_tilde(:,13:15) * link_mass(1,5) * link_velocity(:,5);
  L_d = L_d + link_inertia(:,16:18) * link_angularvelocity(:,6) + link_tilde(:,16:18) * link_mass(1,6) * link_velocity(:,6);
  L_d = L_d + link_inertia(:,19:21) * link_angularvelocity(:,7) + link_tilde(:,19:21) * link_mass(1,7) * link_velocity(:,7);
  L_d = L_d + link_inertia(:,22:24) * link_angularvelocity(:,8) + link_tilde(:,22:24) * link_mass(1,8) * link_velocity(:,8);
  PP_d_2 = [ P_d' L_d' ]';
% ターゲット重心位置の歪対称行列
  target_tilde = tilde( SV_ts.R0 );
% ターゲットの並進運動量P_t, 角運動量L_t
  P_ts = LP_ts.m0 * SV_ts.v0;
  L_ts = LP_ts.inertia0 * SV_ts.w0 + target_tilde * P_ts;  
  PP_ts_2 = [ P_ts' L_ts' ]';
% 系全体の運動量
  PP_2 = PP_d_2 + PP_ts_2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 結果をファイルに出力 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 機構長さ   % アニメ用長さ，ロボット・ターゲットの機構的長さ
if time == 0
fprintf( fo_01_Length_etc, [ repmat( g, 1, 16 ), ge ], ...   % =17コ
         r_tip, side_target, half_side, r_motor, r_base, r_airtank, ...   % 6
         base_tate, base_yoko, base_jushin_teihen', base_arm_yoko_1, base_arm_yoko_2, base_arm_tate_1, base_arm_tate_2, base_airtank_yoko, base_airtank_tate, L, theta );   % 11
end
% ロボット重心 運動   % ロボット重心の運動に関する変数　位置，速度，加速度，姿勢，角速度，各加速度
fprintf( fo_02_RobotCoM_Motion, [ repmat( g, 1, 18 ), ge ], ...   % =19
         display_time, SV_d.R0, SV_d.v0, SV_d.vd0, SV_d.Q0, SV_d.w0, SV_d.wd0 );   % 19
% ロボット手先 運動   % ロボット手先の運動に関する変数　位置，姿勢，接触位置(ロボット重心座標系)
fprintf( fo_03_RobotEE_Motion, [ repmat( g, 1, 36 ), ge ], ...   % =37
         display_time, POS_eL1, POS_eL2, POS_eR1, POS_eR2, ...   % 13
         Qe_degL1, Qe_degL2, Qe_degR1, Qe_degR2, ...   % 12
         curPosBP3_L1, curPosBP3_L2, curPosBP3_R1, curPosBP3_R2 );   % 12
% ロボットリンク 運動   % ロボットリンクの運動に関する変数　位置，関節角度，目標手先速度
fprintf( fo_04_RobotLink_Motion, [ repmat( g, 1, 36 ), ge ], ...   % =37
         display_time, POS_j1, POS_j2, POS_j3, POS_j4, POS_j5, POS_j6, POS_j7, POS_j8, ...   % 17
         SV_d.q, veL_des, veR_des );   % 20
% ターゲット重心 運動   % ターゲットの運動に関する変数　位置，速度，加速度，姿勢，角速度，各加速度
fprintf( fo_05_STargetCoM_Motion, [ repmat( g, 1, 21 ), ge ], ...   % =19
         display_time, SV_ts.R0, SV_ts.v0, SV_ts.vd0, SV_ts.Q0, SV_ts.w0, SV_ts.wd0, w_dtots );   % 19
% ターゲット手先 運動   % ターゲット手先の運動に関する変数　四角頂点位置，接触位置(ターゲット重心座標系)，接触位置速度
fprintf( fo_06_STargetEE_Motion, [ repmat( g, 1, 36 ), ge ], ...   % =37
         display_time, shikaku1, shikaku2, shikaku3, shikaku4, ...   % 13
         curPosAP3_L1, curPosAP3_L2, curPosAP3_R1, curPosAP3_R2, ...   % 12
         curPosAP3_vel_L1, curPosAP3_vel_L2, curPosAP3_vel_R1, curPosAP3_vel_R2 );   % 12
% ロボット 力学      % ロボットの力学に関する変数　(ロボットが受ける) 法線力(1~4)，接線力(1~4)，接触力(1~4)，，接触トルク，目標トルク
fprintf( fo_07_Robot_Kinetics, [ repmat( g, 1, 56 ), ge ], ...   % =57
         display_time, FR_N_L1, FR_N_L2, FR_N_R1, FR_N_R2, FR_T_L1, FR_T_L2, FR_T_R1, FR_T_R2, ...   % 25
         FR_L1, FR_L2, FR_R1, FR_R2, T_d3_L1, T_d3_L2, T_d3_R1, T_d3_R2, ...   % 24
         SV_d.tau );   % 8
% ターゲット 力学      % ターゲットの力学に関する変数　(ターゲットが受ける) 外力，外トルク
fprintf( fo_08_STarget_Kinetics, [ repmat( g, 1, 6 ), ge ], ...   % =7
         display_time, SV_ts.F0, SV_ts.T0 );   % 7
% 接触フラグ   % 接触判定(1~4)
fprintf( fo_09_ContactFlag, [ repmat( g, 1, 9 ), ge ], ...   % =10
         display_time, contactflag_L1, contactflag_L2, contactflag_R1, contactflag_R2, flag_1_1, flag_2_1, flag_1_2, flag_2_2, catchtime );   % 10
% 捕獲フラグ   % 捕獲のためのフラグ
fprintf( fo_10_ContactFlagMatrix, [ repmat( g, 1, 37 ), ge ], ...   % =38
         display_time, q0, ...   % 2
         contact_flag_L1(1,1), contact_flag_L1(2,2), contact_flag_L1(3,3), contact_flag_L1(4,4), contact_flag_L1(1,2), contact_flag_L1(2,3), contact_flag_L1(3,4), contact_flag_L1(4,1), contact_flag_L1(5,5), ...   % 9
         contact_flag_L2(1,1), contact_flag_L2(2,2), contact_flag_L2(3,3), contact_flag_L2(4,4), contact_flag_L2(1,2), contact_flag_L2(2,3), contact_flag_L2(3,4), contact_flag_L2(4,1), contact_flag_L2(5,5), ...   % 9
         contact_flag_R1(1,1), contact_flag_R1(2,2), contact_flag_R1(3,3), contact_flag_R1(4,4), contact_flag_R1(1,2), contact_flag_R1(2,3), contact_flag_R1(3,4), contact_flag_R1(4,1), contact_flag_R1(5,5), ...   % 9
         contact_flag_R2(1,1), contact_flag_R2(2,2), contact_flag_R2(3,3), contact_flag_R2(4,4), contact_flag_R2(1,2), contact_flag_R2(2,3), contact_flag_R2(3,4), contact_flag_R2(4,1), contact_flag_R2(5,5) );   % 9
% 推定情報   % 推定変数　観測点(1,2)，重心位置，重心速度，姿勢，姿勢(0~pi/2)，角速度，幾何中心，長さ，角度
fprintf( fo_11_Estimation, [ repmat( g, 1, 31 ), ge ], ....   % 32
         display_time, Obs1, Obs2, est_P, est_V, est_Q, est_q, est_W, ...   % 20
         est_geo, ts_geo, est_r1, est_r2, est_phi1, est_phi2, est_R12, est_Theta12 );   % 12
% めり込み   % l_x_H(1~4), delta_H(1~4), deltaVel_H(1~4)
fprintf( fo_12_Length_ForMovie, [ repmat( g, 1, 12 ), ge ], ...   % =13
         display_time, l_x_L1, l_x_L2, l_x_R1, l_x_R2, ...   % 5
         delta_L1, delta_L2, delta_R1, delta_R2, ...   % 4
         deltaVel_L1, deltaVel_L2, deltaVel_R1, deltaVel_R2 );   % 4
% 運動量   % ロボットの運動量，角運動量，ターゲットの運動量，角運動量，系全体の運動量，角運動量
fprintf( fo_13_Momentum_ForMovie, [ repmat( g, 1, 36 ), ge ], ...   % =37
         display_time, PP_d, PP_ts, PP, ...   % 19
         PP_d_2, PP_ts_2, PP_2 );   % 18
% 力 アニメーション用   % 法線単位ベクトル，接線単位ベクトル，法線力，接線力
fprintf( fo_14_Force_ForMovie, [ repmat( g, 1, 60 ), ge ], ...   % =61
         display_time, norm_L1, norm_L2, norm_R1, norm_R2, ...   % 13
         tang_L1, tang_L2, tang_R1, tang_R2, ...   % 12
         PointC_L1, PointC_L2, PointC_R1, PointC_R2, ...   % 12
         FR_N_L1, FR_N_L2, FR_N_R1, FR_N_R2, ...   % 12
         FR_T_L1, FR_T_L2, FR_T_R1, FR_T_R2 );   % 12
% ロボット アニメーション用    % 重心位置，関節位置，手先位置，重心姿勢
fprintf( fo_15_Robot_ForMovie, [ repmat( g, 1, 46 ), ge ], ...   % =47
         display_time, SV_d.R0, ...   % 4
         POS_j1, POS_j2, POS_j3, POS_j4, POS_j5, POS_j6, POS_j7, POS_j8, ...   % 16
         POS_eL1, POS_eL2, POS_eR1, POS_eR2, ...   % 12
         Qe_radL1, Qe_radL2, Qe_radR1, Qe_radR2, ...   % 12
         SV_d.Q0 );   % 3
% ターゲット アニメーション用    % 重心位置，関節位置，手先位置，重心姿勢
fprintf( fo_16_STarget_ForMovie, [ repmat( g, 1, 18 ), ge ], ...   % =19
         display_time, SV_ts.R0, shikaku1, shikaku2, shikaku3, shikaku4, SV_ts.Q0 );   % 19

% d_POS
fprintf( fo_17_memo, [ repmat( g, 1, 31 ), ge ], ...   % =32
         display_time, d_POS_j4, d_POS_j8, xe_des(1:2), xe_des(7:8), d_ten1, d_ten2, d_ten3, d_ten4, QL_3(3,1), QR_7(3,1), QL_2(3,1), QR_6(3,1), d_QL_3, d_QR_7, Phase, catchtime, flagphase5 );   % 32


   if catchtime > catchlimit;
      ntime = cputime - startCPUT;
      nhour = floor( ntime / 3600 );   % 単位:時間 各要素以下の最も近い整数に丸める
      nmin = floor( ( ntime - nhour * 3600 ) / 60 );   % 単位:分 残りの分，整数に丸める
      nsec = ntime - nhour * 3600 - nmin * 60;   % 単位:秒 残りの秒，整数に丸める
      fopen( [ path,'\○捕獲成功 開始=',num2str(datestr(startT,30)),',終了=',num2str(datestr(clock,30)),',計算時間=',num2str(nhour),'時間',num2str(nmin),'分',num2str(nsec),'秒.txt' ],'w' );
     break
   else
%      if time == endtime
%       ntime = cputime - startCPUT;
%       nhour = floor( ntime / 3600 );   % 単位:時間 各要素以下の最も近い整数に丸める
%       nmin = floor( ( ntime - nhour * 3600 ) / 60 );   % 単位:分 残りの分，整数に丸める
%       nsec = ntime - nhour * 3600 - nmin * 60;   % 単位:秒 残りの秒，整数に丸める
%       fopen( [ path,'\×捕獲失敗 開始=',num2str(datestr(startT,30)),',終了=',num2str(datestr(clock,30)),',計算時間=',num2str(nhour),'時間',num2str(nmin),'分',num2str(nsec),'秒.txt'),'w' ];
%      end
   end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% シミュレーションループストップ %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 結果の表示 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%

%%% 各種設定(figファイル作成後も編集可能) %%%

% 図モード設定
% モード，時間設定
movie_d_time = 0.02;
simplemode = 1  ;
kakudaimode = 1  ;
gridmode = 0 ;


FSA = 40;   % メモリ・ラベルのフォント数
FST = 60;   % タイトルのフォント数
left =   400;   % ウィンドウの位置を変更する際はココをいじる
bottom = 40;
width =  1000;
height = 950;
Position = [ left bottom width height ];

usui_haiiro = [ 0.9 0.9 0.9 ];   % 好きな色をRGBで指定
koi_haiiro = [ 0.5 0.5 0.5 ];
usui_ao = [ 102/255 153/255 255/255 ];
orange = [ 255/255 217/255 0 ];
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
mat_16 = dlmread( [ datpath, '/', timepath, '_', '16_STarget_ForMovie.dat' ], '\t', 1, 0 ); 

% dPOS
mat_17 = dlmread( [ datpath, '/', timepath, '_', '17_memo.dat' ], '\t', 1, 0 ); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 動画の作成 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% {

%%%%%%%%%% 接触・カウントフラグ設定 %%%%%%%%%%
frame_n = 1;             
for_d_angle = linspace( 0, 2 * pi , n );   % 0～2πの区間の等間隔の点をn個含む行ベクトルを返す(0.02π[deg]間隔)
for_pi_half = 0:1/n:pi/2 ;

%%%%%%%%%% 長さ設定 %%%%%%%%%%
r_tip = mat_01(1);   % 金属球半径
side_target = mat_01(2);   % ターゲット1辺
r_motor = mat_01(4);  % モータ半径
r_base = mat_01(5);   % ベースの重心描画用円
base_tate = mat_01(7); base_yoko = mat_01(8); base_jushin_teihen = mat_01(9);
tekubiyubisaki = mat_01(16); hirakikaku = mat_01(17);

%%%%%%%%%% 追加 %%%%%%%%%%
d_pos_j4 = mat_17(:,2:3); d_pos_j8 = mat_17(:,5:6);
xeL = mat_17(:,8:9); xeR = mat_17(:,10:11);
d1 = mat_17(:,12:13); d2 = mat_17(:,15:16); d3 = mat_17(:,18:19); d4 = mat_17(:,21:22);
qL3 = mat_17(:,24); qR7 = mat_17(:,25); qL2 = mat_17(:,26); qR6 = mat_17(:,27);
dqL3 = mat_17(:,28); dqR7 = mat_17(:,29);
phase = mat_17(:,30);


% ロボット
mvtime = mat_15(:,1);   % time 経過刻み時間
pos_rb = mat_15(:,2:3);   % ベース重心位置x,y
pos_j1 = mat_15(:,5:6); pos_j2 = mat_15(:,7:8); pos_j3 = mat_15(:,9:10); pos_j4 = mat_15(:,11:12);
pos_j5 = mat_15(:,13:14); pos_j6 = mat_15(:,15:16); pos_j7 = mat_15(:,17:18); pos_j8 = mat_15(:,19:20);
pos_eL1 = mat_15(:,21:22); pos_eL2 = mat_15(:,24:25); pos_eR1 = mat_15(:,27:28); pos_eR2 = mat_15(:,30:31);
rad_eL1 = mat_15(:,35:36); rad_eL2 = mat_15(:,38:39); rad_eR1 = mat_15(:,41:42); rad_eR2 = mat_15(:,44:45);
ang_bs = mat_15(:,47);                % ベース重心姿勢
% ターゲット
pos_ts =  mat_16(:,2:3);  % ターゲット重心位置x,y
shikaku1 = mat_16(:,5:6); shikaku2 = mat_16(:,8:9); shikaku3 = mat_16(:,11:12); shikaku4 = mat_16(:,14:15);   % 頂点
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
l_target = sqrt( 2 ) * side_target / 2;
lb = sqrt( ( pos_rb(1,1) - pos_j1(1,1) )^2 + ( pos_rb(1,2) - pos_j1(1,2) )^2 );
psi = atan( -( pos_j1(1,1) - pos_rb(1,1) ) / ( pos_j1(1,2) - pos_rb(1,2) ) );

%%%%%%%%%% 頂点 %%%%%%%%%%
% ベース外枠端点位置   % 2017新テストベッド用に書き換え
t_x = [ shikaku1(:,1) shikaku2(:,1) shikaku3(:,1) shikaku4(:,1) shikaku1(:,1) ]; t_y = [ shikaku1(:,2) shikaku2(:,2) shikaku3(:,2) shikaku4(:,2) shikaku1(:,2) ];
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
hold on; box on; axis equal; % grid on; grid minor;
% set( gcf, 'Position', Position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );
% set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
if kakudaimode == 0
 xlim( [ -0.5, 0.5 ] ); ylim( [ -0.2, 0.8 ] );
elseif kakudaimode == 1
 xlim( [ -0.4, 0.4 ] ); ylim( [  -0.2, 0.6 ] );
end
xlabel( 'x [m]' ); ylabel( 'y [m]' );
f = figure(n_figure);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 動画保存 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
moviename = [ movpath, '/', timepath, '_DualArm_TargetShikaku_', 'FrameRate', num2str(framerate), '_Mov.avi' ];
% moviename2 = [ movpath2, '/', daym, '_DualArm_TargetShikaku_', 'FrameRate', num2str(framerate), '_Mov2.avi' ];

% writerObj = VideoWriter( [ datepath, path, '/', timepath, '-', 'mov/', 'sim100.avi' ] );
writerObj = VideoWriter( moviename );
set( writerObj, 'FrameRate', framerate, 'Quality', 100 );
open( writerObj );

r0s = r_tip * sin(for_d_angle); r0c = r_tip * cos(for_d_angle);
r1s = r_motor * sin(for_d_angle); r1c = r_motor * cos(for_d_angle);
r2s = r_motor/2 * sin(for_d_angle); r2c = r_motor/2 * cos(for_d_angle);

if simplemode == 0
    % 追加
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

% ロボット
    rb_base  = fill( b_x(1,:), b_y(1,:), orange, 'LineWidth', sen2 );   % ベース四角形頂点を描画
    rb_arm_L = plot( arm_L_x(1,:), arm_L_y(1,:), 'b-', 'linewidth', sen2 ); rb_arm_R = plot( arm_R_x(1,:), arm_R_y(1,:), 'b-', 'linewidth', sen2 );   % 腕関節を描画
    rb_hand_L = plot( hand_L_x(1,:), hand_L_y(1,:), 'b-', 'linewidth', sen2 ); rb_hand_R = plot( hand_R_x(1,:), hand_R_y(1,:), 'b-', 'linewidth', sen2 );   % 腕関節を描画
    rb_tip_L1 = fill( pos_eL1(1,1) + r0s, pos_eL1(1,2) + r0c, usui_ao, 'LineWidth', sen1 ); rb_tip_L2 = fill( pos_eL2(1,1) + r0s, pos_eL2(1,2) + r0c, usui_ao, 'LineWidth', sen1 );   % 左手先球の円を100個の点で描画
    rb_tip_R1 = fill( pos_eR1(1,1) + r0s, pos_eR1(1,2) + r0c, usui_ao, 'LineWidth', sen1 ); rb_tip_R2 = fill( pos_eR2(1,1) + r0s, pos_eR2(1,2) + r0c, usui_ao, 'LineWidth', sen1 );   % 左手先球の円を100個の点で描画
    rb_j1 = fill( pos_j1(1,1) + r1s, pos_j1(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 ); rb_j2 = fill( pos_j2(1,1) + r1s, pos_j2(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 );
    rb_j3 = fill( pos_j3(1,1) + r1s, pos_j3(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 ); rb_j4 = fill( pos_j4(1,1) + r2s, pos_j4(1,2) + r2c, shiro, 'LineWidth', sen1 );
    rb_j5 = fill( pos_j5(1,1) + r1s, pos_j5(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 ); rb_j6 = fill( pos_j6(1,1) + r1s, pos_j6(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 );
    rb_j7 = fill( pos_j7(1,1) + r1s, pos_j7(1,2) + r1c, koi_haiiro, 'LineWidth', sen1 ); rb_j8 = fill( pos_j8(1,1) + r2s, pos_j8(1,2) + r2c, shiro, 'LineWidth', sen1 );   % 関節のモータ円を100個の点で描画
    % ターゲット
    tg_basesen = fill( t_x(1,:), t_y(1,:), 'r-', 'LineWidth', sen2 );
    % 重心の扇形
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

if simplemode == 0
    % 推定情報
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

% % 初期図を保存
% figure(n_figure)
% title( sprintf( 'Time = %0.3f [s]', mvtime(1) ), 'FontName', 'Times New Roman', 'FontSize', FST );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_1stFig.fig' ] );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_1stFig.png' ] );

%%%%%%%%%% 図を更新 %%%%%%%%%%
i = 1;
while i <= length( mvtime(:,1) )
mvtime(i)
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

     % ロボット
       set( rb_base,  'XData', b_x(i,:), 'YData', b_y(i,:) );
       set( rb_arm_L, 'XData', arm_L_x(i,:), 'YData', arm_L_y(i,:) ); set( rb_arm_R, 'XData', arm_R_x(i,:), 'YData', arm_R_y(i,:) );
       set( rb_hand_L, 'XData', hand_L_x(i,:), 'YData', hand_L_y(i,:) ); set( rb_hand_R, 'XData', hand_R_x(i,:), 'YData', hand_R_y(i,:) );
       set( rb_tip_L1, 'XData', pos_eL1(i,1) + r0s, 'YData', pos_eL1(i,2) + r0c ); set( rb_tip_L2, 'XData', pos_eL2(i,1) + r0s, 'YData', pos_eL2(i,2) + r0c );
       set( rb_tip_R1, 'XData', pos_eR1(i,1) + r0s, 'YData', pos_eR1(i,2) + r0c ); set( rb_tip_R2, 'XData', pos_eR2(i,1) + r0s, 'YData', pos_eR2(i,2) + r0c );
       set( rb_j1, 'XData', pos_j1(i,1) + r1s, 'YData', pos_j1(i,2) + r1c ); set( rb_j2, 'XData', pos_j2(i,1) + r1s, 'YData', pos_j2(i,2) + r1c );
       set( rb_j3, 'XData', pos_j3(i,1) + r1s, 'YData', pos_j3(i,2) + r1c ); set( rb_j4, 'XData', pos_j4(i,1) + r2s, 'YData', pos_j4(i,2) + r2c );
       set( rb_j5, 'XData', pos_j5(i,1) + r1s, 'YData', pos_j5(i,2) + r1c ); set( rb_j6, 'XData', pos_j6(i,1) + r1s, 'YData', pos_j6(i,2) + r1c );
       set( rb_j7, 'XData', pos_j7(i,1) + r1s, 'YData', pos_j7(i,2) + r1c ); set( rb_j8, 'XData', pos_j8(i,1) + r2s, 'YData', pos_j8(i,2) + r2c );
       set( rb_cross_1, 'XData', [ pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half ) ),        pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half + pi ) ),     pos_rb(i,1) ], ...
                        'YData', [ pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half ) ),        pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half + pi ) ),     pos_rb(i,2) ] );
       set( rb_cross_2, 'XData', [ pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half + pi/2 ) ), pos_rb(i,1), ( pos_rb(i,1) + r_base * cos( ang_bs(i,1) + for_pi_half + 3*pi/2 ) ), pos_rb(i,1) ], ...
                        'YData', [ pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half + pi/2 ) ), pos_rb(i,2), ( pos_rb(i,2) + r_base * sin( ang_bs(i,1) + for_pi_half + 3*pi/2 ) ), pos_rb(i,2) ] );
     % ターゲット
       set( tg_basesen, 'XData', t_x(i,:), 'YData', t_y(i,:) );

       set( tg_cross_1, 'XData', [ pos_ts(i,1), ( pos_ts(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half ) ),        pos_ts(i,1), ( pos_ts(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half + pi ) ),     pos_ts(i,1) ], ...
                        'YData', [ pos_ts(i,2), ( pos_ts(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half ) ),        pos_ts(i,2), ( pos_ts(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half + pi ) ),     pos_ts(i,2) ] );
       set( tg_cross_2, 'XData', [ pos_ts(i,1), ( pos_ts(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half + pi/2 ) ), pos_ts(i,1), ( pos_ts(i,1) + r_base * cos( ang_tr(i,1) + for_pi_half + 3*pi/2 ) ), pos_ts(i,1) ], ...
                        'YData', [ pos_ts(i,2), ( pos_ts(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half + pi/2 ) ), pos_ts(i,2), ( pos_ts(i,2) + r_base * sin( ang_tr(i,1) + for_pi_half + 3*pi/2 ) ), pos_ts(i,2) ] );

if simplemode == 0
     % 推定情報
       set( obs1, 'XData', obs1_tr(i,1) + r0s/2, 'YData', obs1_tr(i,2) + r0c/2 );
       set( obs2, 'XData', obs2_tr(i,1) + r0s/2, 'YData', obs2_tr(i,2) + r0c/2 );
       set( tg_estcom, 'XData', pos_est(i,1) + r_tip * sin(for_d_angle), 'YData', pos_est(i,2) + r_tip * cos(for_d_angle) );
end

       title( sprintf( 'Time = %0.3f [s]', mvtime(i,1) ), 'FontName', 'Times New Roman', 'FontSize', FST );
       F( frame_n ) = getframe( n_figure );
       hold off;
       drawnow;
       writeVideo( writerObj, F( frame_n ) );
       frame_n = frame_n + 1;

    end 

% {
    if rem( mvtime(i), movie_d_time * 5 ) == 0 && mvtime(i) >= 0 && mvtime(i) <= 10    % mvtime(i)をmovie_d_timeで除算した後の剰余が0のとき(割り切れるとき)
        % 図を保存
        figure(n_figure)
        title( sprintf( 'Time = %0.3f [s]', mvtime(i) ), 'FontName', 'Times New Roman', 'FontSize', FST );
        saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_TrackingContact_', num2str(mvtime(i)*100), 'Fig.fig' ] );
        saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_TrackingContact_', num2str(mvtime(i)*100), 'Fig.png' ] );
%         saveas( figure(n_figure), [ movpath, '/', daym, '_DualArm_TargetShikaku_TrackingContact_', num2str(mvtime(i)*100), '-Fig2.fig' ] );
%         saveas( figure(n_figure), [ movpath, '/', daym, '_DualArm_TargetShikaku_TrackingContact_', num2str(mvtime(i)*100), '-Fig2.png' ] );
    end
%}

    i = i + 1;

end

% % 最後の図を保存
% figure(n_figure)
% title( sprintf( 'Time = %0.3f [s]', mvtime(end,1) ), 'FontName', 'Times New Roman', 'FontSize', FST );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_LastFig4.fig' ] );
% saveas( figure(n_figure), [ movpath, '/', timepath, '_DualArm_TargetShikaku_LastFig4.png' ] );
% n_figure = n_figure + 1;

close( writerObj );

%}

%}   % 動画作成終了
close

%%

n_figure = 1;

%%% 各種設定(figファイル作成後も編集可能) %%%

FSA = 20;   % メモリ・ラベルのフォントサイズ
FST = 20;   % タイトルのフォントサイズ
left =   300;   % ウィンドウの位置を変更する際はココをいじる
bottom = 0;
width =  1200;
height = 1000;
Position = [ left bottom width height ];

usui_haiiro = [ 0.9 0.9 0.9 ];   % 好きな色を指定
koi_haiiro = [ 0.5 0.5 0.5 ];
usui_ao = [ 102/255 153/255 255/255 ];
orange = [ 255/255 217/255 0 ];
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
% ロボット重心 運動
mat_02 = dlmread( [ datpath, '/', timepath, '_', '02_RobotCoM_Motion.dat' ], '\t', 1, 0 );
% ロボット手先 運動
mat_03 = dlmread( [ datpath, '/', timepath, '_', '03_RobotEE_Motion.dat' ], '\t', 1, 0 );
% ロボットリンク 運動
mat_04 = dlmread( [ datpath, '/', timepath, '_', '04_RobotLink_Motion.dat' ], '\t', 1, 0 );
% ターゲット重心 運動
mat_05 = dlmread( [ datpath, '/', timepath, '_', '05_STargetCoM_Motion.dat' ], '\t', 1, 0 );
% ターゲット重心 運動
mat_06 = dlmread( [ datpath, '/', timepath, '_', '06_STargetEE_Motion.dat' ], '\t', 1, 0 );
% ロボット 力学
mat_07 = dlmread( [ datpath, '/', timepath, '_', '07_Robot_Kinetics.dat' ], '\t', 1, 0 );
% ターゲット 力学
mat_08 = dlmread( [ datpath, '/', timepath, '_', '08_STarget_Kinetics.dat' ], '\t', 1, 0 );
% 接触フラグ
mat_09 = dlmread( [ datpath, '/', timepath, '_', '09_ContactFlag.dat' ], '\t', 1, 0 );
% 接触フラグ行列
mat_10 = dlmread( [ datpath, '/', timepath, '_', '10_ContactFlagMatrix.dat' ], '\t', 1, 0 );
% 推定情報
mat_11 = dlmread( [ datpath, '/', timepath, '_', '11_Estimation.dat' ], '\t', 1, 0 );
% めり込み
mat_12 = dlmread( [ datpath, '/', timepath, '_', '12_Length_ForMovie.dat' ], '\t', 1, 0 );
% 運動量
mat_13 = dlmread( [ datpath, '/', timepath, '_', '13_Momentum_ForMovie.dat' ], '\t', 1, 0 );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 図の作成 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{
% まとめる
mat = [ mat_02(:,1), ...
        mat_02(:,2:1+9-1), rad2deg( mat_02(:,10:end) ), ...
        mat_03(:,2:end), ...
        mat_04(:,2:2+16-1), rad2deg( mat_04(:,18:18+8-1) ), ...mat_04(:,25:27), rad2deg( mat_04(:,28:30) ), mat_04(:,31:33), rad2deg( mat_04(:,34:36) ), ... % vel_desはとりあえず図にしない
        mat_05(:,2:2+9-1), rad2deg( mat_05(:,11:end) ), ...
        mat_06(:,2:end), ...
        mat_07(:,2:end), ...
        mat_08(:,2:end), ...
        mat_09(:,2:end), ...
        rad2deg( mat_10(:,2) ), mat_10(:,3:end), ...
        mat_11(:,2:2+12-1), rad2deg( mat_11(:,14:14+7-1) ), mat_11(:,21:21+6-1), ...
        mat_12(:,2:end), ...
        mat_13(:,2:end)  ];


% 変数の列数
var_size = [ 1;   % time
             3;3;3; 3;3;3;   % 02 -- 18コ
             3;3;3;3; 3;3;3;3; 3;3;3;3;   % 03 -- 36コ
             2;2;2;2; 2;2;2;2; 8;%3;3;3;3;   % 04 -- 36
             3;3;3; 3;3;3;3;   % 05 -- 21
             3;3;3;3; 3;3;3;3; 3;3;3;3;   % 06 -- 36
             3;3;3;3; 3;3;3;3; 3;3;3;3; 3;3;3;3; 8;   % 07 -- 56
             3; 3;   % 08 -- 6
             4; 4; 1;   % 09 -- 9
             1; 9; 9; 9; 9;   % 10 -- 37
             3;3; 3;3; 3;1;3; 3;3;   % 11 -- 36
             4; 4; 4;   % 12 -- 12
             3;3; 3;3; 3;3;  3;3; 3;3; 3;3 ];   % 13 -- 36
% 変数名
var_name = { 'time';
             'SV\_d.R0';'SV\_d.v0';'SV\_d.vd0';  'SV\_d.Q0';'SV\_d.w0';'SV\_d.wd0'; % 02
             'POS\_eL1';'POS\_eL2';'POS\_eR1';'POS\_eR2';  'Qe\_degL1';'Qe\_degL2';'Qe\_degR1';'Qe\_degR2';  'curPosBP3\_L1';'curPosBP3\_L2';'curPosBP3\_R1';'curPosBP3\_R2'; % 03
             'POS\_j1';'POS\_j2';'POS\_j3';'POS\_j4';'POS\_j5';'POS\_j6';'POS\_j7';'POS\_j8';  'SV\_d.q';%'ve1\_des';'we1\_des';'ve2\_des';'we2\_des'; % 04
             'SV\_ts.R0';'SV\_ts.v0';'SV\_ts.vd0';  'SV\_ts.Q0';'SV\_ts.w0';'SV\_ts.wd0';'w_dtots'; % 05
             'shikaku1';'shikaku2';'shikaku3';'shikaku4';  'curPosAP3\_L1';'curPosAP3\_L2';'curPosAP3\_R1';'curPosAP3\_R2';  'curPosAP3\_vel\_L1';'curPosAP3\_vel\_L2';'curPosAP3\_vel\_R1';'curPosAP3\_vel\_R2'; % 06
             'FR\_N\_L1';'FR\_N\_L2';'FR\_N\_R1';'FR\_N\_R2';  'FR\_T\_L1';'FR\_T\_L2';'FR\_T\_R1';'FR\_T\_R2';  'FR\_L1';'FR\_L2';'FR\_R1';'FR\_R2';  'T\_d3\_L1';'T\_d3\_L2';'T\_d3\_R1';'T\_d3\_R2';  'SV\_d.tau'; % 07
             'SV\_ts.F0';  'SV\_ts.T0'; % 08
             'contactflag';  'flag';'catchcount'; % 09
             'q0';'contact\_flag\_L1';'contact\_flag\_L2';'contact\_flag\_R3';'contact\_flag\_R4'; % 10
             'Obs1';'Obs2';  'est\_P';'est\_V';'est\_Q';'est\_q';'est\_W';  'est\_geo\_center';'ts\_geo'; % 11
             'l\_x';  'delta';'deltaVel'; % 12
             'P\_d';'L\_ts';'P\_ts';'L\_d';'P';'L';  'P\_d\_2';'L\_d\_2';'P\_ts\_2';'L\_ts\_2';'P\_2';'L\_2' }; % 13
% ラベル名
var_label = { 'Time [s]';
              'Base Position [m]';'Base Velocity [m/s]';'Base Acceleration [m/s\^2]';  'Base Orientation [deg]';'Base Angular Velocity [deg/s]';'Base Angular Acceleration [deg/s\^2]';
              'End-Effector L1 Position [m]';'End-Effector L2 Position [m]';'End-Effector R1 Position [m]';'End-Effector R2 Position [m]';
                  'End-Effector L1 Orientation [deg]';'End-Effector L2 Orientation [deg]';'End-Effector R1 Orientation [deg]';'End-Effector R2 Orientation [deg]';
                  'Contact Point L1 Position(Robot) [m]';'Contact Point L2 Position(Robot) [m]';'Contact Point R1 Position(Robot) [m]';'Contact Point R2 Position(Robot) [m]';
              'Joint1 Position [m]';'Joint2 Position [m]';'Joint3 Position [m]';'Joint4 Position [m]';'Joint5 Position [m]';'Joint6 Position [m]';'Joint7 Position [m]';'Joint8 Position [m]';
                  'Joints Angles [deg]';%'Desired Velocity1 [m/s]';'Desired Angular Velocity1 [deg/s]';'Desired Velocity2 [m/s]';'Desired Angular Velocity2 [deg/s]';
              'Target Position [m]';'Target Velocity [m/s]';'Target Acceleration [m/s\^2]';  'Target Orientation [deg]';'Target Angular Velocity [deg/s]';'Target Angular Acceleration [deg/s\^2]';'Relative Angular Velocity [deg/s]';
              'Vertex1 Position [m]';'Vertex2 Position [m]';'Vertex3 Position [m]';'Vertex4 Position [m]';
                  'Contact Point L1 Position(Target) [m]';'Contact Point L2 Position(Target) [m]';'Contact Point R1 Position(Target) [m]';'Contact Point R2 Position(Target) [m]';
                  'Contact PointL1 Velocity(Target) [m]';'Contact Point L2 Velocity(Target) [m]';'Contact Point R1 Velocity(Target) [m]';'Contact Point R2 Velocity(Target) [m]';
              'Normal Force L1 [N]';'Normal Force L2 [N]';'Normal Force R1 [N]';'Normal Force R2 [N]';'Tangential Force L1 [N]';'Tangential Force L2 [N]';'Tangential Force R1 [N]';'Tangential Force R2 [N]';
                  'External Contact Force L1 [N]';'External Contact Force L2 [N]';'External Contact Force R1 [N]';'External Contact Force R2 [N]';'External Torque L1 [Nm]';'External Torque L2 [Nm]';'External Torque R1 [Nm]';'External Torque R2 [Nm]';'Motor Torque [Nm]';
                  'External Force [N]';  'External Torque [Nm]';
              'Contact Flags';'flags';  'Catch Count [s]';
              'Target Orientation(0～π/2) [deg]';  'Contact Flags L1';'Contact Flags L2';'Contact Flags R1';'Contact Flags R2';
              'Observation Point 1 Position [m]';'Observation Point 2 Position [m]';  'Estimation Center of Mass Position [m]';'Estimation Center of Mass Velocity [m/s]';'Estimation Orientation [deg]';'Estimation Orientation(0～π/2) [deg]';'Estimation Angular Velocity [deg/s]';
                  'Estimation Geometric Center Position [m]';'Geometric Center Position [m]';
              'Contact Length From Geometric Center(Target) [m]';  'Virtual Penetrations [m]';'Virtual Penetration Velocities [m/s]';
              'Robot Linear momentum [Ns]';'Robot Angular momentum [Nms]';'Target Linear momentum [Ns]';'Target Angular momentum [Nms]';'System Linear momentum [Ns]';'System Angular momentum [Nms]';
                  'Robot Linear momentum [Ns]';'Robot Angular momentum [Nms]';'Target Linear momentum [Ns]';'Target Angular momentum [Nms]';'System Linear momentum [Ns]';'System Angular momentum [Nms]' };
% ファイル名
file_name = { '';
              'ベース重心位置';'ベース重心速度';'ベース重心加速度';  'ベース姿勢';'ベース角速度';'ベース角加速度'; % 02
              '手先L1位置';'手先L2位置';'手先R1位置';'手先R2位置';  '手先L1姿勢';'手先L2姿勢';'手先R1姿勢';'手先R2姿勢';  '接触位置L1(ロボット座標系)';'接触位置L2(ロボット座標系)';'接触位置R1(ロボット座標系)';'接触位置R2(ロボット座標系)'; % 03
              '関節1位置';'関節2位置';'関節3位置';'関節4位置';'関節5位置';'関節6位置';'関節7位置';'関節8位置';  '関節角度';%'目標手先1並進速度';'目標手先1角速度';'目標手先2並進速度';'目標手先2角速度'; % 04
              'ターゲット重心位置';'ターゲット重心速度';'ターゲット重心加速度';  'ターゲット姿勢';'ターゲット角速度';'ターゲット角加速度';'ベースから見たターゲット角速度'; % 05
              'ターゲット頂点1';'ターゲット頂点2';'ターゲット頂点3';'ターゲット頂点4';  '接触位置L1(ターゲット座標系)';'接触位置L2(ターゲット座標系)';'接触位置R1(ターゲット座標系)';'接触位置R2(ターゲット座標系)';  '接触位置L1速度(ターゲット座標系)';'接触位置L2速度(ターゲット座標系)';'接触位置R1速度(ターゲット座標系)';'接触位置R2速度(ターゲット座標系)'; % 06
              '法線力L1(ロボット)';'法線力L2(ロボット)';'法線力R1(ロボット)';'法線力R2(ロボット)';  '接線力L1(ロボット)';'接線力L2(ロボット)';'接線力R1(ロボット)';'接線力R2(ロボット)';  '接触力L1(ロボット)';'接触力L2(ロボット)';'接触力R1(ロボット)';'接触力R2(ロボット)';  '接触トルクL1(ロボット)';'接触トルクL2(ロボット)';'接触トルクR1(ロボット)';'接触トルクR2(ロボット)';  '軸トルク'; % 07
              '外力(ターゲット)';  '外トルク(ターゲット)'; % 08
              '接触フラグ';'フラグ';  '捕獲秒数'; % 09
              'ターゲット姿勢q0';  'ターゲット領域フラグL1';'ターゲット領域フラグL2';'ターゲット領域フラグR1';'ターゲット領域フラグR2'; % 10
              '観測点位置1';'観測点位置2';  '推定重心位置';'推定重心速度';'推定姿勢';'推定姿勢q0';'推定角速度';'推定幾何的中心位置';'幾何的中心位置'; % 11
              'ターゲット幾何的軸から接触点までの距離';  'めり込み量';'めり込み速度'; % 12
              'ロボット運動量';'ロボット角運動量';'ターゲット運動量';'ターゲット角運動量';'系全体の運動量';'系全体の角運動量';  'ロボット運動量2';'ロボット角運動量2';'ターゲット運動量2';'ターゲット角運動量2';'系全体の運動量2';'系全体の角運動量2' }; % 13

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% グラフ描画・保存 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

j = 2;
for i = 2 : length( var_size )
i
%%% 見たい図を制限するにはココを
if i < 99

%%% 図設定
 figure( i )
 hold on; box on; grid on; grid minor;
%  title( var_name( i ), 'FontName', 'メイリオ', 'FontSize', FST );   % タイトルをvar_nameから取り出す
 title( file_name( i ), 'FontName', 'メイリオ', 'FontSize', FST );   % タイトルをvar_nameから取り出す(日本語)
 set( gcf, 'Position', Position, 'Color', 'white' );set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );


%%% y座標を調整
  if     var_size( i ) == 4 && max( max( mat( :, j : j + 3 ) ) ) >= 1   % 接触フラグ(1~4)のみ，y座標を調整してグラフを見やすくしている
     plot( mat( :, 1 ), mat( :, j + 0 ) - 3*1/2*0, ...
           mat( :, 1 ), mat( :, j + 1 ) - 3*1/2*1, ...
           mat( :, 1 ), mat( :, j + 2 ) - 3*1/2*2, ...
           mat( :, 1 ), mat( :, j + 3 ) - 3*1/2*3, 'linewidth', sen );
     legend( 'L1', 'L2', 'R1', 'R2' );
     ylim([ -5.5, 2.0 ]);
  elseif var_size( i ) == 9   % 領域フラグのみ，y座標を調整してグラフを見やすくしている
     plot( mat( :, 1 ), mat( :, j + 0 )/111 - 3*1/2*0, ...
           mat( :, 1 ), mat( :, j + 1 )/111 - 3*1/2*1, ...
           mat( :, 1 ), mat( :, j + 2 )/111 - 3*1/2*2, ...
           mat( :, 1 ), mat( :, j + 3 )/111 - 3*1/2*3, ...
           mat( :, 1 ), mat( :, j + 4 )/111 - 3*1/2*4, ...
           mat( :, 1 ), mat( :, j + 5 )/111 - 3*1/2*5, ...
           mat( :, 1 ), mat( :, j + 6 )/111 - 3*1/2*6, ...
           mat( :, 1 ), mat( :, j + 7 )/111 - 3*1/2*7, ...
           mat( :, 1 ), mat( :, j + 8 )/111 - 3*1/2*8, 'linewidth', sen );
     legend( '1,1', '2,2', '3,3', '4,4', '1,2', '2,3', '3,4', '4,1', '0,0' );
     ylim([ -13, 2 ]);
  else   % それ以外は時間とその変数のサイズ分の列をプロット
%      if i == 30
%           plot( tmp( :, 1 ), tmp( :, j + 3), ...
%                 tmp( :, 1 ), tmp( :, j + 4), ...
%                 tmp( :, 1 ), tmp( :, j + 8), ...
%                 tmp( :, 1 ), tmp( :, j + 9), 'linewidth', sen );
%      elseif i == 6 || i == 39
%           plot( tmp( :, 1 ), tmp( :, j + 2), 'linewidth', sen );
%      else
          plot( mat( :, 1 ), mat( :, j : ( j + var_size(i) - 1 ) ), 'linewidth', sen );
%      end
%    if max( max( tmp( :, j : ( j + var_size(i) - 1 ) ) ) ) == 0
%    else
% %     if max( max( tmp( :, j : ( j + var_size(i) - 1 ) ) ) ) <  1e-05
% %      ylim([ -0.001, 0.001 ]);
% %     end
%    end
  end


%%% ラベル
 xlabel( var_label( 1 ) ); ylabel( var_label( i ) );   % ラベルをvar_labelから取り出す


%%% 凡例
  if     var_size( i ) == 2   % 列数が2のものはx,y座標
     legend( 'x', 'y' );
  elseif var_size( i ) == 3   % 列数が3のものはx,y,z座標(回転も含める)
     legend( 'x', 'y', 'z' );
  elseif var_size( i ) == 8   % 列数が10のものは関節の変数
     legend( 'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7', 'j8' );
  end
     xlim([ mat(1,1), mat(end,1) ]);   % 定義域


%%% ファイル名を揃える
  if   i < 10   
      num = [ '00', num2str( i ) ];
  elseif i >= 10 && i < 100
      num = [ '0', num2str( i ) ];
  else
      num = num2str( i );
  end


%%% 保存
 saveas( figure(i), [ figpath, '/', timepath, '_', num, '_', char( file_name( i ) ), '.fig' ] );   % figファイルとpngファイルとして保存
 saveas( figure(i), [ pngpath, '/', timepath, '_', num, '_', char( file_name( i ) ), '.png' ] );

end

 j = j + var_size(i);    % 次の変数に進む
end

%}

%}   % グラフ作成終了


close all
   % 結果表示終了


end 
end 


end
end
end
end
end
end
end
end
end   % zetaシミュレーションループ終了



%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% シミュレーション時間の計測と表示 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% シミュレーション全体時間 単位:秒
ntime = cputime - startCPUT;

nhour = floor( ntime / 3600 );   % 単位:時間 各要素以下の最も近い整数に丸める
nmin = floor( ( ntime - nhour * 3600 ) / 60 );   % 単位:分 残りの分，整数に丸める
nsec = ntime - nhour * 3600 - nmin * 60;   % 単位:秒 残りの秒，整数に丸める

% 結果表示
fprintf( '\n\n %s %s', '開始時間 :', datestr( startT, 31 ) );
fprintf( '\n %s %s', '終了時間 :', datestr( clock, 31 ) );
fprintf( '\n %s %d %s %02d %s %04.1f %s \n\n\n', '計算所要時間 :', nhour, ' 時間 ', nmin, ' 分 ', nsec, ' 秒 ' );

clear
close all
%%% EOF
%