%%%%%%%%%% SpaceDyn サンプルプログラム  3リンク双腕宇宙ロボットの接触シミュレーション(手先速度制御＋ベースフリー) ターゲット丸型
%%%%%%%%%% 結果表示(グラフ描画，動画作成)あり
%%%%%%%%%% DualArm_TargetMaru_Tracking_Contact  2017年11月28日作成
%%%%%%%%%% ～1 → leftarm, ～2 → rightarm
%%
clc
clear all
close all 

global Ez
global d_time
global Gravity


startT = clock();
startCPUT = cputime;
tsxi =  0;%0.1;
for tsyi =  0.4164 %0.37%0.37;%
tszi =  0;
for tswi = 0.12%0.08 %0.0000001   %0.2   %1 
for phase5kyori = 1.4 %0.8:0.1:1.5;0.76;
for change5 =5 %2.53    %2.53:0.01:2.54%2.53の方いいかも        ←Phase5に切り替える時間
for friction = 0.01%0.69   %0.69:-0.01:0.6           

fclose('all')    
zeta = [ 0.01,5, tswi]';

d_timename = strcat( 'dt=', num2str( zeta(1,1) ) );
endtimename = strcat( 'et=', num2str( zeta(2,1) ) );
tmwname = strcat( 'tmω=', num2str( tswi ) );
masatukeisu = strcat( 'cof=', num2str( friction ) );
phase5kyoriname = strcat( 'phase5kyori=', num2str( phase5kyori ) ); 
change5name = strcat( 'change5=', num2str( change5 ) );
under = '_';

zetaname = strcat( d_timename, under, endtimename, under, tmwname, under, masatukeisu, under, phase5kyoriname, under, change5name);

minus_time = 2;
catchlimit = 20;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%
%%%%%%%%%%%%%%%%%%%% ロボット・ターゲットLP設定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L = 2 / sqrt(3) *0.07;
theta = pi/5;
te1 = L * sin(theta); %0.0475

% ロボットリンクパラメータ
% LP_d = DualArm_3dim_LP0801(te1) ;     %DualArm_3dim_LP0801(te1)を呼び出してリンクパラメータLPに格納     3dimのロボット腕の関節が、片方に何故か6関節設定してあり、そのうちの１～3関節がほぼ同一位置（根本）にある。　
LP_d = DualArm_3dim_LP0801_v4(te1) ;  
SV_d = DualArm_3dim_SV( LP_d );     %DualArm_3dim_SV( LP_d )を呼び出して状態変数SVに格納   同時にサイズを決めている

% ターゲットリンクパラメータ
LP_ts = TargetBall_TrackingContact_LP();
SV_ts = TargetBall_TrackingContact_SV( LP_ts );

  HH_d =  zeros( 6 + LP_d.num_q );
  Hb_d =  HH_d( 1:6, 1:6 );
  Hbm_d = HH_d( 1:6, 7:end );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% フォルダパス指定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% フォルダ作成
timepath = datestr( now, 'yyyy-mmdd-HHMMSS' );
datepath = [ 'C:\MATLAB\Dual-Arm\3_dimention\Target_Box\Box_output\' datestr( now, 'yyyy-mmdd' ) '\' timepath ];
path = strcat( datepath, '_Box_', zetaname );
datfile = [ path, '\dat'];
figfile = [ path, '\fig'];
pngfile = [ path, '\png'];
movfile = [ path, '\mov'];
mkdir( datfile );
mkdir( figfile );
mkdir( pngfile );
mkdir( movfile );

% パス設定
datpath = strcat( path, '\dat' );
figpath = strcat( path, '\fig' );
pngpath = strcat( path, '\png' );
movpath = strcat( path, '\mov' );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%S%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ストップウォッチON %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% シミュレーションにかかる時間の計測
endtime = zeta(2,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 関節の単位回転軸ベクトル %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ez = [ 0 0 1 ]';
d_time = zeta(1,1); % シミュレーション1step当たりの時間
d_time_tmp = d_time;
Gravity = [ 0 0 0 ]'; % 重力（地球重力は Gravity = [0 0 -9.8]）

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 係数設定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 剛性係数，粘性係数設定
kw = 380; %980;
%kw=999;
cw = 10;%0.65;%0.2; %0.65; %8.7;%60.0; %
% 摩擦係数固定値
cof = 0.185;% 0.6;

%%石井さん摩擦データ
% kk = [ 900  920  980  1057  1230 ];
% cc = [ 6.0  6.8  8.7  10.9  15.5 ];
% myu = [ 0.0005  0.112  0.185  0.238  0.298 ];

% 目標反発係数値
% eR_des = 0;
% 目標減衰比
% zeta_des = 1;

% % 分解加速度制御ゲイン
%     Kp = 20000;%10000;   % 20000;
%     Kd = 500;%1000;   % 500;
% %     Kp_2 = 1;3;   % 0.8
% %     Kd_2 = 0.3;   % 0.3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 長さパラメータ設定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

base_arm_yoko_1 = abs( LP_d.c0(1,1) );  %abs()は絶対値を返す関数
base_arm_yoko_2 = abs( LP_d.c0(1,4) );
base_arm_tate_1 = abs( LP_d.c0(2,1) );
base_arm_tate_2 = abs( LP_d.c0(2,4) );

r_tip = 0.01; %0.01; %チェイサ先端球半径 1cm
halftarget = 0.075;         % 0.125;% 0.1;  %ターゲット半径 10cm or 12.5cm       ←ターゲット半径ではなく、横の半辺
halftargetz = 0.055;        % 0.125;% 0.1;  %ターゲット半径 10cm or 12.5cm      ←高さの半辺
rbt_halftate = 0.15;%0.04; %0.1;
r_base = 0.03;   % ベースの重心描画用円

% main32には下記がある
% halftarget = halftarget*1.05;% 0.125;% 0.1;  %ターゲット半径 10cm or 12.5cm
% halftargetz = halftargetz*1.05;% 0.125;% 0.1;  %ターゲット半径 10cm or 12.5cm

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 関節制御で使うパラメータの定義 %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% q_joints_kotei = SV_d.q;
flag_kotei = 0;
flagphase5 = 0;
x_L_tmp = 0;
x_R_tmp = 0;
D = 0;
ts3m = 0;
ts6m = 0;

% 関節指令値
q_des =   SV_d.q;
qd_des =  SV_d.qd;
qdd_des = SV_d.qdd;
n_figure = 1;
catchtime = 0;
hokakutime = 0;

minimum = 1e-3;
Phase = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 初期値設定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% ロボット初期値設定 %%%%%%%%%%
% ベースからnum_eで指定された手先までを結ぶ関節(リンク)を求める  1アーム多リンクなら1, リンクの数を表す
num_eL = 1;       
jointsL = j_num( LP_d, num_eL );   %左
num_eR = 2;
jointsR = j_num( LP_d, num_eR );   %右

% ベースの初期位置・姿勢・速度・角速度
SV_d.R0 = [ 0 0 0 ]';
SV_d.Q0 = [ 0 0 pi / 180 * 0 ]';
SV_d.A0 = rpy2dc( SV_d.Q0 )';
SV_d.v0 = [ 0 0 0 ]';
SV_d.w0 = [ 0 0 0 ]';
SV_d = calc_aa( LP_d, SV_d );
SV_d = calc_pos( LP_d, SV_d );

% ロボットの初期関節角度を設定
% q_1 = [  pi/3 -pi/6 -2*pi/3 ]';   %左側   % q_L = [  pi()/3 -pi()/2 -pi()/3 ]';
% q_2 = [ -pi/3  pi/6  2*pi/3 ]';   %右側   % q_R = [ -pi()/3  pi()/2  pi()/3 ]';
q_1 = [  0 0 pi/3 -pi/2 -pi/3 0 ]'; 
q_2 = -q_1;
% q_1 = [  0 0 pi/3 -pi/2 -pi/3+pi/4 pi/4 pi 0 0]';   %左側   % q_L = [  pi()/3 -pi()/2 -pi()/3 ]';
% q_2 = -q_1;
% q_2(7) = 0;
SV_d.q = [ q_1' q_2' ]';
q_tmp = SV_d.q;

%%%%%%%%%% ターゲット初期値設定 %%%%%%%%%%
% ターゲットの姿勢・速度・角速度
SV_ts_Q0_0 = [0 0 0]';   % ターゲット初期角度 
% SV_ts_Q0_0 = [ pi/2 pi/4 pi/4]'; 
SV_ts.Q0 = SV_ts_Q0_0 - [ 0 0 tswi*minus_time ]';   % time=-minus_timeのときのターゲット初期姿勢
SV_ts.v0 = [ 0 0 0 ]';   % ターゲット初期並進速度
SV_ts.w0 = [ 0.5*tswi 0.5*tswi tswi ]';%[ 0 zeta(3,1) 0.0000001 ]';   % ターゲット初期角速度
% SV_ts.w0 = [ 0 0 tswi ]';
% SV_ts.w0 = [ zeta(3,1) zeta(3,1) zeta(3,1) ]';   % ターゲット初期角速度
% SV_ts.Q0 = SV_ts_Q0_0 + [ -2.1462 0.9974 -2.1462 ]';   % time=-minus_timeのときのターゲット初期姿勢
SV_ts.A0 = rpy2dc( SV_ts.Q0 )';
TB0 = rpy2dc( SV_ts.Q0 );

% ターゲットの初期位置
SV_ts.R0(1,1) = tsxi;
SV_ts.R0(2,1) = tsyi;
SV_ts.R0(3,1) = tszi;
SV_tsR0_tmp = SV_ts.R0;
%%%%%%%%%% 初期手先位置・姿勢の計算 %%%%%%%%%%
% 順動力学計算
SV_d =  f_dyn_rk2( LP_d, SV_d );
SV_ts = f_dyn_rk2( LP_ts, SV_ts );
% 各リンクの座標変換行列(方向余弦行列)の計算（リンクi → 慣性座標系）
SV_d = calc_aa( LP_d, SV_d );
% 各リンク重心の位置
SV_d = calc_pos( LP_d, SV_d );
% アーム手先位置・姿勢
[ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   %左側
[ POS_eR, ORI_eR ] = f_kin_e( LP_d, SV_d, jointsR );   %右側

% 全関節位置姿勢の計算
[ POS_j_L, ORI_j_L ] = f_kin_j( LP_d, SV_d, jointsL );   % 左側
[ POS_j_R, ORI_j_R ] = f_kin_j( LP_d, SV_d, jointsR );   % 右側

% 各関節の位置 x,y
POS_jL1 = POS_j_L( 1:3, 1 );   %左側　関節1（根本）の位置代入　　
POS_jL2 = POS_j_L( 1:3, 2 );   %なぜか関節1とほぼ同一位置にある関節　存在価値なし
POS_jL3 = POS_j_L( 1:3, 3 );   %なぜか関節1とほぼ同一位置にある関節　存在価値なし
POS_jL4 = POS_j_L( 1:3, 4 );   %左側　関節4 の位置代入　　
POS_jL5 = POS_j_L( 1:3, 5 );   %左側　関節5 の位置代入　　
POS_jL6 = POS_j_L( 1:3, 6 );   %左側手先付け根の関節位置　この情報のみDesire関数で利用
d_POS_jL6 = POS_jL6;

POS_jR1 = POS_j_R( 1:3, 1 );   % 右側　関節1の位置代入
POS_jR2 = POS_j_R( 1:3, 2 );   %なぜか関節1とほぼ同一位置にある関節　存在価値なし
POS_jR3 = POS_j_R( 1:3, 3 );   %なぜか関節1とほぼ同一位置にある関節　存在価値なし
POS_jR4 = POS_j_R( 1:3, 4 );   %右側　関節4 の位置代入　　
POS_jR5 = POS_j_R( 1:3, 5 );   %右側　関節5 の位置代入　　
POS_jR6 = POS_j_R( 1:3, 6 );   %%左側手先付け根の関節位置　この情報のみDesire関数で利用
d_POS_jR6 = POS_jR6;

% 関節位置保存
POS_jL6_tmp = POS_jL6;
POS_jR6_tmp = POS_jR6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 変数の初期化 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 手先位置姿勢の計算と手先角度のオイラー角表現
%D = abs( L * ( cos( theta ) - sin( theta ) ) + r_tip * sqrt(2) );   % 手先が丁度ターゲットに触れるような長さを計算
% D =0 ; 
%D_0 = abs( L * cos( theta ) + r_tip );
% 接触位置初期化      　←接触位置というより、末端の手先位置
POS_ee1 = POS_eL + ORI_eL * [te1 -te1 0]';
POS_ee2 = POS_eL + ORI_eL * [te1 te1 * cos(60*pi()/180) te1 * sin(60*pi()/180)]';
POS_ee3 = POS_eL + ORI_eL * [te1 te1 * cos(60*pi()/180) -te1 * sin(60*pi()/180)]';
POS_ee4 = POS_eR + ORI_eR * [te1 te1 0]';
POS_ee5 = POS_eR + ORI_eR * [te1 -te1 * cos(60*pi()/180) te1 * sin(60*pi()/180)]';
POS_ee6 = POS_eR + ORI_eR * [te1 -te1 * cos(60*pi()/180) -te1 * sin(60*pi()/180)]';

POS_end=[POS_ee1 POS_ee2 POS_ee3 POS_ee4 POS_ee5 POS_ee6]; 

caging26 = 0;caging35 = 0;caging15 = 0;caging16 = 0;caging23 = 0;caging24 = 0;caging2dim = 0;caging3dim = 0;

contactflag_LR = 0;
Contact_point_L1_tmp = zeros(3,1); Contact_point_L2_tmp = zeros(3,1); Contact_point_L3_tmp = zeros(3,1);
Contact_point_R1_tmp = zeros(3,1); Contact_point_R2_tmp = zeros(3,1); Contact_point_R3_tmp = zeros(3,1);
delta_L1_tmp = 0.0000; delta_L2_tmp = 0.0000; delta_L3_tmp = 0.0000;
delta_R1_tmp = 0.0000; delta_R2_tmp = 0.0000; delta_R3_tmp = 0.0000;

distance_L1_tmp = zeros(1,6); distance_L2_tmp = zeros(1,6); distance_L3_tmp = zeros(1,6);
distance_R1_tmp = zeros(1,6); distance_R2_tmp = zeros(1,6); distance_R3_tmp = zeros(1,6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 結果書き込み用ファイルの定義 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 各種長さ
fidw_length = fopen(strcat(datpath,'\Length.dat'),'w');
% ロボット
fidw_d = fopen(strcat(datpath,'\DualArm.dat'),'w');
% ロボット
fidw_d2 = fopen(strcat(datpath,'\DualArm2.dat'),'w');
% ロボット
fidw_d3 = fopen(strcat(datpath,'\DualArm3.dat'),'w');
% ロボット
fidw_d4 = fopen(strcat(datpath,'\DualArm4.dat'),'w');
% ターゲット　←運動量
fidw_en = fopen(strcat(datpath,'\Energy.dat'),'w');
% 力，トルクなど
fidw_ft = fopen(strcat(datpath,'\ForceTorque.dat'),'w');
% 力，トルクなど
fidw_thesis = fopen(strcat(datpath,'\Thesis.dat'),'w');
% 接触フラグ
fidw_cf = fopen(strcat(datpath,'\ContactFlag.dat'),'w');
% アニメ用
fidw_movie = fopen(strcat(datpath,'\DualArm_TargetMaru_TrackingContact_ForMovie.dat'),'w');
% アニメ用
fidw_movie2 = fopen(strcat(datpath,'\DualArm_TargetMaru_TrackingContact_ForMovie2.dat'),'w');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 接触・カウントフラグ設定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ts_l = SV_ts.R0 + SV_ts.A0 * [halftarget -1 * halftarget halftarget]';
%ts_r = SV_ts.R0 + SV_ts.A0 * [-1 * halftarget halftarget -1 * halftarget]';

contactflag_tmp = 0;
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% シミュレーションループスタート %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for time = 0:d_time:(endtime + minus_time)
    display_time = (time - minus_time);

    if Phase == 1||Phase ==2 ||Phase == 3 || Phase == 4 || Phase == 5
        d_time = 0.001;
        phasetime = d_time_tmp - d_time;
%     elseif Phase == 3 || Phase == 4 || Phase == 5
%         d_time = 0.001; %0.0001;
%         phasetime = d_time_tmp - d_time;
        
%     elseif Phase == 4 || Phase == 5
%         d_time = 0.0001;
%         phasetime = d_time_tmp - d_time;

    else
        d_time = d_time_tmp;
        phasetime =0;
    end
for time = time:d_time:time + phasetime

    timer = [time display_time Phase catchtime caging3dim contactflag_LR']  

%%%%ターゲット頂点の計算
    ts1 = SV_ts.R0 + SV_ts.A0 * [halftarget halftarget halftargetz]';        %%　ターゲットの位置　＋　ターゲットの姿勢 × ターゲット半辺 (halftarget)
    ts2 = SV_ts.R0 + SV_ts.A0 * [halftarget halftarget -1 * halftargetz]';
    ts3 = SV_ts.R0 + SV_ts.A0 * [halftarget -1 * halftarget halftargetz]';
    ts4 = SV_ts.R0 + SV_ts.A0 * [halftarget -1 * halftarget -1 * halftargetz]';
    ts5 = SV_ts.R0 + SV_ts.A0 * [-1 * halftarget halftarget halftargetz]';
    ts6 = SV_ts.R0 + SV_ts.A0 * [-1 * halftarget halftarget -1 * halftargetz]';
    ts7 = SV_ts.R0 + SV_ts.A0 * [-1 * halftarget -1 * halftarget halftargetz]';
    ts8 = SV_ts.R0 + SV_ts.A0 * [-1 * halftarget -1 * halftarget -1 * halftargetz]';
    %main33による　これは掴みに行く点をあらわす
    D = 0.35 * halftarget;
    Dz= 0.1 * halftargetz;
    ts3m = SV_ts.R0 + SV_ts.A0 * [(halftarget + D)  -1 * (halftarget + D)  (halftargetz + Dz)]';
    ts6m = SV_ts.R0 + SV_ts.A0 * [-1 * (halftarget + D) (halftarget + D) -1 * (halftargetz + Dz)]';    
    
%%%接触計算
%%%ターゲット平面の定義 (行列　Surfabc　を定義。　A(m,:)は、行列m番目の行。）
    
%     Surfabc(1,1:3) = [1 1 1] /([ts1 ts2 ts3]);      %[* * *]
%     Surfabc(2,1:3) = [1 1 1] /([ts1 ts2 ts5]);      %[+ + +]
%     Surfabc(3,1:3) = [1 1 1] /([ts1 ts3 ts5]);      %[# # #]
%     Surfabc(4,1:3) = [1 1 1] /([ts2 ts4 ts8]);      %[$ $ $]
%     Surfabc(5,1:3) = [1 1 1] /([ts3 ts4 ts8]);      %[! ! !]
%     Surfabc(6,1:3) = [1 1 1] /([ts5 ts6 ts8]);      %[= = =]
%     Surfabc = Surfabc';
%     
    tsj = [ts1 ts1 ts1 ts2 ts3 ts5];
    %　         [* * *]
    %　Surfabc = ………　の6×3の行列を作っている。 逆行列にして3×6
    %　         [= = =]
    %平面１
    Surfabc(1,1) =  ( ts2(2,1)-ts1(2,1) ) * ( ts3(3,1)-ts1(3,1) ) -  ( ts3(2,1)-ts1(2,1) ) * (ts2(3,1)-ts1(3,1) );
    Surfabc(2,1) =  ( ts2(3,1)-ts1(3,1) ) * ( ts3(1,1)-ts1(1,1) ) -  ( ts3(3,1)-ts1(3,1) ) * (ts2(1,1)-ts1(1,1) );
    Surfabc(3,1) =  ( ts2(1,1)-ts1(1,1) ) * ( ts3(2,1)-ts1(2,1) ) -  ( ts3(1,1)-ts1(1,1) ) * (ts2(2,1)-ts1(2,1) );
    %平面２
    Surfabc(1,2) =  ( ts2(2,1)-ts1(2,1) ) * ( ts5(3,1)-ts1(3,1) ) -  ( ts5(2,1)-ts1(2,1) ) * (ts2(3,1)-ts1(3,1) );
    Surfabc(2,2) =  ( ts2(3,1)-ts1(3,1) ) * ( ts5(1,1)-ts1(1,1) ) -  ( ts5(3,1)-ts1(3,1) ) * (ts2(1,1)-ts1(1,1) );
    Surfabc(3,2) =  ( ts2(1,1)-ts1(1,1) ) * ( ts5(2,1)-ts1(2,1) ) -  ( ts5(1,1)-ts1(1,1) ) * (ts2(2,1)-ts1(2,1) );
    %平面３
    Surfabc(1,3) =  ( ts3(2,1)-ts1(2,1) ) * ( ts5(3,1)-ts1(3,1) ) -  ( ts5(2,1)-ts1(2,1) ) * (ts3(3,1)-ts1(3,1) );
    Surfabc(2,3) =  ( ts3(3,1)-ts1(3,1) ) * ( ts5(1,1)-ts1(1,1) ) -  ( ts5(3,1)-ts1(3,1) ) * (ts3(1,1)-ts1(1,1) );
    Surfabc(3,3) =  ( ts3(1,1)-ts1(1,1) ) * ( ts5(2,1)-ts1(2,1) ) -  ( ts5(1,1)-ts1(1,1) ) * (ts3(2,1)-ts1(2,1) );
    %平面４
    Surfabc(1,4) =  ( ts4(2,1)-ts2(2,1) ) * ( ts8(3,1)-ts2(3,1) ) -  ( ts8(2,1)-ts2(2,1) ) * ( ts4(3,1)-ts2(3,1) );
    Surfabc(2,4) =  ( ts4(3,1)-ts2(3,1) ) * ( ts8(1,1)-ts2(1,1) ) -  ( ts8(3,1)-ts2(3,1) ) * ( ts4(1,1)-ts2(1,1) );
    Surfabc(3,4) =  ( ts4(1,1)-ts2(1,1) ) * ( ts8(2,1)-ts2(2,1) ) -  ( ts8(1,1)-ts2(1,1) ) * ( ts4(2,1)-ts2(2,1) );
    %平面５
    Surfabc(1,5) =  ( ts4(2,1)-ts3(2,1) ) * ( ts8(3,1)-ts3(3,1) ) -  ( ts8(2,1)-ts3(2,1) ) * ( ts4(3,1)-ts3(3,1) );
    Surfabc(2,5) =  ( ts4(3,1)-ts3(3,1) ) * ( ts8(1,1)-ts3(1,1) ) -  ( ts8(3,1)-ts3(3,1) ) * ( ts4(1,1)-ts3(1,1) );
    Surfabc(3,5) =  ( ts4(1,1)-ts3(1,1) ) * ( ts8(2,1)-ts3(2,1) ) -  ( ts8(1,1)-ts3(1,1) ) * ( ts4(2,1)-ts3(2,1) );
    %平面６
    Surfabc(1,6) =  ( ts6(2,1)-ts5(2,1) ) * ( ts8(3,1)-ts5(3,1) ) -  ( ts8(2,1)-ts5(2,1) ) * ( ts6(3,1)-ts5(3,1) );
    Surfabc(2,6) =  ( ts6(3,1)-ts5(3,1) ) * ( ts8(1,1)-ts5(1,1) ) -  ( ts8(3,1)-ts5(3,1) ) * ( ts6(1,1)-ts5(1,1) );
    Surfabc(3,6) =  ( ts6(1,1)-ts5(1,1) ) * ( ts8(2,1)-ts5(2,1) ) -  ( ts8(1,1)-ts5(1,1) ) * ( ts6(2,1)-ts5(2,1) );
    
% halftarget = halftarget + minimum;
% halftargetz = halftargetz + minimum;

%%%%%%%%%%%%%%%%%%%% 変数初期化 %%%%%%%%%%%%%%%%%%%%
% 法線接線単位ベクトル初期化
normal_vector_L1 = zeros(3,1); normal_vector_L2 = zeros(3,1); normal_vector_L3 = zeros(3,1);
normal_vector_R1 = zeros(3,1); normal_vector_R2 = zeros(3,1); normal_vector_R3 = zeros(3,1);
tang_L1 = zeros(3,1); tang_L2 = zeros(3,1); tang_L3 = zeros(3,1);
tang_R1 = zeros(3,1); tang_R2 = zeros(3,1); tang_R3 = zeros(3,1);
% 接触位置初期化
% PointC_L1 = zeros(3,1); PointC_L2 = zeros(3,1); PointC_R1 = zeros(3,1); PointC_R2 = zeros(3,1);
% curPosAP3_L1 = zeros(3,1); curPosAP3_L2 = zeros(3,1); curPosAP3_R1 = zeros(3,1); curPosAP3_R2 = zeros(3,1);
% curPosBP3_L1 = zeros(3,1); curPosBP3_L2 = zeros(3,1); curPosBP3_R1 = zeros(3,1); curPosBP3_R2 = zeros(3,1);
% curPosAP3_tmp_L1 = zeros(3,1); curPosAP3_tmp_L2 = zeros(3,1); curPosAP3_tmp_R1 = zeros(3,1); curPosAP3_tmp_R2 = zeros(3,1);
% curPosAP3_vel_L1 = zeros(3,1); curPosAP3_vel_L2 = zeros(3,1); curPosAP3_vel_R1 = zeros(3,1); curPosAP3_vel_R2 = zeros(3,1);
Contact_point_L1 = zeros(3,1); Contact_point_L2 = zeros(3,1); Contact_point_L3 = zeros(3,1);
Contact_point_R1 = zeros(3,1); Contact_point_R2 = zeros(3,1); Contact_point_R3 = zeros(3,1);

% 方向余弦行列初期化
% TB0_L1_dash = zeros(3,3); TB0_L2_dash = zeros(3,3); TB0_R1_dash = zeros(3,3); TB0_R2_dash = zeros(3,3);

% めり込み量初期化
% delta_L1 = 0.0000; delta_L2 = 0.0000; delta_L3 = 0.0000;
% delta_R1 = 0.0000; delta_R2 = 0.0000; delta_R3 = 0.0000;
% delta_vel_L1 = 0.0000; delta_vel_L2 = 0.0000; delta_vel_L3 = 0.0000;
% delta_vel_R1 = 0.0000; delta_vel_R2 = 0.0000; delta_vel_R3 = 0.0000;

%接触力初期化
% F_NN_L1 = 0; F_NN_L2 = 0; F_NN_L3 = 0; F_NN_R1 = 0; F_NN_R2 = 0; F_NN_R3 = 0;
% FR_N_L1 = zeros(3,1); FR_N_L2 = zeros(3,1); FR_N_R1 = zeros(3,1); FR_N_R2 = zeros(3,1);
% FR_T_L1 = zeros(3,1); FR_T_L2 = zeros(3,1); FR_T_R1 = zeros(3,1); FR_T_R2 = zeros(3,1);
% FR_L1 = zeros(3,1); FR_L2 = zeros(3,1); FR_R1 = zeros(3,1); FR_R2 = zeros(3,1);

%%%%%%%%%%%%%%%%%%%% 接触・カウントフラグ設定 %%%%%%%%%%%%%%%%%%%%

% 確実に接触が起こらない条件 (ロボット重心・ターゲット重心間距離がl_hen_min or l_kaku_minより大きい時，接触は起こらない) 
l_side_min = r_tip + halftarget;   % 接触する瞬間のロボットアーム手先球中心～ターゲット中心までの距離 = 手先半径 + ターゲットの一辺/2
l_corner_min = r_tip + halftarget * sqrt(2);   % 接触する瞬間のロボットアーム手先球中心～ターゲット中心までの距離 = 手先半径 + ターゲットの対角/2

% % 接触中のループ数カウント
% contact_1_i = 0;
% contact_2_i = 0;
% countflag_1 = 1;
% countflag_2 = 1;
% % カウント
% count = 1;
% % 捕獲カウント
% catchtime = 0;

% 接触フラグ 接触1 非接触0
% contactflag_L1 = 0; contactflag_L2 = 0; contactflag_L3 = 0;
% contactflag_R1 = 0; contactflag_R2 = 0; contactflag_R3 = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 接触判定 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Contact_point：接触位置の位置座標（平面上）
% normal_vector：法線ベクトル（手先から平面に垂直なベクトル）
% relative     ：ターゲット中心座標系からみた、接触位置の相対位置座標

%返し値　contactflag
%%%%%%%%%% 左側1,2,3 %%%%%%%%%%
[ contactflag_L1, Contact_point_L1, D_close_L1, normal_vector_L1, relative_L1, curPosAP33_L1] ... 
= Box_SixTips_CONTACT_FLAG_v05( SV_ts, d_time, r_tip, halftarget, halftargetz, POS_ee1, Surfabc, distance_L1_tmp, tsj, TB0);
[ contactflag_L2, Contact_point_L2, D_close_L2, normal_vector_L2, relative_L2, curPosAP33_L2] ... 
= Box_SixTips_CONTACT_FLAG_v05( SV_ts, d_time, r_tip, halftarget, halftargetz, POS_ee2, Surfabc, distance_L2_tmp, tsj, TB0);
[ contactflag_L3, Contact_point_L3, D_close_L3, normal_vector_L3, relative_L3, curPosAP33_L3] ... 
= Box_SixTips_CONTACT_FLAG_v05( SV_ts, d_time, r_tip, halftarget, halftargetz, POS_ee3, Surfabc, distance_L3_tmp, tsj, TB0);
%%%%%%%%%% 右側3,4.5 %%%%%%%%%%
[ contactflag_R1, Contact_point_R1, D_close_R1, normal_vector_R1, relative_R1, curPosAP33_R1] ...  
= Box_SixTips_CONTACT_FLAG_v05( SV_ts, d_time, r_tip, halftarget, halftargetz, POS_ee4, Surfabc, distance_R1_tmp, tsj, TB0);
[ contactflag_R2, Contact_point_R2, D_close_R2, normal_vector_R2, relative_R2, curPosAP33_R2] ... 
= Box_SixTips_CONTACT_FLAG_v05( SV_ts, d_time, r_tip, halftarget, halftargetz, POS_ee5, Surfabc, distance_R2_tmp, tsj, TB0);
[ contactflag_R3, Contact_point_R3, D_close_R3, normal_vector_R3, relative_R3, curPosAP33_R3] ... 
= Box_SixTips_CONTACT_FLAG_v05( SV_ts, d_time, r_tip, halftarget, halftargetz, POS_ee6, Surfabc, distance_R3_tmp, tsj, TB0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 接触力計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calc_Rgで重心位置決定
Rg_d = calc_Rg( LP_d, SV_d );
%%%%%%%%%% 左側1,2,3 %%%%%%%%%%
%返し値　Force
 [ contactflag_L1, F_NN_L1, FR_N_L1, FR_T_L1, tang_L1, FR_L1, delta_L1, delta_vel_L1 ] ...
= Box_SixTips_ContactForce_v04(contactflag_L1, SV_ts, SV_tsR0_tmp, d_time,  r_tip, cof, kw, cw, ...
         Contact_point_L1, normal_vector_L1, D_close_L1, Contact_point_L1_tmp, delta_L1_tmp);                        
 [ contactflag_L2, F_NN_L2, FR_N_L2, FR_T_L2, tang_L2, FR_L2, delta_L2, delta_vel_L2 ] ...
= Box_SixTips_ContactForce_v04(contactflag_L2, SV_ts, SV_tsR0_tmp, d_time,  r_tip, cof, kw, cw, ...
         Contact_point_L2, normal_vector_L2, D_close_L2, Contact_point_L2_tmp, delta_L2_tmp);    
 [ contactflag_L3, F_NN_L3, FR_N_L3, FR_T_L3, tang_L3, FR_L3, delta_L3, delta_vel_L3 ] ...
= Box_SixTips_ContactForce_v04(contactflag_L3, SV_ts, SV_tsR0_tmp, d_time,  r_tip, cof, kw, cw, ...
         Contact_point_L3, normal_vector_L3, D_close_L3, Contact_point_L3_tmp, delta_L3_tmp);
%%%%%%%%%% 右側3,4.5 %%%%%%%%%%
 [ contactflag_R1, F_NN_R1, FR_N_R1, FR_T_R1, tang_R1, FR_R1, delta_R1, delta_vel_R1 ] ...
= Box_SixTips_ContactForce_v04(contactflag_R1, SV_ts, SV_tsR0_tmp, d_time,  r_tip, cof, kw, cw, ...
         Contact_point_R1, normal_vector_R1, D_close_R1, Contact_point_R1_tmp, delta_R1_tmp);                        
 [ contactflag_R2, F_NN_R2, FR_N_R2, FR_T_R2, tang_R2, FR_R2, delta_R2, delta_vel_R2 ] ...
= Box_SixTips_ContactForce_v04(contactflag_R2, SV_ts, SV_tsR0_tmp, d_time,  r_tip, cof, kw, cw, ...
         Contact_point_R2, normal_vector_R2, D_close_R2, Contact_point_R2_tmp, delta_R2_tmp);                         
 [ contactflag_R3, F_NN_R3, FR_N_R3, FR_T_R3, tang_R3, FR_R3, delta_R3, delta_vel_R3 ] ...
= Box_SixTips_ContactForce_v04(contactflag_R3, SV_ts, SV_tsR0_tmp, d_time,  r_tip, cof, kw, cw, ...
         Contact_point_R3, normal_vector_R3, D_close_R3, Contact_point_R3_tmp, delta_R3_tmp);
                                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 接触トルクの計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% 左側1,2,3 %%%%%%%%%%
T_t3_L1 = cross( SV_ts.A0 \ ( Contact_point_L1 - POS_eL ), SV_ts.A0 \ -FR_L1 ); %curPosAP33_L1 * TB0 * ( -FR_L1 );   % curPosAP33(交代行列) * A0'(ターゲットの方向余弦行列) * -F3(慣性座標系から見たターゲットが受ける接触力);   zeros(3,1);
T_t3_L2 = cross( SV_ts.A0 \ ( Contact_point_L2 - POS_eL ), SV_ts.A0 \ -FR_L2 ); %curPosAP33_L2 * TB0 * ( -FR_L2 ); 
T_t3_L3 = cross( SV_ts.A0 \ ( Contact_point_L3 - POS_eL ), SV_ts.A0 \ -FR_L3 ); %curPosAP33_L3 * TB0 * ( -FR_L3 ); 
T_d3_L1 = cross( SV_d.A0 \ ( Contact_point_L1 - POS_eL ), SV_d.A0 \ FR_L1 );   % zeros(3,1); curPosBP33_L1 * DB0_L1 * (  FR_L1 );   % curPosBP33 * B0' * (F3);
T_d3_L2 = cross( SV_d.A0 \ ( Contact_point_L2 - POS_eL ), SV_d.A0 \ FR_L2 );   % zeros(3,1); curPosBP33_L2 * DB0_L2 * (  FR_L2 );
T_d3_L3 = cross( SV_d.A0 \ ( Contact_point_L3 - POS_eL ), SV_d.A0 \ FR_L3 );
    %慣性座標系から見た接触位置
    % PointC_L?  =  SV_d.A0 \ (Contact_point(1:3,i,j) - POS_eL)
    % SV_d.Te(:,6)  =  cross(  SV_d.A0 \ (Contact_point(1:3,i,j) - POS_eL) , SV_d.A0 \ -force) + cross(  SV_d.A0 \ (Contact_point(1:3,i,j) - POS_eL) , SV_d.A0 \ -friforce);
    % SV_d.Te(:,12) =  cross(  SV_d.A0 \ (Contact_point(1:3,i,j) - POS_eR) , SV_d.A0 \ -force) + cross(  SV_d.A0 \ (Contact_point(1:3,i,j) - POS_eR) , SV_d.A0 \ -friforce);
%%%%%%%%%% 右側3,4,5 %%%%%%%%%%
T_t3_R1 = cross( SV_ts.A0 \ ( Contact_point_R1 - POS_eR ), SV_ts.A0 \ FR_R1 );%curPosAP33_R1 * TB0 * ( -FR_R1 );
T_t3_R2 = cross( SV_ts.A0 \ ( Contact_point_R2 - POS_eR ), SV_ts.A0 \ FR_R2 );%curPosAP33_R2 * TB0 * ( -FR_R2 );
T_t3_R3 = cross( SV_ts.A0 \ ( Contact_point_R3 - POS_eR ), SV_ts.A0 \ FR_R3 );%curPosAP33_R3 * TB0 * ( -FR_R3 );
T_d3_R1 = cross( SV_d.A0 \ ( Contact_point_R1 - POS_eR ), SV_d.A0 \ FR_R1 );%cross( ( Contact_point_R1 - POS_eR ), FR_R1 );   % zeros(3,1); curPosBP33_R1 * DB0_R1 * (  FR_R1 );
T_d3_R2 = cross( SV_d.A0 \ ( Contact_point_R2 - POS_eR ), SV_d.A0 \ FR_R2 );%cross( ( Contact_point_R2 - POS_eR ), FR_R2 );   % zeros(3,1); curPosBP33_R2 * DB0_R2 * (  FR_R2 );
T_d3_R3 = cross( SV_d.A0 \ ( Contact_point_R3 - POS_eR ), SV_d.A0 \ FR_R3 );%cross( ( Contact_point_R3 - POS_eR ), FR_R3 ); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% ターゲット，ロボットが受ける力とトルクの計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_L = FR_L1 + FR_L2 + FR_L3;
FR_R = FR_R1 + FR_R2 + FR_R3;
T_d3_L = T_d3_L1 + T_d3_L2 + T_d3_L3;
T_d3_R = T_d3_R1 + T_d3_R2 + T_d3_R3;

SV_d.Fe(:,6) = FR_L;    % ロボット(左)が受ける力 (左向きにする)
SV_d.Te(:,6) = T_d3_L;   % ロボット(左)が受けるトルク
SV_d.Fe(:,12) = FR_R;    % ロボット(右)が受ける力 (右向きにする)
SV_d.Te(:,12) = T_d3_R;   % ロボット(右)が受けるトルク

SV_ts.F0 =  -( FR_L + FR_R ); 
SV_ts.T0 =  T_t3_L1 + T_t3_L2 + T_t3_L3 + T_t3_R1 + T_t3_R2 + T_t3_R3;

Fh_L = [ SV_d.Fe(:,6)'  SV_d.Te(:,6)' ]';   % 力・トルクをまとめる
Fh_R = [ SV_d.Fe(:,12)' SV_d.Te(:,12)']';
% Fb = [ SV_d.F0' SV_d.T0' ]';

%%%保存値の更新
delta_L1_tmp = delta_L1; delta_L2_tmp = delta_L2; delta_L3_tmp = delta_L3;
delta_R1_tmp = delta_R1; delta_R2_tmp = delta_R2; delta_R3_tmp = delta_R3;
delta_tmp = [delta_L1_tmp delta_L2_tmp delta_L3_tmp delta_R1_tmp delta_R2_tmp delta_R3_tmp]'; 

force_i = [F_NN_L1 F_NN_L2 F_NN_L3 F_NN_R1 F_NN_R2 F_NN_R3]'; 
distance_i = [D_close_L1 D_close_L2 D_close_L3 D_close_R1 D_close_R2 D_close_R3]'; 

Contact_point_L1_tmp = Contact_point_L1; Contact_point_L2_tmp = Contact_point_L2; Contact_point_L3_tmp = Contact_point_L3;
Contact_point_R1_tmp = Contact_point_R1; Contact_point_R2_tmp = Contact_point_R2; Contact_point_R3_tmp = Contact_point_R3;

SV_tsR0_tmp = SV_ts.R0;
contactflag_LR = [contactflag_L1 contactflag_L2 contactflag_L3 contactflag_R1 contactflag_R2 contactflag_R3]'; 
contactflag_tmp = norm(contactflag_LR);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 目標トルク制御の計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 一般化ヤコビ行列計算
J_tmp = zeros(12,12);

    JL = calc_gj( LP_d, SV_d, num_eL );   % 一般化ヤコビ行列(左腕)
    JR = calc_gj( LP_d, SV_d, num_eR );   % 一般化ヤコビ行列(右腕)
    J = [ JL' JR' ]';   % 一般化ヤコビ行列をまとめる
    Je = pinv(J);   % ヤコビの逆行列
    Jd = ( J - J_tmp ) / d_time;
    J_tmp = J;

hokakutime = 0.01;  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 目標手先速度の計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Pase1～3までの、手先の制御を行う関数（長谷さんの関数ベース）
%Phase4以降は下にパス

[ xe_des, ve_des, ae_des, POS_jL6_tmp, POS_jR6_tmp, d_POS_jL6, d_POS_jR6, VEL_jL6, VEL_jR6, ...
           d_QL_3, d_QR_7, aL, aR, q_joints, q_wrist, qd_joints, qd_wrist, delta_t, t0, Phase, catchtime] ...
= Onehand_3dim_DesiredHandMotion_v06( d_time, time, minus_time, SV_d, SV_ts, contactflag_LR, Phase, change5,  ...
                         POS_eL, POS_eR,POS_jL4, POS_jR4, POS_jL5, POS_jR5, POS_jL6, POS_jR6, POS_jL6_tmp, POS_jR6_tmp, d_POS_jL6, d_POS_jR6,...
                         D, d_QL_3, d_QR_7, aL, aR, delta_t, t0, catchtime, hokakutime ,ts1,ts2,ts3,ts4,ts5,ts6,ts3m,ts6m,caging3dim,POS_ee1,POS_ee4);
                     
%長谷さんの元のDesire関数
%Phase 1    待機時間、推定時間　
%Phase 2    計算時間　　　　　　　　　ターゲットの姿勢がpi/4になるまでの時間　td　と、両手先の目標位置を計算する
%Phase 3    制御時間　　　　　　　　　t=td　に両手がターゲットの核を挟む目標位置に移動するよう手先制御を行う。 　
%Phase 4　　ケージング、キープ時間　　両手先が目標接触位置まで移動後、ターゲットのケージングを開始する。キープ

%上の関数
%Phase 1    待機時間
%Phase 2    制御　　　　　　　　　　　x方向　接近
%Phase 3    制御　　　　　　　　　　　手先角度　調整
%Phase 4　　ケージング、キープ時間　　キープ
%Phase 5    


    Kp = 20000;%10000;   % 20000;
    Kd = 500;%1000;   % 500;
%     Kp_2 = 1;3;   % 0.8
%     Kd_2 = 0.3;   % 0.3
Kd_ichi = 500/2 ;  
Kp_ichi = 30000 ;       
Kd_kaku = 500/12;    
Kp_kaku = 20000/30;       

% Kd_ichi = 500/2 ;  
% Kp_ichi = 3000 ;   
%元の質量
Kd_ichi = 500*2 ;  
Kp_ichi = 5000 ;   
Kd_kaku = 50;
Kp_kaku = 500;

% %ロボット100 
% Kd_ichi = 500/2 ;  
% Kp_ichi = 15000 ;   
% Kd_kaku = 50;
% Kp_kaku = 500;
%%%%%%%%%%%%%%%% 手先位置制御のためのφddの設定 %%%%%%%%%%%%%%%%%
%手先角加速度　φdd
%移動開始、移動中(～Phase3)に用いるqdd　 手先の位置と角、両方の成分を計算に組み込んでいる。
% qdd = Je * ...
%       ( ae_des ...
%       + Kd_ichi * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ... 速度（真ん中の項）
%       + Kp_ichi * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
%       + Kd_kaku/50 * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
%       + Kp_kaku/10 * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...角度
%       - Jd * qd_joints );

% qdd = Je * ...
%       ( ae_des ...
%       + 100 * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6(1:2)', zeros(1,4), VEL_jR6(1:2)', zeros(1,4) ]' ) ...
%       + 1000 * ( [ xe_des(1:2,1)', zeros(1,4), xe_des(7:8,1)', zeros(1,4) ]' - [ POS_jL6(1:2)', zeros(1,4), POS_jR6(1:2)', zeros(1,4) ]' ) ...
%       + 100 * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ 0 0 VEL_jL6(3)', zeros(1,3), 0 0 VEL_jR6(3)', zeros(1,3) ]' ) ...
%       + 1000 * ( [ 0 0 xe_des(3,1)', zeros(1,3), 0 0 xe_des(9,1)', zeros(1,3) ]' - [ 0 0 POS_jL6(3)', zeros(1,3), 0 0 POS_jR6(3)', zeros(1,3) ]' ) ...
%     - Jd * qd_joints );
% qdd = Je * ...
%       ( ae_des ...
%       + Kd_ichi * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ... 速度（真ん中の項）
%       + Kp_ichi * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
%       - Jd * qd_joints );

 qdd = Je * ...
      ( ae_des ...
      + Kd_ichi * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ... 速度（真ん中の項）
      + Kp_ichi * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
      + Kd_kaku * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
      + Kp_kaku * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...角度
      - Jd * qd_joints );
  
 qdd_2 = Je * ...
      ( ae_des ...
      + Kd_ichi/10 * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ... 速度（真ん中の項）
      + Kp_ichi/10 * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
      + Kd_kaku/10 * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
      + Kp_kaku/10 * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...角度
      - Jd * qd_joints );

%   qdd_ = Je * ...
%       ( ae_des ...
%       + Kd_ichi * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ... 速度（真ん中の項）
%       + Kp_ichi * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
%       + Kd_kaku/10 * ( [ zeros(1,3), ve_des(4:6,1)', zeros(1,3), ve_des(10:12,1)' ]' - [ zeros(1,3), qd_joints(4,1), qd_joints(5,1), qd_joints(6,1), zeros(1,3), qd_joints(10,1), qd_joints(11,1), qd_joints(12,1) ]' ) ...角速度
%       + Kp_kaku/10 * ( [ zeros(1,3), xe_des(4:6,1)', zeros(1,3), xe_des(10:12,1)' ]' - [ zeros(1,3),  q_joints(4,1),  q_joints(5,1),  q_joints(6,1), zeros(1,3),  q_joints(10,1),  q_joints(11,1),  q_joints(12,1) ]' ) ...角度
%       - Jd * qd_joints );

% qdd_2 = Je * ...
%       ( ae_des ...
%       + Kd_kaku/10 * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...
%       + Kp_kaku/10 * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...
%       - Jd * qd_joints );
%

%   qdd_3 = Je * ...
%       ( ae_des ...
%       + Kd_ichi * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ...             速度（真ん中の項）
%       + Kp_ichi * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
%       + Kd_kaku * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
%       + Kp_kaku * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...角度
%       - Jd * qd_joints );
  

%         + Kd_kaku * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,4),  -qd_joints(6),  qd_joints(5), zeros(1,4),qd_joints(12), qd_joints(11)  ]' ) ...
%       + Kp_kaku * ( [ zeros(1,3),  xe_des(4:6,1)', zeros(1,3),xe_des(10:12,1)' ]' ) ...
%       - Jd * qd_joints );
  
  
%   qdd_4 = Je * ...
%       ( ae_des ...
%       + Kd_ichi/100 * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ...             速度（真ん中の項）
%       + Kp_ichi/100 * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
%       + Kd_kaku*10  * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
%       + Kp_kaku*100 * ( [ zeros(1,3), xe_des(4:6,1)', zeros(1,3), xe_des(10:12,1)' ]' - [ zeros(1,3),q_joints(3,1),q_joints(4,1), q_joints(5,1), zeros(1,3), q_joints(9,1),q_joints(10,1), q_joints(11,1) ]' ) ...角度
%       - Jd * qd_joints );
%   qdd_4 = Je * ...
%       ( ae_des ...
%       + Kd_ichi/10 * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ...             速度（真ん中の項）
%       + Kp_ichi/10 * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
%       + Kd_kaku * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
%       + Kp_kaku*10* ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...角度
%       - Jd * qd_joints );

  qdd_4 = Je * ...
      ( ae_des ...
      + Kd_ichi/50 * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ...             速度（真ん中の項）
      + Kp_ichi/50 * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
      + Kd_kaku/10 * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
      + Kp_kaku/10 * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5), q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...角度
      - Jd * qd_joints );

%移動終了後(Phase5)に用いるqdd         手先の角の成分のみを組み込んで計算をしており、角の制御のみを行っている
%   qdd_5 =  Je * ...
%       ( ae_des ...
%       + 10*Kd_kaku * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...
%       + 10*Kp_kaku * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...
%       - Jd * qd_joints );
  
%    qdd_5 = Je * ...
%       ( ae_des ...
%       + Kd_ichi  * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ... 速度（真ん中の項）
%       + Kp_ichi  * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_jL6', zeros(1,3), POS_jR6', zeros(1,3) ]' ) ... 位置（右端の項）
%       + Kd_kaku  * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
%       + Kp_kaku  * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...角度
%       - Jd * qd_joints );
     qdd_5 = Je * ...
      ( ae_des ...
      + Kd_kaku  * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
      + Kp_kaku  * ( [ zeros(1,5), xe_des(6,1)', zeros(1,5), xe_des(12,1)' ]' - [ zeros(1,5),  q_joints(5,1), zeros(1,5),  q_joints(11,1) ]' ) ...角度
      - Jd * qd_joints );
       qdd_5 = Je * ...
      ( ae_des ...
      + Kd_kaku/10  * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,5), qd_joints(5,1), zeros(1,5), qd_joints(11,1) ]' ) ...角速度
      + Kp_kaku/10  * ( [ zeros(1,5), xe_des(6,1)', zeros(1,4), xe_des(11,1)', xe_des(12,1)' ]' - [  zeros(1,5),  q_joints(5,1), zeros(1,4), qd_joints(12), qd_joints(11) ]' ) ...角度
      - Jd * qd_joints );

%   %異物
%     qdd_3 = Je * ...  
%       ( ae_des ...
%       + Kd_ichi/100000 * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6(1:3)', zeros(1,3), VEL_jR6(1:3)', zeros(1,3) ]' ) ...
%       + Kp_ichi/100000 * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_eL(1:3)', zeros(1,3), POS_eR(1:3)', zeros(1,3) ]' ) ...
%       + Kd_kaku/1000 * ( [ zeros(1,5), ve_des(6,1)', zeros(1,5), ve_des(12,1)' ]' - [ zeros(1,4),  -qd_joints(6),  qd_joints(5), zeros(1,4),qd_joints(12), qd_joints(11)  ]' ) ...
%       + Kp_kaku/1000 * ( [ zeros(1,3),0 xe_des(5:6,1)', zeros(1,3), 0 xe_des(11:12,1)' ]' ) ...
%        );
%   qdd_3 = Je * ...
%       ( ae_des ...
%       + Kd_ichi/100 * ( [ ve_des(1:3,1)', zeros(1,3), ve_des(7:9,1)', zeros(1,3) ]' - [ VEL_jL6', zeros(1,3), VEL_jR6', zeros(1,3) ]' ) ...
%       + Kp_ichi/100 * ( [ xe_des(1:3,1)', zeros(1,3), xe_des(7:9,1)', zeros(1,3) ]' - [ POS_eL(1:3)', zeros(1,3), POS_eR(1:3)', zeros(1,3) ]' ) ...
%      - Jd * qd_joints);
  
% 関数calc_asuta呼び出し　　[H_asuta, C_asuta] = calc_asuta( LP, SV )
    [ H_asuta, C_asuta ] = calc_asuta( LP_d, SV_d );

% 目標トルク　　　各関節にかかる軸トルク tau
if Phase == 0       %初期姿勢をキープ
    
    SV_d.tau = H_asuta * qdd + C_asuta  - JL' * Fh_L - JR' * Fh_R ;
         
elseif Phase == 1   %
    
    SV_d.tau = H_asuta * qdd + C_asuta  - JL' * Fh_L - JR' * Fh_R ;
    
elseif Phase == 2
    
    SV_d.tau = H_asuta * qdd + C_asuta  - JL' * Fh_L - JR' * Fh_R ;
    
elseif Phase == 3   %スプライン曲線に沿った手先の制御を行うフェーズ
%     Kpp = 1;
%     Kdd = 0.1;
% %     des_q =[0 0 0 0 xe_des(6) 0    0 0 0 0 xe_des(12) 0 ]';
% %     des_q =[0 0 0 0 0 -xe_des(5)    0 0 0 0 0 xe_des(11)]';
%     des_q =[0 0 0 0 xe_des(6) -xe_des(5)    0 0 0 0 xe_des(12) xe_des(11)]';
%     SV_d.tau = H_asuta * qdd + C_asuta - JL' * Fh_L - JR' * Fh_R + Kpp * des_q  - Kdd * SV_d.qd ;
  
    SV_d.tau = H_asuta * qdd + C_asuta  - JL' * Fh_L - JR' * Fh_R ;
    
elseif Phase == 4 
  
    SV_d.tau = H_asuta * qdd_4 + C_asuta  - JL' * Fh_L - JR' * Fh_R ;
    
...             + 1 * ( zeros(8,1) - [ qd_joints(1:2)', zeros(1,2), qd_joints(5:6)', zeros(1,2) ]' ) ...
...             + 2 * ( [ q_joints_kotei(1:2)', zeros(1,2), q_joints_kotei(5:6)', zeros(1,2) ]' - [ q_joints(1:2)', zeros(1,2), q_joints(5:6)', zeros(1,2) ]' ) ...
...             + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
...             + 2 * ( [ q_joints_kotei(1:3)', 0, q_joints_kotei(5:7)', 0 ]' - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
...             + Kd_wrist * ( zeros(12,1) - qd_wrist ) + Kp_wrist * ( zeros(12,1) - q_wrist );
    
    
%     q_joints_kotei = [0.247128441730728;-2.19831874898074;1.21540891526275;-1.66718075523459;-0.708118802990349;-1.16960767718874;0.230984105174590;-1.95459129012145;-0.825769693690444;1.63409084121031;1.01791887061409;0.882504316228143];
%   
%     SV_d.tau =   - JL' * Fh_L - JR' * Fh_R ...
%              + 1 * ( zeros(12,1) - [ qd_joints(1:3)', zeros(1,3), qd_joints(7:9)', zeros(1,3) ]' ) ...
%              + 2 * ( [ q_joints_kotei(1:3)', zeros(1,3), q_joints_kotei(7:9)', zeros(1,3) ]' - [ q_joints(1:3)', zeros(1,3), q_joints(7:9)', zeros(1,3) ]' );


%         %トルク　＝　Kpp * 目標角との差　ー　Kdd * 角速度
%     SV_d.tau = Kpp * (des_q - SV_d.q)  - Kdd * SV_d.qd;
%     
%     if norm(ts6-POS_eL) > phase5kyori * te1 || norm(ts3-POS_eR) > phase5kyori * te1
%         %関節空間における運動方程式　τ= H * qdd   +  C  -  J * Fh
%         SV_d.tau = H_asuta * qdd + C_asuta - JL' * Fh_L - JR' * Fh_R    +    Kpp * (des_q - SV_d.q)  - Kdd * SV_d.qd;
%     end

elseif Phase == 5


        SV_d.tau = H_asuta * qdd_4 + C_asuta  - JL' * Fh_L - JR' * Fh_R ;
        
%     q_joints_kotei = [0.247128441730728;-2.19831874898074;1.21540891526275;-1.66718075523459;-0.708118802990349;-1.16960767718874;0.230984105174590;-1.95459129012145;-0.825769693690444;1.63409084121031;1.01791887061409;0.882504316228143]*0.8;
% 
%     SV_d.tau =  - JL' * Fh_L - JR' * Fh_R ...
%              + 0.1 * ( zeros(12,1) - [ qd_joints(1:12)']' ) ...
%              + 0.2 * ( [ q_joints_kotei(1:12)' ]' - [ q_joints(1:12)' ]' );

%     Kpp =20;
% Kdd =2;
% 
% % q_tmp = [0.224153864843193;-2.16665497197746;1.18266643534682;-1.64184863429214;-0.722588824959391;-1.15289624513847;0.169355683521157;-1.65621614379469;-0.844035531867656;1.68727601851660;1.05083931780229;0.833664039559653];%2.85(2.86)
% % q_tmp = [0.124542744831853;-2.46022461196123;1.18145582744551;-1.61696602859185;-0.705580509920546;-1.23855892419515;0.263419065170588;-1.94902305428311;-0.851253298578236;1.63184678326897;1.07475071564723;0.712178816996584];%3(3.01)
% q_tmp = [0.247128441730728;-2.19831874898074;1.21540891526275;-1.66718075523459;-0.708118802990349;-1.16960767718874;0.230984105174590;-1.95459129012145;-0.825769693690444;1.63409084121031;1.01791887061409;0.882504316228143];
% SV_d.tau =   Kpp * (q_tmp - SV_d.q)  - Kdd * SV_d.qd;

    % H_asuta * qdd_5 + C_asuta 
    
%   if catchtime >= hokakutime
%      if catchtime > ( hokakutime - d_time )  &&  catchtime < ( hokakutime + d_time )
%         flagphase5 = 1;
%         q_joints_kotei = q_joints;
%      end

% KL = kakudo( [ POS_j3' 0 ]', [ ( 2 * POS_j3 - POS_j2 )' 0 ]', [ POS_j7' 0 ]' );
% KR = kakudo( [ POS_j7' 0 ]', [ ( 2 * POS_j7 - POS_j6 )' 0 ]', [ POS_j3' 0 ]' );
%     SV_d.tau = - JL' * Fh_L - JR' * Fh_R ... H_asuta * qdd_2 + C_asuta - JL' * Fh_L - JR' * Fh_R ... 
% ...             + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
% ...             + 3 * ( [ q_joints_kotei(1:3)', 0, q_joints_kotei(5:7)', 0 ]' - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
%              + 1 * ( zeros(8,1) - [ qd_joints(1:3)', 0, qd_joints(5:7)', 0 ]' ) ...
%              + 3 * ( [ q_joints_kotei(1:2)', KL, 0, q_joints_kotei(5:6)', KR, 0 ]' ...
%                    - [ q_joints(1:3)', 0, q_joints(5:7)', 0 ]' ) ...
%              + Kd_wrist * ( zeros(8,1) - qd_wrist ) + Kp_wrist * ( zeros(8,1) - q_wrist );

         
% elseif Phase == 2 || Phase == 3 
% 
% %     SV_d.tau = H_asuta * qdd + C_asuta - JL' * Fh_L - JR' * Fh_R; 
%     SV_d.tau = H_asuta * qdd + C_asuta - JL' * Fh_L - JR' * Fh_R ...
%              + Kd_wrist * ( zeros(12,1) - qd_wrist ) + Kp_wrist * ( zeros(12,1) - q_wrist );
% 
% else %Phase1
%    %main32には下記が記載されていない　（球に必要?　or main32以降の最終版で追加された?）
% %     SV_d.tau = H_asuta * qdd+ C_asuta ;

else 
%          SV_d.tau = H_asuta * qdd + C_asuta - JL' * Fh_L - JR' * Fh_R ;
%          SV_d.tau = zeros(12,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 順動力学の計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 運動方程式より次の時間のベースの加速度及び関節角加速度を計算し，それを積分することによりでベースの位置，速度，関節角，関節角速度を求める．
% 運動方程式中の速度非線形項を計算するために，再帰ニュートンオイラー法による逆動力学も内部で解いている．
    SV_d =  f_dyn_rk2( LP_d, SV_d );
    SV_ts = f_dyn_rk2( LP_ts, SV_ts );
% 各リンクの座標変換行列(方向余弦行列)の計算（リンクi → 慣性座標系）
    SV_d = calc_aa( LP_d, SV_d );
% 各リンク重心の位置
    SV_d = calc_pos( LP_d, SV_d );    
    % 速度、加速度の計算
    SV_d = calc_vel(LP_d,SV_d);
    SV_d = calc_acc(LP_d,SV_d);

% 全関節位置姿勢の計算
    [ POS_j_L, ORI_j_L ] = f_kin_j( LP_d, SV_d, jointsL );   % 左側
    [ POS_j_R, ORI_j_R ] = f_kin_j( LP_d, SV_d, jointsR );   % 右側
    
% 各関節の位置 x,y
POS_jL1 = POS_j_L( 1:3, 1 );   %左側　関節1（根本）の位置代入　　
POS_jL2 = POS_j_L( 1:3, 2 );   %  なぜか関節1とほぼ同一位置にある関節　存在価値なし
POS_jL3 = POS_j_L( 1:3, 3 );   %  なぜか関節1とほぼ同一位置にある関節　存在価値なし
POS_jL4 = POS_j_L( 1:3, 4 );   %左側　関節4 の位置代入　　
POS_jL5 = POS_j_L( 1:3, 5 );   %左側　関節5 の位置代入　　
POS_jL6 = POS_j_L( 1:3, 6 );   %左側手先付け根の関節位置　この情報のみDesire関数で利用

POS_jR1 = POS_j_R( 1:3, 1 );   % 右側　関節1の位置代入
POS_jR2 = POS_j_R( 1:3, 2 );   %  なぜか関節1とほぼ同一位置にある関節　存在価値なし
POS_jR3 = POS_j_R( 1:3, 3 );   %  なぜか関節1とほぼ同一位置にある関節　存在価値なし
POS_jR4 = POS_j_R( 1:3, 4 );   %右側　関節4 の位置代入　　
POS_jR5 = POS_j_R( 1:3, 5 );   %右側　関節5 の位置代入　　
POS_jR6 = POS_j_R( 1:3, 6 );   %左側手先付け根の関節位置　この情報のみDesire関数で利用

% アーム手先位置・姿勢
[ POS_eL, ORI_eL ] = f_kin_e( LP_d, SV_d, jointsL );   %左側
[ POS_eR, ORI_eR ] = f_kin_e( LP_d, SV_d, jointsR );   %右側

POS_ee1 = POS_eL + ORI_eL * [te1 -te1 0]';
POS_ee2 = POS_eL + ORI_eL * [te1 te1 * cos(60*pi()/180) te1 * sin(60*pi()/180)]';
POS_ee3 = POS_eL + ORI_eL * [te1 te1 * cos(60*pi()/180) -te1 * sin(60*pi()/180)]';
POS_ee4 = POS_eR + ORI_eR * [te1 te1 0]';
POS_ee5 = POS_eR + ORI_eR * [te1 -te1 * cos(60*pi()/180) te1 * sin(60*pi()/180)]';
POS_ee6 = POS_eR + ORI_eR * [te1 -te1 * cos(60*pi()/180) -te1 * sin(60*pi()/180)]';

POS_end=[POS_ee1 POS_ee2 POS_ee3 POS_ee4 POS_ee5 POS_ee6]; 

% tarPOS_ee1 = inv(SV_ts.A0) * (POS_ee1 -SV_ts.R0);
% tarPOS_ee2 = inv(SV_ts.A0) * (POS_ee2 -SV_ts.R0);
% tarPOS_ee3 = inv(SV_ts.A0) * (POS_ee3 -SV_ts.R0);
% tarPOS_ee4 = inv(SV_ts.A0) * (POS_ee4 -SV_ts.R0);
% tarPOS_ee5 = inv(SV_ts.A0) * (POS_ee5 -SV_ts.R0);
% tarPOS_ee6 = inv(SV_ts.A0) * (POS_ee6 -SV_ts.R0);

tarPOS_ee1 = SV_ts.A0 \ (POS_ee1 -SV_ts.R0);
tarPOS_ee2 = SV_ts.A0 \ (POS_ee2 -SV_ts.R0);
tarPOS_ee3 = SV_ts.A0 \ (POS_ee3 -SV_ts.R0);
tarPOS_ee4 = SV_ts.A0 \ (POS_ee4 -SV_ts.R0);
tarPOS_ee5 = SV_ts.A0 \ (POS_ee5 -SV_ts.R0);
tarPOS_ee6 = SV_ts.A0 \ (POS_ee6 -SV_ts.R0);


if abs(tarPOS_ee2(1) - tarPOS_ee6(1)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee2(2) - tarPOS_ee6(2)) < halftarget * 2 + r_tip *2
    caging26 = 1;
else
    caging26 = 0;
end
if abs(tarPOS_ee3(1) - tarPOS_ee5(1)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee3(2) - tarPOS_ee5(2)) < halftarget * 2 + r_tip *2
    caging35 = 1;
else
    caging35 = 0;
end
if caging26 ==1 && caging35 ==1
    caging2dim = 1;
else
    caging2dim = 0;
end

if abs(tarPOS_ee1(1) - tarPOS_ee5(1)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee1(2) - tarPOS_ee5(2)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee1(3) - tarPOS_ee5(3)) < halftargetz * 2 + r_tip *2
    caging15 = 1;
else
    caging15 = 0;
end
if abs(tarPOS_ee1(1) - tarPOS_ee6(1)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee1(2) - tarPOS_ee6(2)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee1(3) - tarPOS_ee6(3)) < halftargetz * 2 + r_tip *2
    caging16 = 1;
else
    caging16 = 0;
end
if abs(tarPOS_ee2(1) - tarPOS_ee3(1)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee2(2) - tarPOS_ee3(2)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee2(3) - tarPOS_ee3(3)) < halftargetz * 2 + r_tip *2
    caging23 = 1;
else
    caging23 = 0;
end
if abs(tarPOS_ee2(1) - tarPOS_ee4(1)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee2(2) - tarPOS_ee4(2)) < halftarget * 2 + r_tip *2 && abs(tarPOS_ee2(3) - tarPOS_ee4(3)) < halftargetz * 2 + r_tip *2
    caging24 = 1;
else
    caging24 = 0;
end

if caging2dim == 1 && caging15 ==1 && caging16 ==1 && caging23 ==1 && caging24 ==1
    caging3dim = 1;
else
    caging3dim = 0;
end
tarPOS_end=[tarPOS_ee1 tarPOS_ee2 tarPOS_ee3 tarPOS_ee4 tarPOS_ee5 tarPOS_ee6]; 

Mins = min(tarPOS_end,[],2);
Maxs = max(tarPOS_end,[],2);

limitedMins(1:2) = Mins(1:2) + (halftarget + r_tip);
limitedMins(3) = Mins(3) + (halftargetz + r_tip);
limitedMaxs(1:2) = Maxs(1:2) - (halftarget + r_tip);
limitedMaxs(3) = Maxs(3) - (halftargetz + r_tip);


if caging3dim ==1
    vertex1 = SV_ts.R0 + SV_ts.A0 * [limitedMins(1) limitedMins(2) limitedMins(3)]';
    vertex2 = SV_ts.R0 + SV_ts.A0 * [limitedMaxs(1) limitedMins(2) limitedMins(3)]';
    vertex3 = SV_ts.R0 + SV_ts.A0 * [limitedMins(1) limitedMaxs(2) limitedMins(3)]';
    vertex4 = SV_ts.R0 + SV_ts.A0 * [limitedMins(1) limitedMins(2) limitedMaxs(3)]';
    vertex5 = SV_ts.R0 + SV_ts.A0 * [limitedMaxs(1) limitedMaxs(2) limitedMins(3)]';
    vertex6 = SV_ts.R0 + SV_ts.A0 * [limitedMins(1) limitedMaxs(2) limitedMaxs(3)]';
    vertex7 = SV_ts.R0 + SV_ts.A0 * [limitedMaxs(1) limitedMins(2) limitedMaxs(3)]';
    vertex8 = SV_ts.R0 + SV_ts.A0 * [limitedMaxs(1) limitedMaxs(2) limitedMaxs(3)]';
else
    vertex1 = SV_ts.R0;
    vertex2 = SV_ts.R0;
    vertex3 = SV_ts.R0;
    vertex4 = SV_ts.R0;
    vertex5 = SV_ts.R0;
    vertex6 = SV_ts.R0;
    vertex7 = SV_ts.R0;
    vertex8 = SV_ts.R0;
end
for i = 1:3
    if limitedMins(i) > 0
        limitedMins(i) = 0;
    end
    if limitedMaxs(i) < 0
        limitedMins(i) = 0;
    end
end
if caging3dim ==1
    VoM = (abs(limitedMins(1)) + abs(limitedMaxs(1))) * (abs(limitedMins(2)) + abs(limitedMaxs(2))) * (abs(limitedMins(3)) + abs(limitedMaxs(3)));
else
    VoM = NaN;
end


%%%%ターゲット頂点の計算
    ts1r = SV_ts.R0 + SV_ts.A0 * [halftarget + r_tip halftarget + r_tip halftargetz + r_tip]';
    ts2r = SV_ts.R0 + SV_ts.A0 * [halftarget + r_tip halftarget + r_tip -1 * halftargetz - r_tip]';
    ts3r = SV_ts.R0 + SV_ts.A0 * [halftarget + r_tip -1 * halftarget - r_tip halftargetz + r_tip]';
    ts4r = SV_ts.R0 + SV_ts.A0 * [halftarget + r_tip -1 * halftarget - r_tip -1 * halftargetz - r_tip]';
    ts5r = SV_ts.R0 + SV_ts.A0 * [-1 * halftarget - r_tip halftarget + r_tip halftargetz + r_tip]';
    ts6r = SV_ts.R0 + SV_ts.A0 * [-1 * halftarget - r_tip halftarget + r_tip -1 * halftargetz - r_tip]';
    ts7r = SV_ts.R0 + SV_ts.A0 * [-1 * halftarget - r_tip -1 * halftarget - r_tip halftargetz + r_tip]';
    ts8r = SV_ts.R0 + SV_ts.A0 * [-1 * halftarget - r_tip -1 * halftarget - r_tip -1 * halftargetz - r_tip]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 全運動量の計算 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ロボット
  HH_d =  calc_hh( LP_d, SV_d );
  Hb_d =  HH_d( 1:6, 1:6 );
  Hbm_d = HH_d( 1:6, 7:end );
  PP_base =  Hb_d * [ SV_d.v0' SV_d.w0' ]';
  
  PP_d =  Hb_d * [ SV_d.v0' SV_d.w0' ]' + Hbm_d * SV_d.qd;

% ターゲット
  HH_tm = calc_hh( LP_ts, SV_ts );
  Hb_tm = HH_tm( 1:6, 1:6 );
  
  PP_tm = Hb_tm * [ SV_ts.v0' SV_ts.w0' ]';

% 系全体
% Rg_d = calc_Rg( LP_d, SV_d );  %重心位置
%   R = SV_ts.R0 - Rg_d;
  
  PP(1:3,1) = PP_d(1:3,1) + PP_tm(1:3,1);
%   PP(4:6,1) = PP_d(4:6,1) + PP_tm(4:6,1) + LP_ts.mass * cross((SV_ts.R0 - SV_d.R0),SV_ts.v0);
  PP(4:6,1) = PP_d(4:6,1) + PP_tm(4:6,1) + cross((SV_ts.R0),PP_tm(1:3)) + cross((SV_d.R0), PP_d(1:3));
%   PP(4:6,1) = PP_d(4:6,1) + PP_tm(4:6,1) + cross((SV_ts.R0),PP_tm(1:3)) + cross((Rg_d), PP_d(1:3));

%   R = SV_ts.R0 - SV_d.R0;
%   PP( 1:3, 1 ) = PP_d( 1:3, 1 ) + PP_tm( 1:3, 1 );
%   PP( 4:5, 1 ) = zeros( 2, 1 );
%   PP( 6, 1 ) = PP_d( 6, 1 ) + PP_tm( 6, 1 ) + LP_ts.mass * ( R( 1, 1 ) * SV_ts.v0( 2, 1 ) - R( 2, 1 ) * SV_ts.v0( 1, 1 ) );
  
% 各リンク質量
 link_mass = LP_d.m( 1, 1:12 );
% 各リンク慣性モーメント
 link_inertia = [ LP_d.inertia( :, 1:3 ) LP_d.inertia( :, 4:6 ) LP_d.inertia( :, 7:9 ) LP_d.inertia( :, 10:12 ) LP_d.inertia( :, 13:15 ) LP_d.inertia( :, 16:18 ) LP_d.inertia( :, 19:21 ) LP_d.inertia( :, 22:24 ) LP_d.inertia( :, 25:27 ) LP_d.inertia( :, 28:30 ) LP_d.inertia( :, 31:33 ) LP_d.inertia( :, 34:36 )];
% 各リンク重心位置
 link_position = SV_d.RR( :, 1:12 );
% 各リンク重心位置の歪対称行列
 link_tilde = [ tilde( link_position( :, 1 ) ) tilde( link_position( :, 2 ) ) tilde( link_position( :, 3 ) ) tilde( link_position( :, 4 ) ) tilde( link_position( :, 5 ) ) tilde( link_position( :, 6 ) ) tilde( link_position( :, 7 ) ) tilde( link_position( :, 8 ) ) tilde( link_position( :, 9 ) ) tilde( link_position( :, 10 ) ) tilde( link_position( :, 11 ) ) tilde( link_position( :, 12 ) ) ];
% 各リンク重心速度
 link_velocity = SV_d.vv( :, 1:12 );
% 各リンク重心角速度
 link_angularvelocity = SV_d.ww( :, 1:12 );
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
  P_d = P_d + link_mass(1,9) * link_velocity(:,9);
  P_d = P_d + link_mass(1,10) * link_velocity(:,10);
  P_d = P_d + link_mass(1,11) * link_velocity(:,11);
  P_d = P_d + link_mass(1,12) * link_velocity(:,12);
  L_d = LP_d.inertia0 * SV_d.w0 + tilde( SV_d.R0 ) * LP_d.m0 * SV_d.v0;
  L_d = L_d + link_inertia(:,1:3)   * link_angularvelocity(:,1) + link_tilde(:,1:3)   * link_mass(1,1) * link_velocity(:,1);
  L_d = L_d + link_inertia(:,4:6)   * link_angularvelocity(:,2) + link_tilde(:,4:6)   * link_mass(1,2) * link_velocity(:,2);
  L_d = L_d + link_inertia(:,7:9)   * link_angularvelocity(:,3) + link_tilde(:,7:9)   * link_mass(1,3) * link_velocity(:,3);
  L_d = L_d + link_inertia(:,10:12) * link_angularvelocity(:,4) + link_tilde(:,10:12) * link_mass(1,4) * link_velocity(:,4);
  L_d = L_d + link_inertia(:,13:15) * link_angularvelocity(:,5) + link_tilde(:,13:15) * link_mass(1,5) * link_velocity(:,5);
  L_d = L_d + link_inertia(:,16:18) * link_angularvelocity(:,6) + link_tilde(:,16:18) * link_mass(1,6) * link_velocity(:,6);
  L_d = L_d + link_inertia(:,19:21) * link_angularvelocity(:,7) + link_tilde(:,19:21) * link_mass(1,7) * link_velocity(:,7);
  L_d = L_d + link_inertia(:,22:24) * link_angularvelocity(:,8) + link_tilde(:,22:24) * link_mass(1,8) * link_velocity(:,8);
  L_d = L_d + link_inertia(:,25:27) * link_angularvelocity(:,9) + link_tilde(:,25:27) * link_mass(1,9) * link_velocity(:,9);
  L_d = L_d + link_inertia(:,28:30) * link_angularvelocity(:,10) + link_tilde(:,28:30) * link_mass(1,10) * link_velocity(:,10);
  L_d = L_d + link_inertia(:,31:33) * link_angularvelocity(:,11) + link_tilde(:,31:33) * link_mass(1,11) * link_velocity(:,11);
  L_d = L_d + link_inertia(:,34:36) * link_angularvelocity(:,12) + link_tilde(:,34:36) * link_mass(1,12) * link_velocity(:,12);
  PP_d_2 = [ P_d' L_d' ]';
% ターゲット重心位置の歪対称行列
  target_tilde = tilde( SV_ts.R0 );
% ターゲットの並進運動量P_t, 角運動量L_t
  P_ts = LP_ts.m0 * SV_ts.v0;
  L_ts = LP_ts.inertia0 * SV_ts.w0 + target_tilde * P_ts;  
  PP_ts_2 = [ P_ts' L_ts' ]';
% 系全体の運動量6*1（運動量３つと角運動量３つ） 　　　ロボットの総運動量6*1　＋　ターゲットの総運動量6*1
  PP_2 = PP_d_2 + PP_ts_2;


%%%%%ロボット頂点の計算
    
    rbt1 = SV_d.R0 + SV_d.A0 * [LP_d.c0(1,1) LP_d.c0(2,1) rbt_halftate]';
    rbt2 = SV_d.R0 + SV_d.A0 * [LP_d.c0(1,1) LP_d.c0(2,1) -1 * rbt_halftate]';
    rbt3 = SV_d.R0 + SV_d.A0 * [LP_d.c0(1,1) -1 * LP_d.c0(2,1) rbt_halftate]';
    rbt4 = SV_d.R0 + SV_d.A0 * [LP_d.c0(1,1) -1 * LP_d.c0(2,1) -1 * rbt_halftate]';
    rbt5 = SV_d.R0 + SV_d.A0 * [-1 * LP_d.c0(1,1) LP_d.c0(2,1) rbt_halftate]';
    rbt6 = SV_d.R0 + SV_d.A0 * [-1 * LP_d.c0(1,1) LP_d.c0(2,1) -1 * rbt_halftate]';
    rbt7 = SV_d.R0 + SV_d.A0 * [-1 * LP_d.c0(1,1) -1 * LP_d.c0(2,1) rbt_halftate]';
    rbt8 = SV_d.R0 + SV_d.A0 * [-1 * LP_d.c0(1,1) -1 * LP_d.c0(2,1) -1 * rbt_halftate]';

%     タガメ型
%     rbt1 = SV_d.R0 + SV_d.A0 * [LP_d.c0(1,1) LP_d.c0(2,1) rbt_halftate]';
%     rbt2 = SV_d.R0 + SV_d.A0 * [LP_d.c0(1,1) LP_d.c0(2,1) -1 * rbt_halftate]';
%     rbt5 = SV_d.R0 + SV_d.A0 * [-1 * LP_d.c0(1,1) LP_d.c0(2,1) rbt_halftate]';
%     rbt6 = SV_d.R0 + SV_d.A0 * [-1 * LP_d.c0(1,1) LP_d.c0(2,1) -1 * rbt_halftate]';
%     
%     rbt3 = SV_d.R0 + SV_d.A0 * [1.5 * LP_d.c0(1,1) -3.5 * LP_d.c0(2,1) 1.5 * rbt_halftate]';
%     rbt4 = SV_d.R0 + SV_d.A0 * [1.5 * LP_d.c0(1,1) -3.5 * LP_d.c0(2,1) -1.5 * rbt_halftate]';
%     rbt7 = SV_d.R0 + SV_d.A0 * [-1.5 * LP_d.c0(1,1) -3.5 * LP_d.c0(2,1) 1.5 * rbt_halftate]';
%     rbt8 = SV_d.R0 + SV_d.A0 * [-1.5 * LP_d.c0(1,1) -3.5 * LP_d.c0(2,1) -1.5 * rbt_halftate]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 結果をファイルに出力 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 機構長さ   % アニメ用長さ，ロボット・ターゲットの機構的長さ
% if time == 0
% fprintf( fo_01_Length_etc, [ repmat( g, 1, 16 ), ge ], ...   % =17コ
%          r_tip, side_target, half_side, r_motor, r_base, r_airtank, ...   % 6
%          base_tate, base_yoko, base_jushin_teihen', base_arm_yoko_1, base_arm_yoko_2, base_arm_tate_1, base_arm_tate_2, base_airtank_yoko, base_airtank_tate, L, theta );   % 11
% end

if rem(time,0.01) == 0% 各種長さ
    fprintf( fidw_length, '%g\n', halftarget );
    % r_tip手先球半径，r_targetターゲット半径，w_dロボット全体質量，mtm_estターゲット推定質量，base_yokoベース四角形横，base_tateベース四角形縦，base_jushin_teihenベースの重心から底辺までの長さ
    % base_arm_yoko_1ベース重心から左アーム付け根までの横の長さ，base_arm_yoko_2ベース重心から右アーム付け根までの横の長さ，base_arm_tate_1ベース重心から左アーム付け根までの縦の長さ，base_arm_tate_2ベース重心から右アーム付け根までの縦の長さ，r_motorモータ半径，r_baseベースの重心描画用円
    % エアタンク半径，ベース重心からエアタンク中心までの横距離，ベース重心からエアタンク中心までの縦距離

% ターゲット
    fprintf( fidw_movie, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\n',...
        time, ts1, ts2, ts3, ts4, ts5, ts6, ts7, ts8, SV_ts.w0 * 180 / pi);
% ターゲット
    fprintf( fidw_movie2, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\n',...
        time, ts1r, ts2r, ts3r, ts4r, ts5r, ts6r, ts7r, ts8r);

% ロボット
    fprintf( fidw_d, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t      %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\n ',...
        time, rbt1, rbt2, rbt3, rbt4, rbt5, rbt6, rbt7, rbt8,POS_j_L,POS_j_R);

% ロボット2
    fprintf( fidw_d2, '%g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\n ',...
        POS_ee1 + ORI_eL * [-te1 0 0]',POS_ee2 + ORI_eL * [-te1 0 0]',POS_ee3 + ORI_eL * [-te1 0 0]',POS_ee4 + ORI_eR * [-te1 0 0]',POS_ee5 + ORI_eR * [-te1 0 0]',POS_ee6 + ORI_eR * [-te1 0 0]',POS_ee1,POS_ee2,POS_ee3,POS_ee4,POS_ee5,POS_ee6);

% ロボット3
    fprintf( fidw_d3, '%g\t  %g\t%g\t%g\t%g\t%g\t%g\t %g\t%g\t%g\t%g\t%g\t%g\n ',...
        time,SV_d.q*180/pi);

% ロボット4
    fprintf( fidw_d4, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t %g\n ',...
        time,vertex1,vertex2,vertex3,vertex4,vertex5,vertex6,vertex7,vertex8,VoM);
    
% 運動量     PP→系全体の運動量 PP_2　長谷さんの計算式による、系全体の運動量と角運動量それぞれの総和
    fprintf( fidw_en, '%g\t  %g\t%g\t%g\t%g\t%g\t%g\t  %g\t%g\t%g\t%g\t%g\t%g\t  %g\t%g\t%g\t%g\t%g\t%g\t %g\t%g\t%g\t%g\t%g\t%g\t %g\t%g\t%g\t%g\t%g\t%g\n %g\t%g\t%g\t%g\t%g\t%g\n',...
        time, PP_d, PP_tm, PP,  PP_2,PP_d_2,PP_ts_2);

% コンタクトフラッグ
    fprintf( fidw_cf, '%g\t %g\t%g\t%g\t%g\t%g\t%g\t %g\t %g\n',time, contactflag_LR, Phase, caging3dim);

% δ，力（このままでは力としててんてん）
    fprintf( fidw_ft, '%g\t %g\t%g\t%g\t%g\t%g\t%g\t %g\t%g\t%g\t%g\t%g\t%g\t %g\t%g\t%g\t%g\t%g\t%g\n',...
        time,delta_tmp,force_i,distance_i);
    
% δ，力（このままでは力としててんてん）
    fprintf( fidw_thesis, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\n',...
        time,SV_d.R0,dc2rpy(SV_d.A0),SV_d.v0,SV_d.w0,SV_ts.R0,dc2rpy(SV_ts.A0),SV_ts.v0,SV_ts.F0,SV_ts.T0);
end
% time時間1，SV_tm.R0ターゲットの位置3，SV_tm.v0ターゲットの並進速度3，SV_tm.Q0ターゲットの角度[deg]3，SV_tm.w0ターゲットの角速度3，rel_pos左手先とターゲットの相対位置3，curPosAP3_delta_1接触位置の1ステップ分の差3，curPosAP3_1現在の接触位置3，PP_tmターゲットの全運動量6
%{
    if catchcount > catchlimit
      P = fopen(strcat(path,'\〇捕獲成功.txt'),'w');
      break
    end
    
    if casecount > caselimit
      P = fopen(strcat(path,'\〇ゲージング成功.txt'),'w');
      break
    end
    
    LLL = sqrt( ( SV_d.R0(1,1) - SV_ts.R0(1,1) )^2 + ( SV_d.R0(2,1) - SV_ts.R0(2,1) )^2 );

    if abs( PP(1,1) ) > 0.005 || abs( PP(2,1) ) > 0.005
      P = fopen(strcat(path,'\×特異点.txt'),'w');
        break
    end
  
    if  LLL > 0.5
      P = fopen(strcat(path,'\×範囲外.txt'),'w');
        break
    end
  
    if LLL < (r_target + base_arm_tate_1)
      P = fopen(strcat(path,'\×ベース衝突.txt'),'w');
        break
    end
%}
% 
%     if max(max(delta)) > 0.95 * r_tip
%        P = fopen(strcat(path,'\くいこみ.txt'),'w');
%        break
%     end
    if norm(SV_d.R0 - SV_ts.R0) > 1
       P = fopen(strcat(path,'\離れすぎ.txt'),'w');
       break
    end
%     if abs(PP_d(1) + PP_d(2) + PP_d(3) + PP_tm(1) + PP_tm(2) + PP_tm(3)  ) > 1
%        P = fopen(strcat(path,'\発散.txt'),'w');
%        break
%     end
%     if caging3dim ==1
%         break
%     end
%     if Phase == 4
%        P = fopen(strcat(path,'\Phase4.txt'),'w');
%         break
%     end
end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% シミュレーションループストップ %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 結果の表示 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% {

%%
%%%%%%%%%% 値読み込み %%%%%%%%%%
%{ 各種長さ
fidw_length = fopen( strcat( datpath, '\Length.dat' ), 'r' );
tmp_length = fscanf( fidw_length, '%g\n', [1 inf] );
tmp_length = tmp_length';   
fclose( fidw_length );   % r_tip, r_target, w_d, mtm_est, base_yoko, base_tate, base_jushin_teihen, base_arm_yoko_1, base_arm_yoko_2, base_arm_tate_1, base_arm_tate_2, r_motor, r_base, r_airtank, base_airtank_yoko, base_airtank_tate
% アニメ用
fidw_movie = fopen( strcat( datpath, '\DualArm_TargetMaru_TrackingContact_ForMovie.dat' ), 'r' ); 
tmp_movie = fscanf( fidw_movie, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\n', [28 inf] );
tmp_movie = tmp_movie';   % 時間1 ベース位置2 各関節位置2,2,2,2,2,2，左手先位置2，右手先位置2，ターゲット重心位置2，ロボットベース姿勢1，ターゲット姿勢1
fclose( fidw_movie );   % time, SV_d.R0(1:2,1), POS_j1, POS_j2, POS_j3, POS_j4, POS_j5, POS_j6, POS_eL(1:2,1), POS_eR(1:2,1), SV_tm.R0(1:2,1), SV_d.Q0(3,1), SV_tm.Q0(3,1), tm_dammy_R0
% % アニメ用
% fidw_movie2 = fopen( strcat( datpath, '\DualArm_TargetMaru_TrackingContact_ForMovie2.dat' ), 'r' ); 
% tmp_movie2 = fscanf( fidw_movie2, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\n', [25 inf] );
% tmp_movie2 = tmp_movie2';   % 時間1 ベース位置2 各関節位置2,2,2,2,2,2，左手先位置2，右手先位置2，ターゲット重心位置2，ロボットベース姿勢1，ターゲット姿勢1
% fclose( fidw_movie2 );   % time, SV_d.R0(1:2,1), POS_j1, POS_j2, POS_j3, POS_j4, POS_j5, POS_j6, POS_eL(1:2,1), POS_eR(1:2,1), SV_tm.R0(1:2,1), SV_d.Q0(3,1), SV_tm.Q0(3,1), tm_dammy_R0

%robot
fidw_d = fopen( strcat( datpath, '\DualArm.dat' ), 'r' ); 
tmp_d = fscanf( fidw_d, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t     %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\n ', [61 inf] );
tmp_d = tmp_d';   % 時間1 ベース位置2 各関節位置2,2,2,2,2,2，左手先位置2，右手先位置2，ターゲット重心位置2，ロボットベース姿勢1，ターゲット姿勢1
fclose( fidw_d );   % time, SV_d.R0(1:2,1), POS_j1, POS_j2, POS_j3, POS_j4, POS_j5, POS_j6, POS_eL(1:2,1), POS_eR(1:2,1), SV_tm.R0(1:2,1), SV_d.Q0(3,1), SV_tm.Q0(3,1), tm_dammy_R0

%robot2
fidw_d2 = fopen( strcat( datpath, '\DualArm2.dat' ), 'r' ); 
tmp_d2 = fscanf( fidw_d2, '%g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\t %g\t%g\t%g\n ', [36 inf] );
tmp_d2 = tmp_d2';   % 時間1 ベース位置2 各関節位置2,2,2,2,2,2，左手先位置2，右手先位置2，ターゲット重心位置2，ロボットベース姿勢1，ターゲット姿勢1
fclose( fidw_d2 );   % time, SV_d.R0(1:2,1), POS_j1, POS_j2, POS_j3, POS_j4, POS_j5, POS_j6, POS_eL(1:2,1), POS_eR(1:2,1), SV_tm.R0(1:2,1), SV_d.Q0(3,1), SV_tm.Q0(3,1), tm_dammy_R0

%robot3
fidw_d3 = fopen( strcat( datpath, '\DualArm3.dat' ), 'r' ); 
tmp_d3 = fscanf( fidw_d3, '%g\t  %g\t%g\t%g\t%g\t%g\t%g\t %g\t%g\t%g\t%g\t%g\t%g\n ', [13 inf] );
tmp_d3 = tmp_d3';   % 時間1 ベース位置2 各関節位置2,2,2,2,2,2，左手先位置2，右手先位置2，ターゲット重心位置2，ロボットベース姿勢1，ターゲット姿勢1
fclose( fidw_d3 );   % time, SV_d.R0(1:2,1), POS_j1, POS_j2, POS_j3, POS_j4, POS_j5, POS_j6, POS_eL(1:2,1), POS_eR(1:2,1), SV_tm.R0(1:2,1), SV_d.Q0(3,1), SV_tm.Q0(3,1), tm_dammy_R0

%robot4
fidw_d4 = fopen( strcat( datpath, '\DualArm4.dat' ), 'r' ); 
tmp_d4 = fscanf( fidw_d4, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\n ', [26 inf] );
tmp_d4 = tmp_d4';   % 時間1 ベース位置2 各関節位置2,2,2,2,2,2，左手先位置2，右手先位置2，ターゲット重心位置2，ロボットベース姿勢1，ターゲット姿勢1
fclose( fidw_d4 );   % time, SV_d.R0(1:2,1), POS_j1, POS_j2, POS_j3, POS_j4, POS_j5, POS_j6, POS_eL(1:2,1), POS_eR(1:2,1), SV_tm.R0(1:2,1), SV_d.Q0(3,1), SV_tm.Q0(3,1), tm_dammy_R0

% 接触フラグ
%%% time時間1，左接触フラグ，右接触フラグ，flag_1，flag_2，flag_1_1，flag_1_2，flag_2_1，flag_2_2, catchflag
fidw_cf = fopen( strcat( datpath, '\ContactFlag.dat'), 'r' );
tmp_cf = fscanf( fidw_cf, '%g\t %g\t%g\t%g\t%g\t%g\t%g\t %g\n %g\n', [9 inf] );
tmp_cf = tmp_cf';
fclose( fidw_cf );

% 変形量
%%% time時間1，
fidw_ft = fopen( strcat( datpath, '\ForceTorque.dat'), 'r' );
tmp_ft = fscanf( fidw_ft, '%g\t  %g\t%g\t%g\t%g\t%g\t%g\t %g\t%g\t%g\t%g\t%g\t%g\t %g\t%g\t%g\t%g\t%g\t%g\n', [19 inf] );
tmp_ft = tmp_ft';
fclose( fidw_ft );

%energy　運動量（ターゲット、ロボット、系全体、系全体（手計算）、ベース）
fidw_en = fopen( strcat( datpath, '\Energy.dat' ), 'r' ); 
tmp_en = fscanf( fidw_en, '%g\t  %g\t%g\t%g\t%g\t%g\t%g\t  %g\t%g\t%g\t%g\t%g\t%g\t  %g\t%g\t%g\t%g\t%g\t%g\t　%g\t%g\t%g\t%g\t%g\t%g\t %g\t%g\t%g\t%g\t%g\t%g\n %g\t%g\t%g\t%g\t%g\t%g\n', [37 inf] );
tmp_en = tmp_en';  %PP_d PP_tm PP PP_2 PP_2のbase,PP_2のtar
fclose( fidw_en );   

%energy
fidw_thesis = fopen( strcat( datpath, '\Thesis.dat' ), 'r' ); 
tmp_thesis = fscanf( fidw_thesis, '%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\t  %g\t%g\t%g\n', [28 inf] );
tmp_thesis = tmp_thesis';   
fclose( fidw_thesis );  


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 動画の作成 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% {

%n = 50;
%sen1 = 2;
%sen2 = 2;
%sen3 = 1.5;
framerate = 100;   % d_time=0.01でだいたい実時間と同じに

%%%%%%%%%% 長さ設定 %%%%%%%%%%
half_target = tmp_length(:,1);

%%%%%%%%%% 変数設定 %%%%%%%%%%
mvtime = tmp_movie(:,1);   % time 経過刻み時間

% ターゲット角１～８の座標
pos1_tr = tmp_movie(:,2:4);      % SV_tm.R0(1:2,1) ターゲット重心位置x,y   
pos2_tr = tmp_movie(:,5:7);      % SV_tm.Q0(3,1)   ターゲット重心姿勢
pos3_tr = tmp_movie(:,8:10);     % SV_tm.Q0(3,1)   ターゲット重心姿勢
pos4_tr = tmp_movie(:,11:13);
pos5_tr = tmp_movie(:,14:16);
pos6_tr = tmp_movie(:,17:19);
pos7_tr = tmp_movie(:,20:22);
pos8_tr = tmp_movie(:,23:25);

% pos1_trr = tmp_movie2(:,2:4);   % SV_tm.R0(1:2,1) ターゲット重心位置x,y
% pos2_trr = tmp_movie2(:,5:7);     % SV_tm.Q0(3,1)   ターゲット重心姿勢
% pos3_trr = tmp_movie2(:,8:10);     % SV_tm.Q0(3,1)   ターゲット重心姿勢
% pos4_trr = tmp_movie2(:,11:13);
% pos5_trr = tmp_movie2(:,14:16);
% pos6_trr = tmp_movie2(:,17:19);
% pos7_trr = tmp_movie2(:,20:22);
% pos8_trr = tmp_movie2(:,23:25);
%ターゲットの幾何中心位置
ts_R0 = (pos1_tr + pos2_tr + pos3_tr + pos4_tr + pos5_tr + pos6_tr + pos7_tr + pos8_tr)/8;


%ロボット
rbt1 = tmp_d(:,2:4);   % SV_tm.R0(1:2,1) ターゲット重心位置x,y
rbt2 = tmp_d(:,5:7);     % SV_tm.Q0(3,1)   ターゲット重心姿勢
rbt3 = tmp_d(:,8:10);     % SV_tm.Q0(3,1)   ターゲット重心姿勢
rbt4 = tmp_d(:,11:13);
rbt5 = tmp_d(:,14:16);
rbt6 = tmp_d(:,17:19);
rbt7 = tmp_d(:,20:22);
rbt8 = tmp_d(:,23:25);
l_1 = tmp_d(:,26:28);
l_2 = tmp_d(:,29:31);
l_3 = tmp_d(:,32:34);
l_4 = tmp_d(:,35:37);
l_5 = tmp_d(:,38:40);
l_6 = tmp_d(:,41:43);
r_1 = tmp_d(:,44:46);
r_2 = tmp_d(:,47:49);
r_3 = tmp_d(:,50:52);
r_4 = tmp_d(:,53:55);
r_5 = tmp_d(:,56:58);
r_6 = tmp_d(:,59:61);

n_time = length(tmp_d4);

%事前割り当て２
VertexData = zeros(8,3,701);
VertexDatai = zeros(8,3,701);
PatchData_X = zeros(4,6,701);
PatchData_Y = zeros(4,6,701);
PatchData_Z = zeros(4,6,701);
%PatchData_Xi = zeros(,,701,6);
%PatchData_Yi = zeros(,,701,6);
%PatchData_Zi = zeros(,,701,6);
RR = zeros(701,3,6);
POS_e = zeros(701,3,6);
%F = zeros(1,696); なんかバグるから書かない
%hi= zeros(1,6);
%pl
%pr

%オブジェクトクロージャーの　頂点（Vertex）と　辺（patch）
%Compute propagation of vertices and patches

% for i_time=1:n_time
%     for k = 1:3:22
%     VertexData((k+2)/3,:,i_time) = tmp_d4(i_time,k+1:k+3);
%     end
%     [X,Y,Z] = GeoPatMakeBlock(VertexData(:,:,i_time));
%     PatchData_X(:,:,i_time) = X;
%     PatchData_Y(:,:,i_time) = Y;
%     PatchData_Z(:,:,i_time) = Z;
% end


k=1;
 clearvars POS_e
 clearvars RR
for i=1:6 %iは手先番号,jはxyz
    for j=1:3
        RR(:,j,i) = tmp_d2(:,k);
        k=k+1;
    end
end
for i=1:6
    for j=1:3
        POS_e(:,j,i) = tmp_d2(:,k);
        k=k+1;
    end
end
 
% ６つの各手先を中心とするオブジェクトクロージャー　格納
% for i = 1:6%手先番号
% for i_time=1:n_time
%     VertexDatai(1,:,i_time) = pos8_trr(i_time,:) - ts_R0(i_time,:) + POS_e(i_time,:,i);%vertexnum,xyz,time
%     VertexDatai(2,:,i_time) = pos4_trr(i_time,:) - ts_R0(i_time,:) + POS_e(i_time,:,i);%vertexnum,xyz,time
%     VertexDatai(3,:,i_time) = pos6_trr(i_time,:) - ts_R0(i_time,:) + POS_e(i_time,:,i);%vertexnum,xyz,time
%     VertexDatai(4,:,i_time) = pos7_trr(i_time,:) - ts_R0(i_time,:) + POS_e(i_time,:,i);%vertexnum,xyz,time
%     VertexDatai(5,:,i_time) = pos2_trr(i_time,:) - ts_R0(i_time,:) + POS_e(i_time,:,i);%vertexnum,xyz,time
%     VertexDatai(6,:,i_time) = pos5_trr(i_time,:) - ts_R0(i_time,:) + POS_e(i_time,:,i);%vertexnum,xyz,time
%     VertexDatai(7,:,i_time) = pos3_trr(i_time,:) - ts_R0(i_time,:) + POS_e(i_time,:,i);%vertexnum,xyz,time
%     VertexDatai(8,:,i_time) = pos1_trr(i_time,:) - ts_R0(i_time,:) + POS_e(i_time,:,i);%vertexnum,xyz,time
%     [Xi,Yi,Zi] = GeoPatMakeBlock(VertexDatai(:,:,i_time));
%     PatchData_Xi(:,:,i_time,i) = Xi;
%     PatchData_Yi(:,:,i_time,i) = Yi;
%     PatchData_Zi(:,:,i_time,i) = Zi;
% end
% 
% end
%%%%%%%%%% 接触・カウントフラグ設定 %%%%%%%%%%
frame_n = 1;             
movie_d_time = 0.001 ;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 初期図描画 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(n_figure);
hold on;
box on;
grid on;
grid minor;
left =   400;
bottom = 30;
width =  750;
height = 750;
position = [ left bottom width height ];
% set( gcf, 'Position', position, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );

 set( gcf, 'Color', 'white', 'Resize', 'off', 'Renderer', 'OpenGL' );
 set( gca, 'FontName', 'Times New Roman', 'FontSize', 25 ); 

%axis vis3d
axis equal  % x,y軸を等間隔にする   % axis square   % 系の重心を中心に描画する?
xlim( [ -0.5, 0.5 ] );
ylim( [ -0.3, 0.7 ] );
xlabel( 'X [m]', 'FontName', 'Times New Roman', 'FontSize', 25 );
ylabel( 'Y [m]', 'FontName', 'Times New Roman', 'FontSize', 25 );
f = figure(n_figure);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 動画保存 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% hold on;
title( sprintf( 'Time = %0.3f [s]', tmp_movie(:,1) ), 'FontSize', 26, 'FontName', 'Times New Roman' )
moviename = strcat( movpath, '\', timepath, '_DualArm_TargetBox_TrackingContact_Movie_2___FrameRate', num2str(framerate), '.mp4' );
writerObj = VideoWriter( moviename,'MPEG-4' );
set( writerObj, 'FrameRate', framerate, 'Quality', 100 );
open( writerObj );


%%%%%%%%%%%%%%%%%%%% ライティング・テクスチャ設定 %%%%%%%%%%%%%%%%%%%%
brighten(0.6)
h=light;
h.Color=[0.5 0.6 1];
h.Position=[-4 -4 10];
h.Style='infinite';%local low beam
% white high beam [1 0 1] default
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% {
%%%
% pause(5)

%%%%%%%%%% 図を更新 %%%%%%%%%%
i = 1;
hold on
X1=[pos1_tr(i,1) pos2_tr(i,1) pos4_tr(i,1) pos3_tr(i,1) pos1_tr(i,1)];  %１面のx情報
Y1=[pos1_tr(i,2) pos2_tr(i,2) pos4_tr(i,2) pos3_tr(i,2) pos1_tr(i,2)];
Z1=[pos1_tr(i,3) pos2_tr(i,3) pos4_tr(i,3) pos3_tr(i,3) pos1_tr(i,3)];
X2=[pos1_tr(i,1) pos2_tr(i,1) pos6_tr(i,1) pos5_tr(i,1) pos1_tr(i,1)];
Y2=[pos1_tr(i,2) pos2_tr(i,2) pos6_tr(i,2) pos5_tr(i,2) pos1_tr(i,2)];
Z2=[pos1_tr(i,3) pos2_tr(i,3) pos6_tr(i,3) pos5_tr(i,3) pos1_tr(i,3)];
X3=[pos1_tr(i,1) pos3_tr(i,1) pos7_tr(i,1) pos5_tr(i,1) pos1_tr(i,1)];
Y3=[pos1_tr(i,2) pos3_tr(i,2) pos7_tr(i,2) pos5_tr(i,2) pos1_tr(i,2)];
Z3=[pos1_tr(i,3) pos3_tr(i,3) pos7_tr(i,3) pos5_tr(i,3) pos1_tr(i,3)];
X4=[pos2_tr(i,1) pos4_tr(i,1) pos8_tr(i,1) pos6_tr(i,1) pos2_tr(i,1)];
Y4=[pos2_tr(i,2) pos4_tr(i,2) pos8_tr(i,2) pos6_tr(i,2) pos2_tr(i,2)];
Z4=[pos2_tr(i,3) pos4_tr(i,3) pos8_tr(i,3) pos6_tr(i,3) pos2_tr(i,3)];
X5=[pos3_tr(i,1) pos4_tr(i,1) pos8_tr(i,1) pos7_tr(i,1) pos3_tr(i,1)];
Y5=[pos3_tr(i,2) pos4_tr(i,2) pos8_tr(i,2) pos7_tr(i,2) pos3_tr(i,2)];
Z5=[pos3_tr(i,3) pos4_tr(i,3) pos8_tr(i,3) pos7_tr(i,3) pos3_tr(i,3)];
X6=[pos5_tr(i,1) pos6_tr(i,1) pos8_tr(i,1) pos7_tr(i,1) pos5_tr(i,1)];
Y6=[pos5_tr(i,2) pos6_tr(i,2) pos8_tr(i,2) pos7_tr(i,2) pos5_tr(i,2)];
Z6=[pos5_tr(i,3) pos6_tr(i,3) pos8_tr(i,3) pos7_tr(i,3) pos5_tr(i,3)];

%ロボット
x1=[rbt1(i,1) rbt2(i,1) rbt4(i,1) rbt3(i,1) rbt1(i,1)];
y1=[rbt1(i,2) rbt2(i,2) rbt4(i,2) rbt3(i,2) rbt1(i,2)];
z1=[rbt1(i,3) rbt2(i,3) rbt4(i,3) rbt3(i,3) rbt1(i,3)];
x2=[rbt1(i,1) rbt2(i,1) rbt6(i,1) rbt5(i,1) rbt1(i,1)];
y2=[rbt1(i,2) rbt2(i,2) rbt6(i,2) rbt5(i,2) rbt1(i,2)];
z2=[rbt1(i,3) rbt2(i,3) rbt6(i,3) rbt5(i,3) rbt1(i,3)];
x3=[rbt1(i,1) rbt3(i,1) rbt7(i,1) rbt5(i,1) rbt1(i,1)];
y3=[rbt1(i,2) rbt3(i,2) rbt7(i,2) rbt5(i,2) rbt1(i,2)];
z3=[rbt1(i,3) rbt3(i,3) rbt7(i,3) rbt5(i,3) rbt1(i,3)];
x4=[rbt2(i,1) rbt4(i,1) rbt8(i,1) rbt6(i,1) rbt2(i,1)];
y4=[rbt2(i,2) rbt4(i,2) rbt8(i,2) rbt6(i,2) rbt2(i,2)];
z4=[rbt2(i,3) rbt4(i,3) rbt8(i,3) rbt6(i,3) rbt2(i,3)];
x5=[rbt3(i,1) rbt4(i,1) rbt8(i,1) rbt7(i,1) rbt3(i,1)];
y5=[rbt3(i,2) rbt4(i,2) rbt8(i,2) rbt7(i,2) rbt3(i,2)];
z5=[rbt3(i,3) rbt4(i,3) rbt8(i,3) rbt7(i,3) rbt3(i,3)];
x6=[rbt5(i,1) rbt6(i,1) rbt8(i,1) rbt7(i,1) rbt5(i,1)];
y6=[rbt5(i,2) rbt6(i,2) rbt8(i,2) rbt7(i,2) rbt5(i,2)];
z6=[rbt5(i,3) rbt6(i,3) rbt8(i,3) rbt7(i,3) rbt5(i,3)];

lx=[l_1(i,1) l_2(i,1) l_3(i,1) l_4(i,1) l_5(i,1) l_6(i,1)];
ly=[l_1(i,2) l_2(i,2) l_3(i,2) l_4(i,2) l_5(i,2) l_6(i,2)];
lz=[l_1(i,3) l_2(i,3) l_3(i,3) l_4(i,3) l_5(i,3) l_6(i,3)];

rx=[r_1(i,1) r_2(i,1) r_3(i,1) r_4(i,1) r_5(i,1) r_6(i,1)];
ry=[r_1(i,2) r_2(i,2) r_3(i,2) r_4(i,2) r_5(i,2) r_6(i,2)];
rz=[r_1(i,3) r_2(i,3) r_3(i,3) r_4(i,3) r_5(i,3) r_6(i,3)];

for l=1:3
    for k=1:3
        pl(i,1:4,k,l)=[l_6(i,k) (RR(i,k,1)+RR(i,k,2)+RR(i,k,3) )/3 RR(i,k,l) POS_e(i,k,l)];%p1(i,xxx,xyz,手先番号)
    end
end
t=1;
for l=4:6
    for k=1:3
        pr(i,1:4,k,t)=[r_6(i,k) (RR(i,k,4)+RR(i,k,5)+RR(i,k,6) )/3 RR(i,k,l) POS_e(i,k,l)];%pr(xyz,手先番号)
    end
    t=t+1;
end


sen1 = 2.5; %1.5; %2.5;　腕
sen2 = 2.5; %手先
       %ターゲット　描画
%        Sur1=patch(X1,Y1,Z1,'r');alpha(Sur1,1);
%        Sur2=patch(X2,Y2,Z2,'r');alpha(Sur2,1);
%        Sur3=patch(X3,Y3,Z3,'r');alpha(Sur3,1);
%        Sur4=patch(X4,Y4,Z4,'g');alpha(Sur4,1);
%        Sur5=patch(X5,Y5,Z5,'g');alpha(Sur5,1);
%        Sur6=patch(X6,Y6,Z6,'g');alpha(Sur6,1);
            %向かい合わせ3組
       Sur1=patch(X1,Y1,Z1,'r');alpha(Sur1,1);
       Sur2=patch(X2,Y2,Z2,'g');alpha(Sur2,1);
       Sur3=patch(X3,Y3,Z3,'b');alpha(Sur3,1);
       Sur4=patch(X4,Y4,Z4,'b');alpha(Sur4,0.8);
       Sur5=patch(X5,Y5,Z5,'g');alpha(Sur5,0.8);
       Sur6=patch(X6,Y6,Z6,'r');alpha(Sur6,0.8);
       
       
       %ロボット　描画
       sur1=patch(x1,y1,z1,'k');alpha(sur1,0.6);
       sur2=patch(x2,y2,z2,'k');alpha(sur2,0.6);
       sur3=patch(x3,y3,z3,'k');alpha(sur3,0.6);
       sur4=patch(x4,y4,z4,'k');alpha(sur4,0.6);
       sur5=patch(x5,y5,z5,'k');alpha(sur5,0.6);
       sur6=patch(x6,y6,z6,'k');alpha(sur6,0.6);
       
       %ロボット腕
       l_arm=plot3(lx,ly,lz, 'LineWidth', sen1);
       r_arm=plot3(rx,ry,rz, 'LineWidth', sen1);
       
        %手先球の描画
       p_l = zeros(1,3);
       p_r = zeros(1,3);
       ballplot = zeros(1,6);
       for l=1:3
           p_l(l)=plot3(pl(i,1:4,1,l),pl(i,1:4,2,l),pl(i,1:4,3,l), 'LineWidth', sen2);
           p_r(l)=plot3(pr(i,1:4,1,l),pr(i,1:4,2,l),pr(i,1:4,3,l), 'LineWidth', sen2);
       end
       for k=1:6
               ballplot(k) = plot3(POS_e(i,1,k),POS_e(i,2,k),POS_e(i,3,k),'o','MarkerEdgeColor','c','MarkerFaceColor','c','MarkerSize',100*6*r_tip /2, 'LineWidth', sen2);
       end
       
       
       %オブジェクトクロージャー　描画
%        h = patch(PatchData_X(:,:,1),PatchData_Y(:,:,1),PatchData_Z(:,:,1),'y'); 
%        set(h,'EdgeColor','black','FaceColor','red','LineWidth',1);
%        set(h,'EraseMode','normal');

%        for i = 1:6
%        hi(i) = patch(PatchData_Xi(:,:,1,i),PatchData_Yi(:,:,1,i),PatchData_Zi(:,:,1,i));
%        set(hi(i),'EdgeColor','black','FaceColor','red','FaceAlpha',0.1,'EdgeAlpha',0.7,'LineWidth',1);
% %        set(hi(i),'EdgeColor','black','FaceColor','yellow','FaceAlpha',0.1,'LineAlpha',0.1,'LineWidth',2,'LineStyle','--');
%        set(hi(i),'EraseMode','normal');
%        end
%        
       
       %（見せかけの）球面の描画
%        [x,y,z] = sphere;
%         x = x .* 0.15;
%         y = y .* 0.15;
%         z = z .* 0.15;
%         
%         ORI = rpy2dc(tmp_thesis(i,17),tmp_thesis(i,18),tmp_thesis(i,19));
%         for iii = 1:21
%             for jjj =1:21
%                 X(iii,jjj) = ORI(1,1:3) * [x(iii,jjj) y(iii,jjj) z(iii,jjj) ]';
%                 Y(iii,jjj) = ORI(2,1:3) * [x(iii,jjj) y(iii,jjj) z(iii,jjj) ]';
%                 Z(iii,jjj) = ORI(3,1:3) * [x(iii,jjj) y(iii,jjj) z(iii,jjj) ]';
%             end
%         end
%         ball = mesh(X + tmp_thesis(i,14),Y + tmp_thesis(i,15),Z + tmp_thesis(i,16))

%        set(ball,'EraseMode','normal')

       %ターゲット　描画
       set(Sur1,'EraseMode','normal')
       set(Sur2,'EraseMode','normal')
       set(Sur3,'EraseMode','normal')
       set(Sur4,'EraseMode','normal')
       set(Sur5,'EraseMode','normal')
       set(Sur6,'EraseMode','normal')
       
       %ロボット　描画
       set(sur1,'EraseMode','normal')
       set(sur2,'EraseMode','normal')
       set(sur3,'EraseMode','normal')
       set(sur4,'EraseMode','normal')
       set(sur5,'EraseMode','normal')
       set(sur6,'EraseMode','normal')
       
       set(l_arm,'EraseMode','normal')
       set(r_arm,'EraseMode','normal')
       
       set(p_l,'EraseMode','normal')
       set(p_r,'EraseMode','normal')
       set(ballplot,'EraseMode','normal')
       
        xlabel('x [m]','FontSize',10);
        ylabel('y [m]','FontSize',10);
        zlabel('z [m]','FontSize',10);
        set(gca,'FontSize',10);
   
%           カメラ設定
%         axis vis3d equal;
%   位置
%campos([-1.376246275769847,6.229853221069663,5.874013303222815]);
% campos([-1.176246275769847, 10 ,0]);


%   アングル・方向
%camtarget([0.060934217987478,0.204217109051259,0.090489244256438]); %カメラターゲットの位置　
%   方位角と仰角　　仰角が正＝上側から見下ろす　負＝下側から見上げる
%view([-1.665847805987102e+02,43.034000782453759]);  
%関数 view は視線の方向を制御しますが、線の開始場所もカメラ位置も制御しません。
%より詳細に制御するにはcampos、camtarget、camup、camva、camroll、camproj などのカメラ関数を使用します。

% view([-1.320668812633383e+02,24.160779615108339]);
% view([-180,0]);    %ターゲット側からロボット側方向
% view([98.723827700500067,51.270728675897281]);
% view([37.5,30]);
%          view([0,90,0]);
% view([180,0]);

%真上から
%         view([-180,90]);

% % コリジョン証明
% view([155,25]);

% %斜めから
% campos([4.474614276151736,7.844404533648864,3.817372775750953]);

campos([3.14423637287249,6.997692518583239,3.505272559408342]);
camtarget([0.042649054875438,0.346317050573879,0.083049598996298]);


        camlight;
        grid on;
         lim = 0.6;
%         xlim([-lim,lim]);
%         ylim([-lim,lim]);
        xlim([-0.6,0.6]);
        
%         ylim([-0.4,0.8]);
%コリジョン
        ylim([-0.3, 0.6]);


%         xlim([-0.45,0.45]);
%         ylim([-0.2,0.7]);
%         yticks([-0.2 0 0.2 0.4 0.6])
%         zlim([-lim,lim]);
        zlim([-0.4,0.4]);
        
        
figure(n_figure)
title( 'Time = 0.00 [s]', 'FontSize', 24, 'FontName', 'Times New Roman' );
saveas( figure(n_figure), strcat( movpath, '\', timepath, '_DualArm_TargetMaru_TrackingContact_2_FirstFigure.fig' ) );
saveas( figure(n_figure), strcat( movpath, '\', timepath, '_DualArm_TargetMaru_TrackingContact_2_FirstFigure.png' ) );
hold off

while i <= length( tmp_movie(:,1) )   
%ターゲット
X1=[pos1_tr(i,1) pos2_tr(i,1) pos4_tr(i,1) pos3_tr(i,1) pos1_tr(i,1)];  %１面のx情報
Y1=[pos1_tr(i,2) pos2_tr(i,2) pos4_tr(i,2) pos3_tr(i,2) pos1_tr(i,2)];
Z1=[pos1_tr(i,3) pos2_tr(i,3) pos4_tr(i,3) pos3_tr(i,3) pos1_tr(i,3)];
X2=[pos1_tr(i,1) pos2_tr(i,1) pos6_tr(i,1) pos5_tr(i,1) pos1_tr(i,1)];
Y2=[pos1_tr(i,2) pos2_tr(i,2) pos6_tr(i,2) pos5_tr(i,2) pos1_tr(i,2)];
Z2=[pos1_tr(i,3) pos2_tr(i,3) pos6_tr(i,3) pos5_tr(i,3) pos1_tr(i,3)];
X3=[pos1_tr(i,1) pos3_tr(i,1) pos7_tr(i,1) pos5_tr(i,1) pos1_tr(i,1)];
Y3=[pos1_tr(i,2) pos3_tr(i,2) pos7_tr(i,2) pos5_tr(i,2) pos1_tr(i,2)];
Z3=[pos1_tr(i,3) pos3_tr(i,3) pos7_tr(i,3) pos5_tr(i,3) pos1_tr(i,3)];
X4=[pos2_tr(i,1) pos4_tr(i,1) pos8_tr(i,1) pos6_tr(i,1) pos2_tr(i,1)];
Y4=[pos2_tr(i,2) pos4_tr(i,2) pos8_tr(i,2) pos6_tr(i,2) pos2_tr(i,2)];
Z4=[pos2_tr(i,3) pos4_tr(i,3) pos8_tr(i,3) pos6_tr(i,3) pos2_tr(i,3)];
X5=[pos3_tr(i,1) pos4_tr(i,1) pos8_tr(i,1) pos7_tr(i,1) pos3_tr(i,1)];
Y5=[pos3_tr(i,2) pos4_tr(i,2) pos8_tr(i,2) pos7_tr(i,2) pos3_tr(i,2)];
Z5=[pos3_tr(i,3) pos4_tr(i,3) pos8_tr(i,3) pos7_tr(i,3) pos3_tr(i,3)];
X6=[pos5_tr(i,1) pos6_tr(i,1) pos8_tr(i,1) pos7_tr(i,1) pos5_tr(i,1)];
Y6=[pos5_tr(i,2) pos6_tr(i,2) pos8_tr(i,2) pos7_tr(i,2) pos5_tr(i,2)];
Z6=[pos5_tr(i,3) pos6_tr(i,3) pos8_tr(i,3) pos7_tr(i,3) pos5_tr(i,3)];
%ロボット
x1=[rbt1(i,1) rbt2(i,1) rbt4(i,1) rbt3(i,1) rbt1(i,1)];
y1=[rbt1(i,2) rbt2(i,2) rbt4(i,2) rbt3(i,2) rbt1(i,2)];
z1=[rbt1(i,3) rbt2(i,3) rbt4(i,3) rbt3(i,3) rbt1(i,3)];
x2=[rbt1(i,1) rbt2(i,1) rbt6(i,1) rbt5(i,1) rbt1(i,1)];
y2=[rbt1(i,2) rbt2(i,2) rbt6(i,2) rbt5(i,2) rbt1(i,2)];
z2=[rbt1(i,3) rbt2(i,3) rbt6(i,3) rbt5(i,3) rbt1(i,3)];
x3=[rbt1(i,1) rbt3(i,1) rbt7(i,1) rbt5(i,1) rbt1(i,1)];
y3=[rbt1(i,2) rbt3(i,2) rbt7(i,2) rbt5(i,2) rbt1(i,2)];
z3=[rbt1(i,3) rbt3(i,3) rbt7(i,3) rbt5(i,3) rbt1(i,3)];
x4=[rbt2(i,1) rbt4(i,1) rbt8(i,1) rbt6(i,1) rbt2(i,1)];
y4=[rbt2(i,2) rbt4(i,2) rbt8(i,2) rbt6(i,2) rbt2(i,2)];
z4=[rbt2(i,3) rbt4(i,3) rbt8(i,3) rbt6(i,3) rbt2(i,3)];
x5=[rbt3(i,1) rbt4(i,1) rbt8(i,1) rbt7(i,1) rbt3(i,1)];
y5=[rbt3(i,2) rbt4(i,2) rbt8(i,2) rbt7(i,2) rbt3(i,2)];
z5=[rbt3(i,3) rbt4(i,3) rbt8(i,3) rbt7(i,3) rbt3(i,3)];
x6=[rbt5(i,1) rbt6(i,1) rbt8(i,1) rbt7(i,1) rbt5(i,1)];
y6=[rbt5(i,2) rbt6(i,2) rbt8(i,2) rbt7(i,2) rbt5(i,2)];
z6=[rbt5(i,3) rbt6(i,3) rbt8(i,3) rbt7(i,3) rbt5(i,3)];

lx=[l_1(i,1) l_2(i,1) l_3(i,1) l_4(i,1) l_5(i,1) l_6(i,1)];
ly=[l_1(i,2) l_2(i,2) l_3(i,2) l_4(i,2) l_5(i,2) l_6(i,2)];
lz=[l_1(i,3) l_2(i,3) l_3(i,3) l_4(i,3) l_5(i,3) l_6(i,3)];

rx=[r_1(i,1) r_2(i,1) r_3(i,1) r_4(i,1) r_5(i,1) r_6(i,1)];
ry=[r_1(i,2) r_2(i,2) r_3(i,2) r_4(i,2) r_5(i,2) r_6(i,2)];
rz=[r_1(i,3) r_2(i,3) r_3(i,3) r_4(i,3) r_5(i,3) r_6(i,3)];

%ごまかし球の数値
% ORI = rpy2dc(tmp_thesis(i,17),tmp_thesis(i,18),tmp_thesis(i,19));
%         for iii = 1:21
%             for jjj =1:21
%                 X(iii,jjj) = ORI(1,1:3) * [x(iii,jjj) y(iii,jjj) z(iii,jjj) ]';
%                 Y(iii,jjj) = ORI(2,1:3) * [x(iii,jjj) y(iii,jjj) z(iii,jjj) ]';
%                 Z(iii,jjj) = ORI(3,1:3) * [x(iii,jjj) y(iii,jjj) z(iii,jjj) ]';
%             end
%         end

for l=1:3
    for k=1:3
        pl(i,1:4,k,l)=[l_6(i,k) (RR(i,k,1)+RR(i,k,2)+RR(i,k,3) )/3 RR(i,k,l) POS_e(i,k,l)];%p1(i,xxx,xyz,手先番号)
    end
end
t=1;
for l=4:6
    for k=1:3
        pr(i,1:4,k,t)=[r_6(i,k) (RR(i,k,4)+RR(i,k,5)+RR(i,k,6) )/3 RR(i,k,l) POS_e(i,k,l)];%pr(xyz,手先番号)
    end
    t=t+1;
end

    if rem( mvtime(i), movie_d_time ) == 0   % mvtime(i)をmovie_d_timeで除算した後の剰余が0のとき(割り切れるとき)
       
       if rem( mvtime(i), 0.01 ) == 0
           %title( strcat( 'Time = ', num2str(mvtime(i)), ' [s]' ), 'FontSize', 25, 'FontName', 'Times New Roman' );
           title( sprintf( 'Time = %0.2f [s]', mvtime(i) ), 'FontSize', 22, 'FontName', 'Times New Roman' );

       end
       
%ターゲット　値更新
       set(Sur1,'XData',X1)
       set(Sur1,'YData',Y1)
       set(Sur1,'ZData',Z1)
       set(Sur2,'XData',X2)
       set(Sur2,'YData',Y2)
       set(Sur2,'ZData',Z2)
       set(Sur3,'XData',X3)
       set(Sur3,'YData',Y3)
       set(Sur3,'ZData',Z3)
       set(Sur4,'XData',X4)
       set(Sur4,'YData',Y4)
       set(Sur4,'ZData',Z4)
       set(Sur5,'XData',X5)
       set(Sur5,'YData',Y5)
       set(Sur5,'ZData',Z5)
       set(Sur6,'XData',X6)
       set(Sur6,'YData',Y6)
       set(Sur6,'ZData',Z6)
       
%ロボット　値更新
       set(sur1,'XData',x1)
       set(sur1,'YData',y1)
       set(sur1,'ZData',z1)
       set(sur2,'XData',x2)
       set(sur2,'YData',y2)
       set(sur2,'ZData',z2)
       set(sur3,'XData',x3)
       set(sur3,'YData',y3)
       set(sur3,'ZData',z3)
       set(sur4,'XData',x4)
       set(sur4,'YData',y4)
       set(sur4,'ZData',z4)
       set(sur5,'XData',x5)
       set(sur5,'YData',y5)
       set(sur5,'ZData',z5)
       set(sur6,'XData',x6)
       set(sur6,'YData',y6)
       set(sur6,'ZData',z6)
       
       %ロボット　手足
       set(l_arm,'XData',lx)
       set(l_arm,'YData',ly)
       set(l_arm,'ZData',lz)
       set(r_arm,'XData',rx)
       set(r_arm,'YData',ry)
       set(r_arm,'ZData',rz)
       
       for l=1:3
           set(p_l(l),'XData',pl(i,1:4,1,l))
           set(p_l(l),'YData',pl(i,1:4,2,l))
           set(p_l(l),'ZData',pl(i,1:4,3,l))
           set(p_r(l),'XData',pr(i,1:4,1,l))
           set(p_r(l),'YData',pr(i,1:4,2,l))
           set(p_r(l),'ZData',pr(i,1:4,3,l))
       end
       for k=1:6
           set(ballplot(k),'XData',POS_e(i,1,k))
           set(ballplot(k),'YData',POS_e(i,2,k))
           set(ballplot(k),'ZData',POS_e(i,3,k))
       end
       
      % オブジェクトクロージャー　描画
%            set(h,'XData',PatchData_X(:,:,i));
%            set(h,'YData',PatchData_Y(:,:,i));
%            set(h,'ZData',PatchData_Z(:,:,i));
%        for ia = 1:6
%            set(hi(ia),'XData',PatchData_Xi(:,:,i,ia));
%            set(hi(ia),'YData',PatchData_Yi(:,:,i,ia));
%            set(hi(ia),'ZData',PtchData_Zi(:,:,i,ia));
%        end
       
       %見せかけの球の描画？
%            set(ball,'XData',X(:,:) + tmp_thesis(i,14))
%            set(ball,'YData',Y(:,:) + tmp_thesis(i,15))
%            set(ball,'ZData',Z(:,:) + tmp_thesis(i,16))


       F( frame_n ) = getframe( n_figure );
        hold off
       drawnow                               
    
       writeVideo( writerObj, F( frame_n ) );
       frame_n = frame_n + 1;

    end 

    i = i + 1;

%      pause(1);

end

% 最後の図を保存
figure(n_figure)
title( sprintf( 'Time = %0.2f [s]', tmp_movie(end,1) ), 'FontSize', 22, 'FontName', 'Times New Roman' );
saveas( figure(n_figure), strcat( movpath, '\', timepath, '_DualArm_TargetMaru_TrackingContact_2_LastFigure.fig' ) );
saveas( figure(n_figure), strcat( movpath, '\', timepath, '_DualArm_TargetMaru_TrackingContact_2_LastFigure.png' ) );

close( writerObj );
%}
   % 動画作成終了


%%
%%%%%%%%%% 値読み込み %%%%%%%%%%

% {


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% グラフ描画・保存 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_figure = n_figure + 1;

% sen = 2.2;
sen = 3;

left = 300;
bottom = 30;
width = 900;
height = 750;
position = [ left bottom width height ];
% 
% % コンタクトフラグ左
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% % title( 'left contactflag', 'FontSize', 25 );
% set( gca, 'FontName', 'Times New Roman' , 'FontSize', 25);
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_cf(:,1), tmp_cf(:,2), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_cf(:,1), tmp_cf(:,3), 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_cf(:,1), tmp_cf(:,4), 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% legend( {'1', '2', '3'},'Location','northwest'   );
% xlabel( 'Time [s]' ); 
% ylabel( 'ContactFlag' );
% xlim([0 7])
% ylim([0 1.2])
% fig01 = strcat(figpath,'\',timepath,'_01_コンタクトフラグ左');
% saveas(figure(n_figure),strcat(fig01,'.fig'))
% png01 = strcat(pngpath,'\',timepath,'_01_コンタクトフラグ左');
% saveas(figure(n_figure),strcat(png01,'.png'))
% n_figure = n_figure + 1;
% 
% % コンタクトフラグ右
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% % title( 'right contactflag', 'FontSize', 25 );
% set( gca, 'FontName', 'Times New Roman' , 'FontSize', 25);
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_cf(:,1), tmp_cf(:,5), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_cf(:,1), tmp_cf(:,6), 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_cf(:,1), tmp_cf(:,7), 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% legend( {'4', '5', '6'},'Location','northwest'   );
% xlabel( 'Time [s]' ); 
% ylabel( 'ContactFlag' );
% xlim([0 7])
% ylim([0 1.2])
% fig02 = strcat(figpath,'\',timepath,'_02_コンタクトフラグ右');
% saveas(figure(n_figure),strcat(fig02,'.fig'))
% png02 = strcat(pngpath,'\',timepath,'_02_コンタクトフラグ右');
% saveas(figure(n_figure),strcat(png02,'.png'))
% n_figure = n_figure + 1;
% 
% 
% % 左腕めり込み量
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% % title( 'delta left', 'FontSize', 25 );
% set( gca, 'FontName', 'Times New Roman' , 'FontSize', 25);
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_ft(:,1), tmp_ft(:,2), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_ft(:,1), tmp_ft(:,3), 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_ft(:,1), tmp_ft(:,4), 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% legend( {'L1', 'L2', 'L3'},'Location','northwest'   );
% xlabel( 'Time [s]'  , 'FontSize', 25 );
% ylabel( 'delta[m]'  , 'FontSize', 25 );
% xlim([0 7])
% %ylim([0 1.2])
% fig03 = strcat(figpath,'\',timepath,'_03_左腕めり込み量');
% saveas(figure(n_figure),strcat(fig03,'.fig'))
% png03 = strcat(pngpath,'\',timepath,'_03_左腕めり込み量');
% saveas(figure(n_figure),strcat(png03,'.png'))
% n_figure = n_figure + 1;
% 
% % 右腕めり込み量
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% % title( 'delta right', 'FontSize', 25 );
% set( gca, 'FontName', 'Times New Roman' , 'FontSize', 25);
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_ft(:,1), tmp_ft(:,5), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_ft(:,1), tmp_ft(:,6), 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_ft(:,1), tmp_ft(:,7), 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% legend( {'R4', 'R5', 'R6'},'Location','northwest'   );
% xlabel( 'Time [s]' , 'FontSize', 25 ); 
% ylabel( 'delta[m]' , 'FontSize', 25 );  
% xlim([0 7])
% %ylim([0 1.2])
% fig04 = strcat(figpath,'\',timepath,'_04_右腕めり込み量');
% saveas(figure(n_figure),strcat(fig04,'.fig'))
% png04 = strcat(pngpath,'\',timepath,'_04_右腕めり込み量');
% saveas(figure(n_figure),strcat(png04,'.png'))
% n_figure = n_figure + 1;
% 
% % force_i 左右エンドエフェクタに加わる力
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% % title( 'force', 'FontSize', 25 );
% set( gca, 'FontName', 'Times New Roman' , 'FontSize', 25);
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_ft(:,1), tmp_ft(:,8), 'r-', 'linewidth', sen );   %x　左腕？
% plot( tmp_ft(:,1), tmp_ft(:,9), 'g-', 'linewidth', sen );   %y
% plot( tmp_ft(:,1), tmp_ft(:,10), 'b-', 'linewidth', sen );  %z
% plot( tmp_ft(:,1), tmp_ft(:,11), 'm-', 'linewidth', sen );  %x　右腕？
% plot( tmp_ft(:,1), tmp_ft(:,12), 'y-', 'linewidth', sen );  %y
% plot( tmp_ft(:,1), tmp_ft(:,13), 'k-', 'linewidth', sen );  %z
% legend( {'Lx', 'Ly', 'Lz', 'Rx', 'Ry', 'Rz'},'Location','northwest'  );
% xlabel( 'Time [s]' ); 
% ylabel( 'force' );
% xlim([0 7])
% %ylim([0 1.2])
% fig05 = strcat(figpath,'\',timepath,'_05_力');
% saveas(figure(n_figure),strcat(fig05,'.fig'))
% png05 = strcat(pngpath,'\',timepath,'_05_力');
% saveas(figure(n_figure),strcat(png05,'.png'))
% n_figure = n_figure + 1;
% 
% % distance
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% % title( 'distance', 'FontSize', 25 );
% set( gca, 'FontName', 'Times New Roman' , 'FontSize', 25);
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_ft(:,1), tmp_ft(:,14), 'r-', 'linewidth', sen );   %左手先球３つ
% plot( tmp_ft(:,1), tmp_ft(:,15), 'g-', 'linewidth', sen );   
% plot( tmp_ft(:,1), tmp_ft(:,16), 'b-', 'linewidth', sen );   
% plot( tmp_ft(:,1), tmp_ft(:,17), 'm-', 'linewidth', sen );   %右手先球
% plot( tmp_ft(:,1), tmp_ft(:,18), 'y-', 'linewidth', sen ); 
% plot( tmp_ft(:,1), tmp_ft(:,19), 'k-', 'linewidth', sen ); 
% legend( { 'L1', 'L2', 'L3', 'R1', 'R2', 'R3'},'Location','northwest'  );
% xlabel( 'Time [s]' ); 
% ylabel( 'distance' );
% xlim([0 7])
% % ylim([0 0.05])
% fig06 = strcat(figpath,'\',timepath,'_06_distance');
% saveas(figure(n_figure),strcat(fig06,'.fig'))
% png06 = strcat(pngpath,'\',timepath,'_06_distance');
% saveas(figure(n_figure),strcat(png06,'.png'))
% n_figure = n_figure + 1;
% 
% % ベースの重心位置(x座標,y座標) SV_d.R0
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% % title( 'arm angle', 'FontSize', 25 );
% set( gca, 'FontName', 'Times New Roman', 'FontSize', 25);
% set( gcf, 'Position', position, 'Color', 'white' );
% %plot( tmp_d3(:,1), tmp_d3(:,4), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_d3(:,1), tmp_d3(:,5), 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_d3(:,1), tmp_d3(:,6),'g-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_d3(:,1), tmp_d3(:,11), 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_d3(:,1), tmp_d3(:,12), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% legend( { '5','6','11','12'},'Location','northwest'  );
% xlabel( 'Time [s]' ); 
% ylabel( 'arm angle' );
% xlim([0 7])
% %ylim([0 0.05])
% fig07 = strcat(figpath,'\',timepath,'_07_arm angle');
% saveas(figure(n_figure),strcat(fig07,'.fig'))
% png07 = strcat(pngpath,'\',timepath,'_07_arm angle');
% saveas(figure(n_figure),strcat(png07,'.png'))
% n_figure = n_figure + 1;


% % フェーズ
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% % title( 'Phase', 'FontSize', 25 );
% set( gca, 'FontName', 'Times New Roman', 'FontSize', 27);
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_cf(:,1), tmp_cf(:,8), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% xlabel( 'Time [s]' ); 
% ylabel( 'Phase' );
% xlim([0 7])
% fig09 = strcat(figpath,'\',timepath,'_09_Phase');
% saveas(figure(n_figure),strcat(fig09,'.fig'))
% png09 = strcat(pngpath,'\',timepath,'_09_Phase');
% saveas(figure(n_figure),strcat(png09,'.png'))
% n_figure = n_figure + 1;


%---------------------------運動量・角運動量-----------------------------------------------
% ロボットの運動量
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
% title( 'Linear Momentum of Robot' );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40 );
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_en(:,2) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_en(:,3) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_en(:,4) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'   );
xlabel( 'Time [s]' ); 
ylabel( 'Robot Linear Momentum [Ns]', 'FontSize', 32);
xlim([0 7])
fig10 = strcat(figpath,'\', 'ロボット運動量');
saveas(figure(n_figure),strcat(fig10,'.fig'))
png10 = strcat(pngpath,'\', 'ロボット運動量');
saveas(figure(n_figure),strcat(png10,'.png'))
n_figure = n_figure + 1;

% ロボットの角運動量
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
% title( 'Angualar Momentum of Robot' );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40 );
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_en(:,5), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_en(:,6), 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_en(:,7), 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'   );
xlabel( 'Time [s]'); 
ylabel( 'Robot Angular Momentum [Nms]', 'FontSize', 32);
xlim([0 7])
fig11 = strcat(figpath,'\', 'ロボット角運動量');
saveas(figure(n_figure),strcat(fig11,'.fig'))
png11 = strcat(pngpath,'\', 'ロボット角運動量');
saveas(figure(n_figure),strcat(png11,'.png'))
n_figure = n_figure + 1;


% ターゲットの運動量
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
% title( 'Momentum of Target' );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40 );
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_en(:,8) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_en(:,9) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_en(:,10) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'   );
xlabel( 'Time [s]' ); 
ylabel( 'Target Momentum [Ns]');
xlim([0 7])
fig13 = strcat(figpath,'\', 'ターゲット運動量');
saveas(figure(n_figure),strcat(fig13,'.fig'))
png13 = strcat(pngpath,'\', 'ターゲット運動量');
saveas(figure(n_figure),strcat(png13,'.png'))
n_figure = n_figure + 1;


% ターゲットの角運動量
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
% title( 'Angualar Momentum of Target' );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40 );
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_en(:,11) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_en(:,12) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_en(:,13) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'   );
xlabel( 'Time [s]' ); 
ylabel( 'Target Angular Momentum [Nms]' , 'FontSize', 32);
xlim([0 7])
fig13 = strcat(figpath,'\', 'ターゲット角運動量');
saveas(figure(n_figure),strcat(fig13,'.fig'))
png13 = strcat(pngpath,'\', 'ターゲット角運動量');
saveas(figure(n_figure),strcat(png13,'.png'))
n_figure = n_figure + 1;

% % ケージング条件
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% %title( 'Phase', 'FontSize', 16 );
% set( gca, 'FontName', 'Times New Roman', 'FontSize', 40 );
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_cf(:,1), tmp_cf(:,9), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% xlabel( 'Time [s]' ); 
% ylabel( 'Caging judgement' );
% xlim([0 7])
% fig16 = strcat(figpath,'\', 'ケージング');
% saveas(figure(n_figure),strcat(fig16,'.fig'))
% png16 = strcat(pngpath,'\', 'ケージング');
% saveas(figure(n_figure),strcat(png16,'.png'))
% n_figure = n_figure + 1;


% ターゲット 移動可能領域
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
%title( 'Phase', 'FontSize', 16 );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40);
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_cf(:,1), tmp_d4(:,26), 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
xlabel( 'Time [s]' ); 
ylabel( 'Target Movable Volume [m^3]' , 'FontSize', 32);
xlim([0 inf])
fig17 = strcat(figpath,'\', 'VoM');
saveas(figure(n_figure),strcat(fig17,'.fig'))
png17 = strcat(pngpath,'\', 'VoM');
saveas(figure(n_figure),strcat(png17,'.png'))
n_figure = n_figure + 1;
% %
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% %title( 'LinearMomentum_of_Robotbase', 'FontSize', 16 );
% set( gca, 'FontName', 'Times New Roman', 'FontSize', 25 );
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_en(:,1), tmp_en(:,26) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_en(:,1), tmp_en(:,27) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_en(:,1), tmp_en(:,28) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% legend({'x', 'y', 'z'},'Location','northwest'  );
% xlabel( 'Time [s]' ); 
% ylabel( 'Base Linear Momentum [Ns]' );
% xlim([0 7])
% fig19 = strcat(figpath,'\',timepath,'_19_LinearMomentum_of_Robotbase');
% saveas(figure(n_figure),strcat(fig19,'.fig'))
% png19 = strcat(pngpath,'\',timepath,'_19_LinearMomentum_of_Robotbase');
% saveas(figure(n_figure),strcat(png19,'.png'))
% n_figure = n_figure + 1;
% 
% figure(n_figure)
% hold on;
% box on;
% grid on;
% grid minor;
% %title( 'AngualarMomentum_of_Robotbase', 'FontSize', 16 );
% set( gca, 'FontName', 'Times New Roman', 'FontSize', 25 );
% set( gcf, 'Position', position, 'Color', 'white' );
% plot( tmp_en(:,1), tmp_en(:,29) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_en(:,1), tmp_en(:,30) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% plot( tmp_en(:,1), tmp_en(:,31) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
% legend( {'x', 'y', 'z'},'Location','northwest'  );
% xlabel( 'Time [s]' ); 
% ylabel( 'Base Angular Momentum [Nms]' );
% xlim([0 7])
% fig18 = strcat(figpath,'\',timepath,'_18_AngularMomentum_of_Robotbase');
% saveas(figure(n_figure),strcat(fig18,'.fig'))
% png18 = strcat(pngpath,'\',timepath,'_18_AngularMomentum_of_Robotbase');
% saveas(figure(n_figure),strcat(png18,'.png'))
% n_figure = n_figure + 1;



%%%%%%%%%%%%%%%%%%%%%%%%%%%↓↓なんかたまにバグるから一旦カット↓↓%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Base Position [m]
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
%title( 'Angualar Momentum of Target', 'FontSize', 16 );
set( gca, 'FontName', 'Times New Roman' , 'FontSize', 40);
set( gcf, 'Position', position, 'Color', 'white' );
%tmp_thesis　⇒　time,　SV_d.R0,　dc2rpy(SV_d.A0),　SV_d.v0,　SV_d.w0,　SV_ts.R0,　dc2rpy(SV_ts.A0),　SV_ts.v0,　SV_ts.F0,　SV_ts.T0);
plot( tmp_en(:,1), tmp_thesis(:,2) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1) 
plot( tmp_en(:,1), tmp_thesis(:,3) , 'r-', 'linewidth', sen );   % SV_d.R0(2,1)
plot( tmp_en(:,1), tmp_thesis(:,4) , 'b-', 'linewidth', sen );   % SV_d.R0(3,1)
% legend( 'x', 'y', 'z'  );
legend( {'x', 'y', 'z'},'Location','northwest'  );
xlabel( 'Time [s]' ); 
ylabel( 'Base Position [m]' );
xlim([0 7])
fig21 = strcat(figpath,'\', 'ロボット位置');
saveas(figure(n_figure),strcat(fig21,'.fig'))
png21 = strcat(pngpath,'\', 'ロボット位置');
saveas(figure(n_figure),strcat(png21,'.png'))
n_figure = n_figure + 1;


% Base Orientation [rad]
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
%title( 'Angualar Momentum of Target', 'FontSize', 16 );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40 );
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_thesis(:,5) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,6) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,7) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','southwest'  );
xlabel( 'Time [s]' ); 
ylabel( 'Base Orientation [rad]' );
xlim([0 7])
fig22 = strcat(figpath,'\', 'ロボット姿勢');
saveas(figure(n_figure),strcat(fig22,'.fig'))
png22 = strcat(pngpath,'\', 'ロボット姿勢');
saveas(figure(n_figure),strcat(png22,'.png'))
n_figure = n_figure + 1;

% 'Base Velocity
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
%title( 'Angualar Momentum of Target', 'FontSize', 16 );
set( gca, 'FontName', 'Times New Roman' , 'FontSize', 40);
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_thesis(:,8) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,9) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,10) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','southwest'  );
xlabel( 'Time [s]' ); 
ylabel( 'Base Velocity [m/s]' );
xlim([0 7])
fig23 = strcat(figpath,'\', 'ロボット速度');
saveas(figure(n_figure),strcat(fig23,'.fig'))
png23 = strcat(pngpath,'\', 'ロボット速度');
saveas(figure(n_figure),strcat(png23,'.png'))
n_figure = n_figure + 1;


% Base Angular Velocity
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
%title( 'Angualar Momentum of Target', 'FontSize', 16 );
set( gca, 'FontName', 'Times New Roman' , 'FontSize', 40);
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_thesis(:,11) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,12) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,13) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'  );
xlabel( 'Time [s]' ); 
ylabel( 'Base Angular Velocity [rad/s]', 'FontSize', 32);
xlim([0 7])
fig24 = strcat(figpath,'\', 'ロボット角速度');
saveas(figure(n_figure),strcat(fig24,'.fig'))
png24 = strcat(pngpath,'\', 'ロボット角速度');
saveas(figure(n_figure),strcat(png24,'.png'))
n_figure = n_figure + 1;


% Target Position [m]
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
%title( 'Angualar Momentum of Target', 'FontSize', 16 );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40 );
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_thesis(:,14) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,15) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,16) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'  );
xlabel( 'Time [s]' ); 
ylabel( 'Target Position [m]' );
xlim([0 7])
fig25 = strcat(figpath,'\', 'ターゲット位置');
saveas(figure(n_figure),strcat(fig25,'.fig'))
png25 = strcat(pngpath,'\', 'ターゲット位置');
saveas(figure(n_figure),strcat(png25,'.png'))
n_figure = n_figure + 1;


% Target Orinetation [rad]
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
%title( 'Angualar Momentum of Target', 'FontSize', 16 );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40 );
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_thesis(:,17) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,18) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,19) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( 'x', 'y', 'z'  );
xlabel( 'Time [s]' ); 
ylabel( 'Target Orinetation [rad]' );
xlim([0 7])
fig26 = strcat(figpath,'\', 'ターゲット姿勢');
saveas(figure(n_figure),strcat(fig26,'.fig'))
png26 = strcat(pngpath,'\', 'ターゲット姿勢');
saveas(figure(n_figure),strcat(png26,'.png'))
n_figure = n_figure + 1;


% Target Velocity [m/s]
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
%title( 'Angualar Momentum of Target', 'FontSize', 16 );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40 );
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_thesis(:,20) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,21) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,22) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'  );
xlabel( 'Time [s]' ); 
ylabel( 'Target Velocity [m/s]' );
xlim([0 7])
fig27 = strcat(figpath,'\', 'ターゲット速度');
saveas(figure(n_figure),strcat(fig27,'.fig'))
png27 = strcat(pngpath,'\', 'ターゲット速度');
saveas(figure(n_figure),strcat(png27,'.png'))
n_figure = n_figure + 1;

% ターゲット角速度
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
%title( 'angular velocity', 'FontSize', 16 );
set( gca, 'FontName', 'Times New Roman' , 'FontSize', 40);
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_movie(:,1), tmp_movie(:,26) * pi/180, 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_movie(:,1), tmp_movie(:,27)* pi/180, 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_movie(:,1), tmp_movie(:,28)* pi/180, 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'   );
xlabel( 'Time [s]' ); 
ylabel( 'Target Angular Velocity [rad/s]', 'FontSize', 32 );
xlim([0 7])
%ylim([0 0.05])
fig08 = strcat(figpath,'\', 'ターゲット角速度');
saveas(figure(n_figure),strcat(fig08,'.fig'))
png08 = strcat(pngpath,'\', 'ターゲット角速度');
saveas(figure(n_figure),strcat(png08,'.png'))
n_figure = n_figure + 1;


% 
% External Force [N]  　　　SV_ts.F0（ターゲットにはたらく外力）のグラフ
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
% title( 'Angualar Momentum of Target', 'FontSize', 40 );
set( gca, 'FontName', 'Times New Roman', 'FontSize', 40);
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_thesis(:,23) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,24) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,25) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'  );
xlabel( 'Time [s]' ); 
ylabel( 'External Force [N]' );
xlim([0 7])
fig28 = strcat(figpath,'\', 'ターゲット力');
saveas(figure(n_figure),strcat(fig28,'.fig'))
png28 = strcat(pngpath,'\', 'ターゲット力');
saveas(figure(n_figure),strcat(png28,'.png'))
n_figure = n_figure + 1;


%  External Torque          SV_ts.T0（ターゲットにはたらくトルク）のグラフ
figure(n_figure)
hold on;
box on;
grid on;
grid minor;
% title( 'Angualar Momentum of Target', 'FontSize', 40 );
set( gca, 'FontName', 'Times New Roman' , 'FontSize', 40);
set( gcf, 'Position', position, 'Color', 'white' );
plot( tmp_en(:,1), tmp_thesis(:,26) , 'k-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,27) , 'r-', 'linewidth', sen );   % time, SV_d.R0(1,1)
plot( tmp_en(:,1), tmp_thesis(:,28) , 'b-', 'linewidth', sen );   % time, SV_d.R0(1,1)
legend( {'x', 'y', 'z'},'Location','northwest'  );
xlabel( 'Time [s]' ); 
ylabel( 'External Torque [Nm]' );
xlim([0 7])
fig29 = strcat(figpath,'\', 'ターゲットトルク');
saveas(figure(n_figure),strcat(fig29,'.fig'))
png29 = strcat(pngpath,'\', 'ターゲットトルク');
saveas(figure(n_figure),strcat(png29,'.png'))

n_figure = n_figure + 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%↑↑なんかたまにバグるから一旦カット↑↑%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%}

%}   % グラフ作成終了


close all
%}   % 結果表示終了

%}

end% zetaシミュレーションループ終了
end
end
end
end

%%
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

%%% EOF
%%