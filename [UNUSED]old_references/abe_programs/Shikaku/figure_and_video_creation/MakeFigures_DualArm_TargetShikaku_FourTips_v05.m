%%%%%%%%%% グラフ作成専用 for 3リンク双腕宇宙ロボットの接触シミュレーション(手先速度制御＋ベースフリー) ターゲット四角型
%%%%%%%%%% 
%%%%%%%%%% DualArm_TargetShikaku_Tracking_Contact_MakeFigures  2018年1月24日
clc
clear
close all

%%%%%%%%%% フォルダパス設定 %%%%%%%%%
% % 今日の場合は↓
day = datestr( now, 'yyyy-mmdd' );
% % 今日以外の日時を指定する場合は↓
day = '2019-0118';

path = 'dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';

% '2019-0117-152541_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
% '2019-0124-115720_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
% '2019-0117-200253_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
% '2019-0117-223316_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
% '2019-0118-003857_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
% '2019-0126-032234_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.5,ErTH=225';
% '2019-0131-235523_dt=0.001,et=5,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
% '2019-0201-014208_dt=0.001,et=15,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
% '2019-0201-015634_dt=0.001,et=100,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';


'2019-0117-152541_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
'2019-0124-115720_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
'2019-0117-200253_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
'2019-0117-223316_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
'2019-0118-003857_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.25,ErTH=45';
'2019-0126-032234_dt=0.001,et=20,Xts=0,Yts=0,Vxts=0,Vyts=0,THts=0,Wts=1,ErL=0.5,ErTH=225';


% 時間指定
timepath = [ day, '-', '003857' ] ;
% パス指定
datepath = [ 'C:/Users/falsi/Desktop/Matlab/BackUp/', day, '/', timepath, '_' ];
datepath = [ 'C:\Users\falsi\Desktop\修士論文\hase_fig\Case_Simulation_Results\Sim5\', timepath, '_' ];
datpath = [ datepath, path, '/', timepath, '-', 'dat' ];
figpath1 = [ datepath, path, '/', timepath, '-', 'fig' ];
pngpath1 = [ datepath, path, '/', timepath, '-', 'png' ];


fig2file = [ datepath, path, '/', timepath, '-', 'fig2' ];
mkdir( fig2file );
png2file = [ datepath, path, '/', timepath, '-', 'png2' ];
mkdir( png2file );
figpath = [ datepath, path, '/', timepath, '-', 'fig2' ];
pngpath = [ datepath, path, '/', timepath, '-', 'png2' ];



epsfile = [ datepath, path, '/', timepath, '-', 'eps' ];
mkdir( epsfile );
epspath = [ datepath, path, '/', timepath, '-', 'eps' ];


n_figure = 1;


%%% 各種設定(figファイル作成後も編集可能) %%%

FSA = 40;   % メモリ・ラベルのフォントサイズ
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
n = 1000;

sen =  4;   % 線の太さ
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

%%


%%%%%%%%% オブジェクトクロージャ解析 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


mat_01 = dlmread( [ datpath, '/', timepath, '_', '01_Length_etc.dat' ], '\t', 1, 0 );
mat_09 = dlmread( [ datpath, '/', timepath, '_', '09_ContactFlag.dat' ], '\t', 1, 0 );
mat_15 = dlmread( [ datpath, '/', timepath, '_', '15_Robot_ForMovie.dat' ], '\t', 1, 0 ); 

timeD = mat_03(2:end,1);
r_tip = mat_01(1,1);
d = 2 * r_tip;
l_t = mat_01(1,2);
l = mat_01(1,16)*sin(mat_01(1,17))*2;
theta_t = mat_05(1:end,13);
L1 = mat_15(1:end,21:22);
L2 = mat_15(1:end,24:25);
R1 = mat_15(1:end,27:28);
R2 = mat_15(1:end,30:31);
fl1 = mat_09(1:end,2);
fl2 = mat_09(1:end,3);
fl3 = mat_09(1:end,4);
fl4 = mat_09(1:end,5);


theta_EE = zeros(length(timeD),1);
D_max = zeros(length(timeD),1);
D_min = zeros(length(timeD),1);
D_L1R2 = zeros(length(timeD),1);
D_L2R1 = zeros(length(timeD),1);
D_L1R1 = zeros(length(timeD),1);
D_L2R2 = zeros(length(timeD),1);
R = zeros(length(timeD),1);
Rs = zeros(length(timeD),1);
lt = zeros(length(timeD),1);
Maxa = zeros(length(timeD),1);
Maxb = zeros(length(timeD),1);
Mina = zeros(length(timeD),1);
Minb = zeros(length(timeD),1);
Minna = zeros(length(timeD),1);
Minnb = zeros(length(timeD),1);

ll = zeros(length(timeD),1);
f1 = zeros(length(timeD),1);
f2 = zeros(length(timeD),1);
f3 = zeros(length(timeD),1);
f4 = zeros(length(timeD),1);
f5 = zeros(length(timeD),1);
f6 = zeros(length(timeD),1);
f7 = zeros(length(timeD),1);
fL = zeros(length(timeD),1);
fR = zeros(length(timeD),1);
fLR = zeros(length(timeD),1);

D_min0 = sqrt(2) * ( d + l_t ) - l;


for i = 1:length(timeD)
    i
R(i,1) = norm(L1(i,:)'-L2(i,:)');
D_L1R2(i,1) = norm(L1(i,:)'-R2(i,:)');
D_L2R1(i,1) = norm(L2(i,:)'-R1(i,:)');
D_L1R1(i,1) = norm(L1(i,:)'-R1(i,:)');
D_L2R2(i,1) = norm(L2(i,:)'-R2(i,:)');
EE_vec1 = ([ L1(i,:), 0 ]'-[ L2(i,:) 0 ]');
EE_vec2 = rpy2dc( 0, 0, -pi/2 )' * EE_vec1;
f1(i,1) = rad2deg(atan2( EE_vec1(2,1), EE_vec1(1,1) ));
f2(i,1) = rad2deg(atan2( EE_vec2(2,1), EE_vec2(1,1) ));
fL(i,1) = rad2deg(atan2( ( L1(i,2) - L2(i,2)  ), ( L1(i,1) - L2(i,1) ) ));
fR(i,1) = rad2deg(atan2( ( R2(i,2) - R1(i,2)  ), ( R2(i,1) - R1(i,1) ) ));
theta_EE(i,1) = atan2( EE_vec2(2,1), EE_vec2(1,1) );
theta_EE2 = atan2( EE_vec1(2,1), EE_vec1(1,1) ) - pi/2;

% f3(i,1) = rad2deg(theta_t(i,1));
% if ( theta_t(i,1) > 0 && theta_t(i,1) <= pi/4 )  ||  ( theta_t(i,1) > pi/2 && theta_t(i,1) <= 3*pi/4 ) || ( theta_t(i,1) > -pi/2 && theta_t(i,1) <= -pi/4 ) || ( theta_t(i,1) > -pi && theta_t(i,1) <= -3*pi/4 )
% elseif  theta_t(i,1) > pi/4 && theta_t(i,1) <= pi/2 
% theta_t(i,1)=pi/2-theta_t(i,1);
% elseif  theta_t(i,1) > -pi/4 && theta_t(i,1) <= 0
% theta_t(i,1)=pi/2+theta_t(i,1);
% end
% f4(i,1) = rad2deg(theta_t(i,1));
% f5(i,1) = f4(i,1) - f2(i,1);
% f6(i,1) = f3(i,1) - f2(i,1);


% phi = theta_t(i,1) - theta_EE(i,1);
% phi = theta_t(i,1);
% phi = deg2rad(f6(i,1));
phi = theta_t(i,1) - theta_EE2 - pi/4;

% if ( phi > 0 && phi <= pi/4 )||( phi > pi/2 && phi <= 3*pi/4 )||( phi > -pi/2 && phi <= -pi/4 )||( phi > -pi && phi <= -3*pi/4 )
% elseif  phi > pi/4 && phi <= pi/2 
% phi=pi/2-phi;
% elseif  phi > -pi/4 && phi <= 0
% phi=pi/2+phi;
% end

f7(i,1) = timeD(i,1);
f7(i,2) = theta_t(i,1) - theta_EE2;
f7(i,3) = phi;
f7(i,4) = cos(phi);


% if timeD(i,1)>0
% D_max(i,1) = ( d + l_t )/(sin(phi)) - l/(tan(phi));
D_max(i,1) = ( d + l_t ) * sqrt(2) * cos(phi);
% else
% D_max(i,1) = ( D_L1R2(i,1) + D_L2R1(i,1) ) / 2;
% else
% D_max(i,1) = 0;
% end

% if abs( D_max(i,1) )>0.2 || abs( D_max(i,1) )<D_min0
% D_max(i,1) = 0;
% end

D_min(i,1) = D_min0;
ll(i,1) = l;
Rs(i,1) = sqrt(2) * l_t;%ターゲット対角線
lt(i,1) = l_t;%ターゲット一辺

Maxa(i,1) = sqrt(2) * ( d + l_t ) * cos( phi ); abs( cos( phi ) );
Maxb(i,1) = sqrt( Maxa(i,1)^2 + l^2 );

Mina(i,1) = ( d + l_t )/( sin( phi ) ) - l / ( tan( phi ) );
Minna(i,1) = ( d + l_t )*sqrt(2) - l;
Minb(i,1) = sqrt( Mina(i,1)^2 + ll(i,1)^2 );
Minnb(i,1) = sqrt( Minna(i,1)^2 + l^2 );


%    fLR(i,1) = 80;
% if abs(fL(i,1)-fR(i,1))*2/abs(fL(i,1)+fR(i,1)) < 1e-3 &&  i>1
%    fLR(i,1) = 90;
% else
%     Maxa(i) = 0;
%     Maxb(i) = 0;
%     Mina(i) = 0;
%     Minb(i) = 0;
% end
if timeD(i,1)<=0
    Maxa(i) = 0;
    Maxb(i) = 0;
    Mina(i) = 0;
    Minb(i) = 0;
end

end



% L1R2
figure( 800 )
hold on; box on; grid on; %grid minor;
set( gcf, 'Position', Position, 'Color', 'white' );set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
plot( timeD(:,1), Maxa(:,1), 'm--', 'linewidth', 4 );
% plot( timeD(:,1), Mina(:,1), 'y--', 'linewidth', 4 );
plot( timeD(:,1), Minna(:,1), 'c--', 'linewidth', 4 );
plot( timeD(:,1), D_L1R2(:,1), 'r-', 'linewidth', 4 );
plot( timeD(:,1), D_L2R1(:,1), 'b-', 'linewidth', 4 );
legend( 'Max', 'Min', 'L1R2', 'L2R1' );
xlabel( 'Time [s]' );
ylabel( 'Distance [m]' );
% xlim([ 0, 5 ]);
xlim([ timeD(1,1), timeD(end,1) ]);
ylim([ 0.15, 0.40 ]);
fig0800 = [ figpath, '/', timepath, '_001' ];
png0800 = [ pngpath, '/', timepath, '_001' ];
saveas( figure(800), [ fig0800, '.fig' ] );
saveas( figure(800), [ png0800, '.png' ] );
n_figure = n_figure + 1;

% % L1R1
% figure( 801 )
% hold on; box on; grid on; %grid minor;
% set( gcf, 'Position', Position, 'Color', 'white' );set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
% plot( timeD(:,1), Maxb(:,1), 'm--', 'linewidth', 4 );
% plot( timeD(:,1), Minb(:,1), 'c--', 'linewidth', 4 );
% plot( timeD(:,1), Minnb(:,1), 'g--', 'linewidth', 4 );
% plot( timeD(:,1), D_L1R1(:,1), 'r-', 'linewidth', 4 );
% plot( timeD(:,1), D_L2R2(:,1), 'b-', 'linewidth', 4 );
% legend( 'Max', 'Min', 'L1R1', 'L2R2' );
% xlabel( 'Time [s]' );
% ylabel( 'Distance [m]' );
% % xlim([ 0, 5 ]);
% xlim([ timeD(1,1), timeD(end,1) ]);
% ylim([ 0.15, 0.42 ]);
% fig0801 = [ figpath, '/', timepath, '_801' ];
% png0801 = [ pngpath, '/', timepath, '_801' ];
% saveas( figure(801), [ fig0801, '.fig' ] );
% saveas( figure(801), [ png0801, '.png' ] );
% n_figure = n_figure + 1;

% % rotation
% figure( 802 )
% hold on; box on; grid on; %grid minor;
% set( gcf, 'Position', Position, 'Color', 'white' );set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
% plot( timeD(:,1), fL(:,1), 'r-', 'linewidth', 4 );
% plot( timeD(:,1), fR(:,1), 'b-', 'linewidth', 4 );
% plot( timeD(:,1), fLR(:,1), 'g-', 'linewidth', 4 );
% legend( 'L2L1', 'R1R2', 'hantei');
% xlabel( 'Time [s]' );
% ylabel( 'Distance [m]' );
% xlim([ timeD(1,1), timeD(end,1) ]);
% % xlim([ timeD(1,1), timeD(end,1) ]);
% % ylim([ 0.15, 0.42 ]);
% fig0802 = [ figpath, '/', timepath, '_802' ];
% png0802 = [ pngpath, '/', timepath, '_802' ];
% saveas( figure(802), [ fig0802, '.fig' ] );
% saveas( figure(802), [ png0802, '.png' ] );
% n_figure = n_figure + 1;


% figure( n_figure )
% hold on; box on; grid on; %grid minor;
% set( gcf, 'Position', Position, 'Color', 'white' );set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
% plot( timeD(:,1), f1(:,1), 'r-', 'linewidth', 4 );
% plot( timeD(:,1), f2(:,1), 'b-', 'linewidth', 4 );
% plot( timeD(:,1), f3(:,1), 'm-', 'linewidth', 4 );
% % plot( timeD(:,1), f4(:,1), 'c-', 'linewidth', 4 );
% % plot( timeD(:,1), f5(:,1), 'g-', 'linewidth', 4 );
% plot( timeD(:,1), f6(:,1), 'k-', 'linewidth', 4 );
% 
% % plot( timeD(:,1), D_min(:,1), 'k-', 'linewidth', 4 );
% 
% % plot( timeD(:,1), R(:,1), 'c-', 'linewidth', 4 );
% % plot( timeD(:,1), ll(:,1), 'm-', 'linewidth', 4 );
% % legend( 'f1', 'f2', 'f3', 'f4', 'f5', 'f6' );
% legend( 'f1', 'f2', 'f3', 'f6' );
% xlabel( 'Time [s]' );
% ylabel( 'Angle [deg]' );
% xlim([ timeD(1,1), timeD(end,1) ]);
% ylim([ -60, -30 ]);
% ylim([ -200, 200 ]);
% fig01 = [ figpath, '/', timepath, '_998' ];
% png01 = [ pngpath, '/', timepath, '_998' ];
% saveas( figure(n_figure), [ fig01, '.fig' ] );
% saveas( figure(n_figure), [ png01, '.png' ] );
% n_figure = n_figure + 1;


% figure( n_figure )
% hold on; box on; grid on; %grid minor;
% set( gcf, 'Position', Position, 'Color', 'white' );set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
% plot( timeD(:,1), D_max(:,1), 'g-', 'linewidth', 4 );
% plot( timeD(:,1), D_min(:,1), 'k-', 'linewidth', 4 );
% plot( timeD(:,1), D_L1R2(:,1), 'r-', 'linewidth', 4 );
% plot( timeD(:,1), D_L2R1(:,1), 'b-', 'linewidth', 4 );
% plot( timeD(:,1), Rs(:,1), 'y-', 'linewidth', 4 );%ターゲット対角線
% % legend( 'D\_max', 'D\_min', 'D\_L1R2', 'D\_L2R1', 'D\_L1L2', 'D\_R1R2', 'Rs', 'lt' );
% xlabel( 'Time [s]' );
% ylabel( 'Distance [m]' );
% xlim([ timeD(1,1), timeD(end,1) ]);
% ylim([ 0.15, 0.4 ]);
% fig01 = [ figpath, '/', timepath, '_999' ];
% png01 = [ pngpath, '/', timepath, '_999' ];
% saveas( figure(n_figure), [ fig01, '.fig' ] );
% saveas( figure(n_figure), [ png01, '.png' ] );
% n_figure = n_figure + 1;
% 
% figure( n_figure )
% hold on; box on; grid on; %grid minor;
% set( gcf, 'Position', Position, 'Color', 'white' );set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
% plot( timeD(:,1), D_L1R1(:,1), 'm-', 'linewidth', 4 );
% plot( timeD(:,1), D_L2R2(:,1), 'c-', 'linewidth', 4 );
% plot( timeD(:,1), Rs(:,1), 'y-', 'linewidth', 4 );%ターゲット対角線
% plot( timeD(:,1), lt(:,1), 'g-', 'linewidth', 4 );%ターゲット一辺
% % legend( 'D\_max', 'D\_min', 'D\_L1R2', 'D\_L2R1', 'D\_L1L2', 'D\_R1R2', 'Rs', 'lt' );
% xlabel( 'Time [s]' );
% ylabel( 'Distance [m]' );
% xlim([ timeD(1,1), timeD(end,1) ]);
% ylim([ 0.15, 0.4 ]);
% fig01 = [ figpath, '/', timepath, '_997' ];
% png01 = [ pngpath, '/', timepath, '_997' ];
% saveas( figure(n_figure), [ fig01, '.fig' ] );
% saveas( figure(n_figure), [ png01, '.png' ] );
% n_figure = n_figure + 1;


% a = 0:1/100:2*pi;
% 
% d = 2*3.14*1000;
% A = zeros(d,1);
% B = zeros(d,1);
% C = zeros(d,1);
% for i = 1:1:d
% A(i) = 1/sin(i/1000);
% B(i) = 1/tan(i/1000);
% C(i) = i/1000;
% end
% a = 0:1/1000:2*pi;
% 
% 
% figure( n_figure )
% hold on; box on; grid on; %grid minor;
% set( gcf, 'Position', Position, 'Color', 'white' );set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
% plot( C(:), A(:), 'r-', 'linewidth', 4 );
% plot( C(:), B(:), 'b-', 'linewidth', 4 );
% % legend( 'f1', 'f2' );
% xlabel( 'Time [s]' );
% ylabel( 'Angle [deg]' );
% % xlim([ timeD(1,1), timeD(end,1) ]);
% % ylim([ -60, -30 ]);
% % ylim([ -200, 200 ]);
% % fig01 = [ figpath, '/', timepath, '_998' ];
% % png01 = [ pngpath, '/', timepath, '_998' ];
% % saveas( figure(n_figure), [ fig01, '.fig' ] );
% % saveas( figure(n_figure), [ png01, '.png' ] );
% n_figure = n_figure + 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




figure( n_figure )
hold on; box on; grid on; %grid minor;
set( gcf, 'Position', Position, 'Color', 'white' );set( gca, 'FontName', 'Times New Roman', 'FontSize', FSA );
% plot( mat_13(2:end,1), mat_13(2:end,19), 'r-', 'linewidth', 4 );
% plot( mat_13(2:end,1), mat_13(2:end,13), 'b-', 'linewidth', 4 );
plot( mat_13(2:end,1), mat_13(2:end,7), 'r-', 'linewidth', 4 );
plot( mat_13(2:end,1), mat_13(2:end,13), 'b-', 'linewidth', 4 );
legend( 'Base', 'Target' );
xlabel( 'Time [s]' ); 
ylabel( 'Angular Momentum [Nms]' );
% xlim([ 0, 5 ]);
xlim([ mat_13(2,1), mat_13(end,1) ]);
% ylim([ -0.02, 0.02 ]);
fig01 = [ figpath, '/', timepath, '_000' ];
png01 = [ pngpath, '/', timepath, '_000' ];
saveas( figure(n_figure), [ fig01, '.fig' ] );
saveas( figure(n_figure), [ png01, '.png' ] );
n_figure = n_figure + 1;




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
             3;3;3; 3;3;3;   % 05 -- 18
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
             'SV\_ts.R0';'SV\_ts.v0';'SV\_ts.vd0';  'SV\_ts.Q0';'SV\_ts.w0';'SV\_ts.wd0'; % 05
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
              'Target Position [m]';'Target Velocity [m/s]';'Target Acceleration [m/s\^2]';  'Target Orientation [deg]';'Target Angular Velocity [deg/s]';'Target Angular Acceleration [deg/s\^2]';
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
              'ターゲット重心位置';'ターゲット重心速度';'ターゲット重心加速度';  'ターゲット姿勢';'ターゲット角速度';'ターゲット角加速度'; % 05
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
%%
j = 2;
for i = 2 : length( var_size )
[ i, var_name(i) ]
%%% 見たい図を制限するにはココを
if i < 92 % i > 85  && i < 93

%%% 図設定
 figure( i )
 hold on; box on; grid on; %grid minor;
%  title( var_name( i ), 'FontName', 'メイリオ', 'FontSize', FST );   % タイトルをvar_nameから取り出す
%  title( file_name( i ), 'FontName', 'メイリオ', 'FontSize', FST );   % タイトルをvar_nameから取り出す(日本語)
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

if i == 90  ||  i == 91  ||  i == 96  ||  i == 97
     ylim([ -0.01, 0.01 ]);
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
     xlim([ mat(1,1), 20 ]);   % 定義域
     xlim([ 0, 5 ]);   % 定義域


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
%  saveas( figure(i), [ epspath, '/', timepath, '_', num, '_', char( file_name( i ) ), '.eps' ] );

end

 j = j + var_size(i);    % 次の変数に進む
end

%}

%}   % グラフ作成終了
% close all