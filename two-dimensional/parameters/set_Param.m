function param = set_Param()

% 左手リンクが[1,2,3,4], 右手リンクが[5,6,7,8]

% シミュレーション条件 & パス設定
param.generalParam = generalParam();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ロボットリンクパラメータ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ロボットベース部分質量[kg]
% 変更時，慣性行列に注意
param.BaseMass = 10;%7.70;

%ロボットベース部分慣性行列[m^2kg]
param.BaseInertia = [ 1e9     0   0;   % 0の部分がカップリング項(慣性乗積?)
                        0   1e9   0;   % 慣性乗積が0ということは、固定軸の周りを回転するということ
                        0     0   .5];%0.09783069148];

% ロボットリンク部分質量[kg]
% 変更時，慣性行列に注意
% ベースに近いリンクからa, b, c, dとし，左右対称モデルであるためリンク(1,5),(2,6)...は同じ質量．
param.LaMass = 1.09;
param.LbMass = 0.98;
param.LcMass = 0.32;
param.LdMass = 0.21;

% ロボットリンク部分慣性行列[m^2kg]
% ベースに近い方からa, b, c, d 左右対称モデル
param.LaInertia = [1e9   0 0;
                     0 1e9 0;
                     0   0 0.00371];
param.LbInertia = [1e9   0 0;
                     0 1e9 0;
                     0   0 0.00149];
param.LcInertia = [1e9   0 0;
                     0 1e9 0;
                     0   0 0.000752];
param.LdInertia = [1e9   0 0;
                     0 1e9 0;
                     0   0 0.000254];

% ロボット関節条件
param.JointAngLim = repmat(pi-deg2rad(10), [8,1]);
param.JointTrqLim = repmat(10, [8,1]);

% ロボットベース部分サイズ[m]
% animation用．ダイナミクス計算には直接関係ないが，初期のパラメータではベース重心から関節までの位置を定義するのに使用している．
% 変更時，Param.BaseCenter2Jに注意
% 二次元モデルではheight = 0
param.BaseDepth  = .16;%0.22;            % 縦（奥行き）
param.BaseWidth  = .15757*2;%.314;%0.32;            % 横
param.BaseHeight = 0;               % 高さ

% ロボットベース質量重心に対する幾何中心の相対位置
param.BaseMCenter2GCenter = [0, 0, 0]';

% ロボットリンク長さ及びエンドエフェクタ形状パラメータ[m]
% ベースに近い方からa, b, c, d 左右対称モデル
% Ldについては単純長さではなく，関節から刺股状のエンドエフェクタ先端までの距離H, 成す角gamma[rad]エンドエフェクタ先端球直径Dであることに注意
param.LaLength = .25;                                               %.235;%0.251;
param.LbLength = .175;                                              %.16;%0.181;
param.LcLength = .091;                                              %.075;%0.050;
param.LdH      = 0.081393154503312;                                 %0.05;
param.LdGamma  = 0.717235030908703;                                 %deg2rad(36);
param.LdD      = 0.018;

% ベース重心から，ベースに直接結合しているリンク関節までの距離．直接結合していないリンクについては0.
% ベース座標系における表現と考える（？）
% ダイナミクス計算に直接関与
% リンク2,3,4,6,7,8は[0,0,0]'
% リンク１，５がベースに直接結合．変更する場合はDualArm_FourTips_LP_v3.m , 42行目を参照．
param.BaseCenter2J1 = [-param.BaseWidth/2,  param.BaseDepth/2, 0]';
param.BaseCenter2J5 = [ param.BaseWidth/2,  param.BaseDepth/2, 0]';

% リンクi重心から関節jまでの距離をParam.Center2Joint(:,i,j)で表現する．座標系はリンク根元座標で表現 ;LP.cc
% 論文に記述がない可能性あり．-> DualArm_FourTips_LP_v3.m, 81行目参照

param.Center2Joint    = zeros(3,8,8);           % 初期化   ;LP.cc
param.Center2JointEnd = zeros(3,8);             % 初期化   ;LP.ce

% 左手
param.Center2Joint(:,1,1) = [ 0 -0.24641 0 ]'; % リンク1の重心から根本側の関節(J1)への位置ベクトル
param.Center2Joint(:,2,2) = [ 0 -0.17641 0 ]'; % リンク2の重心から根本側の関節(J2)への位置ベクトル
param.Center2Joint(:,3,3) = [ 0 -0.03    0 ]'; % リンク3の重心から根本側の関節(J3)への位置ベクトル
param.Center2Joint(:,4,4) = [ 0 -0.03    0 ]'; % リンク4の重心から根本側の関節(J4)への位置ベクトル

% Param.Center2Joint(:,1,2) = [ 0  0.00459 0 ]'; % リンク1の重心から先端側の関節(J2)への位置ベクトル
% Param.Center2Joint(:,2,3) = [ 0  0.00459 0 ]'; % リンク2の重心から先端側の関節(J3)への位置ベクトル
% Param.Center2Joint(:,3,4) = [ 0  0.02    0 ]'; % リンク3の重心から先端側の関節(J4)への位置ベクトル

% 右手
% Param.Center2Joint(:,5,5) = [ 0 -0.24641 0 ]'; % リンク5の重心から根本側の関節(J5)への位置ベクトル
% Param.Center2Joint(:,6,6) = [ 0 -0.17641 0 ]'; % リンク6の重心から根本側の関節(J6)への位置ベクトル
% Param.Center2Joint(:,7,7) = [ 0 -0.03    0 ]'; % リンク7の重心から根本側の関節(J7)への位置ベクトル
% Param.Center2Joint(:,8,8) = [ 0 -0.03    0 ]'; % リンク8の重心から根本側の関節(J8)への位置ベクトル
param.Center2Joint(:,[37,46,55,64]) = param.Center2Joint(:,[1,10,19,28]); % 左右対称なので省略

%%%%%相対定義%%%%%
% SV初期定義時のリンク向きが，真上から変更された場合，ここを書き直す．それ以外は変更しない．
% 左手リンクの重心から先端側の関節への位置ベクトル，リンク長さによって相対的に決定
param.Center2Joint(:,[ 9,18,27]) = param.Center2Joint(:,[ 1,10,19]) + [[0,0,0];param.LaLength, param.LbLength, param.LcLength;[0,0,0]];
% リンク4重心から左手先端（２つの先端球の中点）への位置ベクトル
param.Center2JointEnd(:,4)       = param.Center2Joint(:,4,4) + [0;param.LdH*cos(param.LdGamma);0];
% 右手リンクの重心から先端側の関節への位置ベクトル，リンク長さによって相対的に決定
param.Center2Joint(:,[45,54,63]) = param.Center2Joint(:,[37,46,55]) + [[0,0,0];param.LaLength, param.LbLength, param.LcLength;[0,0,0]];
% リンク8重心から右手先端（２つの先端球の中点）への位置ベクトル
param.Center2JointEnd(:,8)       = param.Center2Joint(:,8,8) + [0;param.LdH*cos(param.LdGamma);0];
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ロボット初期状態設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ベースの初期位置・姿勢・速度・角速度
param.BasePosition0     = [ 0 0 0]';             % 初期位置          ;SV.R0
param.BaseOrientation0  = [ 0 0 deg2rad( 0 ) ]'; % 初期姿勢  ラジアン ;SV.Q0
param.BaseVelocity0     = [ 0 0 0 ]';            % 初期並進速度 ;SV.v0
param.BaseAngVel0       = [ 0 0 0 ]';            % 初期角速度 ;SV.w0

% 初期関節角度：反時計回り正
%[pi/3 -pi*4/9 -pi*7/18 0]'
param.LinkAngLeft  =  [pi/3 -pi*4/9 -pi*7/18 0]';  % 左手の関節角度，ベース側から ;SV.q
param.LinkAngRight = -[pi/3 -pi*4/9 -pi*7/18 0]';  % 右手の関節角度，ベース側から ;SV.q
% Param.LinkAngLeft  =  [0 0 0 0]';  % 左手の関節角度，ベース側から ;SV.q
% Param.LinkAngRight = -[0 0 0 0]';  % 右手の関節角度，ベース側から ;SV.q


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ターゲットリンクパラメータ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ターゲットサイズ[m]
% 二次元モデルではheight = 0
param.TargetDepth  = 0.14;
param.TargetWidth  = 0.14;%0.16;
param.TargetHeight = 0;

% ターゲット幾何中心に対する質量重心の相対位置
param.TargetMCenter2GCenter = [0, 0, 0]';

% ターゲット質量[kg]
% 変更時，慣性行列に注意
ro = 150;%150; % kg/m^2
param.TargetMass = ro * param.TargetDepth * param.TargetWidth; %3.70;
% ターゲット部分慣性行列[m^2kg]
param.TargetInertia = [ 1e9     0   0;   
                          0   1e9   0;   
                          0     0   param.TargetMass*param.TargetDepth^2/6];%0.0172];0.0164

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ターゲット初期状態設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ターゲットの初期位置・姿勢・速度・角速度
param.TargetPosition0     = [ 0 0.35 0]';             % 初期位置          ;SV.R0
param.TargetOrientation0  = [ 0 0 deg2rad( 0 ) ]';   % 初期姿勢  ラジアン ;SV.Q0
param.TargetVelocity0     = [ -.0 -.0 0 ]';              % 初期並進速度 ;SV.v0
param.TargetAngVel0       = [ 0 0 8]';              % 初期角速度 ;SV.w0


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 物理係数設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.ContactDamp  = 20;%8;%20;         % 接触力減衰係数
param.ContactElast = 1000;%9000;%1000;      % 接触力弾性係数
param.ContactNu    = .1;%0.3;       % 接触力摩擦係数
param.WristDamp    = .3;%0.4;       % 手首関節減衰係数
param.WristElast   = .4;%0.8;       % 手首関節弾性係数

% コントロールパラメータ
param.control = controlParam();
end