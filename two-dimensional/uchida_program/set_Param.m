function Param = set_Param()

%%%%%%%%%%パラメータ設定%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 直方体ベース，左右対称の4つのリンクを持つ双腕ロボットを想定．
% ベースに近いリンクからa,b,c,dとし，リンクdは刺股型のエンドエフェクタを持つ．
% 左手リンクが[1,2,3,4], 右手リンクが[5,6,7,8]
% ロボットモデルのそのものを変更する場合はリンクパラメータ自体を変更する -> DualArmRobo_LP_v3.m
% パラメータ定義にParamのメンバーを利用している箇所は，基本的に変更しないが，構造を理解していれば変更しても良い
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% シミュレーション条件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Param.EndTime   = 1.5;%2;    %シミュレーション終了時間[s]
Param.MinusTime = -1;      %シミュレーション開始からロボット制御開始までの時間[s]
Param.DivTime   = 0.001;  %シミュレーション刻み時間[s]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% アニメーション条件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Param.MovDivTime = 0.01;  % アニメーション刻み時間



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ロボットリンクパラメータ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ロボットベース部分質量[kg]
% 変更時，慣性行列に注意
Param.BaseMass = 7.70;

%ロボットベース部分慣性行列[m^2kg]
Param.BaseInertia = [ 1e9     0   0;   % 0の部分がカップリング項(慣性乗積?)
                        0   1e9   0;   % 慣性乗積が0ということは、固定軸の周りを回転するということ
                        0     0   0.09783069148];

% ロボットリンク部分質量[kg]
% 変更時，慣性行列に注意
% ベースに近いリンクからa, b, c, dとし，左右対称モデルであるためリンク(1,5),(2,6)...は同じ質量．
Param.LaMass = 1.09;
Param.LbMass = 0.98;
Param.LcMass = 0.32;
Param.LdMass = 0.21;

% ロボットリンク部分慣性行列[m^2kg]
% ベースに近い方からa, b, c, d 左右対称モデル
Param.LaInertia = [1e9   0 0;
                     0 1e9 0;
                     0   0 0.00371];
Param.LbInertia = [1e9   0 0;
                     0 1e9 0;
                     0   0 0.00149];
Param.LcInertia = [1e9   0 0;
                     0 1e9 0;
                     0   0 0.000752];
Param.LdInertia = [1e9   0 0;
                     0 1e9 0;
                     0   0 0.000254];

% ロボット関節条件
Param.JointAngLim = repmat(pi-deg2rad(10), [8,1]);
Param.JointTrqLim = repmat(100, [8,1]);

% ロボットベース部分サイズ[m]
% animation用．ダイナミクス計算には直接関係ないが，初期のパラメータではベース重心から関節までの位置を定義するのに使用している．
% 変更時，Param.BaseCenter2Jに注意
% 二次元モデルではheight = 0
Param.BaseDepth  = 0.22;            % 縦（奥行き）
Param.BaseWidth  = 0.32;            % 横
Param.BaseHeight = 0;               % 高さ

% ロボットベース質量重心に対する幾何中心の相対位置
Param.BaseMCenter2GCenter = [0, 0, 0]';

% ロボットリンク長さ及びエンドエフェクタ形状パラメータ[m]
% ベースに近い方からa, b, c, d 左右対称モデル
% Ldについては単純長さではなく，関節から刺股状のエンドエフェクタ先端までの距離H, 成す角gamma[rad]エンドエフェクタ先端球直径Dであることに注意
Param.LaLength = 0.251;
Param.LbLength = 0.181;
Param.LcLength = 0.050;
Param.LdH      = 0.05;
Param.LdGamma  = deg2rad(36);
Param.LdD      = 0.02;

% ベース重心から，ベースに直接結合しているリンク関節までの距離．直接結合していないリンクについては0.
% ベース座標系における表現と考える（？）
% ダイナミクス計算に直接関与
% リンク2,3,4,6,7,8は[0,0,0]'
% リンク１，５がベースに直接結合．変更する場合はDualArm_FourTips_LP_v3.m , 42行目を参照．
Param.BaseCenter2J1 = [-Param.BaseWidth/2,  Param.BaseDepth/2, 0]';
Param.BaseCenter2J5 = [ Param.BaseWidth/2,  Param.BaseDepth/2, 0]';

% リンクi重心から関節jまでの距離をParam.Center2Joint(:,i,j)で表現する．座標系はリンク根元座標で表現 ;LP.cc
% 論文に記述がない可能性あり．-> DualArm_FourTips_LP_v3.m, 81行目参照
%hase_programでは，
%LP_d.cc(:,5,5) = [ 0 -0.24641 0 ]';
%LP_d.cc(:,5,6) = [ 0  0.00459 0 ]';
%LP_d.cc(:,6,6) = [ 0 -0.17641 0 ]';
%LP_d.cc(:,6,7) = [ 0  0.00459 0 ]';
%LP_d.cc(:,7,7) = [ 0 -0.03    0 ]';
%LP_d.cc(:,7,8) = [ 0  0.02    0 ]';
%LP_d.cc(:,8,8) = [ 0 -0.03    0 ]';
%ってなってるけど，間違いでは？全部逆向きでは？->逆にして確かめる．->逆じゃなかった！->yやん

Param.Center2Joint    = zeros(3,8,8);           % 初期化   ;LP.cc
Param.Center2JointEnd = zeros(3,8);             % 初期化   ;LP.ce

% 左手
Param.Center2Joint(:,1,1) = [ 0 -0.24641 0 ]'; % リンク1の重心から根本側の関節(J1)への位置ベクトル
Param.Center2Joint(:,2,2) = [ 0 -0.17641 0 ]'; % リンク2の重心から根本側の関節(J2)への位置ベクトル
Param.Center2Joint(:,3,3) = [ 0 -0.03    0 ]'; % リンク3の重心から根本側の関節(J3)への位置ベクトル
Param.Center2Joint(:,4,4) = [ 0 -0.03    0 ]'; % リンク4の重心から根本側の関節(J4)への位置ベクトル

% Param.Center2Joint(:,1,2) = [ 0  0.00459 0 ]'; % リンク1の重心から先端側の関節(J2)への位置ベクトル
% Param.Center2Joint(:,2,3) = [ 0  0.00459 0 ]'; % リンク2の重心から先端側の関節(J3)への位置ベクトル
% Param.Center2Joint(:,3,4) = [ 0  0.02    0 ]'; % リンク3の重心から先端側の関節(J4)への位置ベクトル

% 右手
% Param.Center2Joint(:,5,5) = [ 0 -0.24641 0 ]'; % リンク5の重心から根本側の関節(J5)への位置ベクトル
% Param.Center2Joint(:,6,6) = [ 0 -0.17641 0 ]'; % リンク6の重心から根本側の関節(J6)への位置ベクトル
% Param.Center2Joint(:,7,7) = [ 0 -0.03    0 ]'; % リンク7の重心から根本側の関節(J7)への位置ベクトル
% Param.Center2Joint(:,8,8) = [ 0 -0.03    0 ]'; % リンク8の重心から根本側の関節(J8)への位置ベクトル
Param.Center2Joint(:,[37,46,55,64]) = Param.Center2Joint(:,[1,10,19,28]); % 左右対称なので省略

%%%%%相対定義%%%%%
% SV初期定義時のリンク向きが，真上から変更された場合，ここを書き直す．それ以外は変更しない．
% 左手リンクの重心から先端側の関節への位置ベクトル，リンク長さによって相対的に決定
Param.Center2Joint(:,[ 9,18,27]) = Param.Center2Joint(:,[ 1,10,19]) + [[0,0,0];Param.LaLength, Param.LbLength, Param.LcLength;[0,0,0]];
% リンク4重心から左手先端（２つの先端球の中点）への位置ベクトル
Param.Center2JointEnd(:,4)       = Param.Center2Joint(:,4,4) + [0;Param.LdH*cos(Param.LdGamma);0];
% 右手リンクの重心から先端側の関節への位置ベクトル，リンク長さによって相対的に決定
Param.Center2Joint(:,[45,54,63]) = Param.Center2Joint(:,[37,46,55]) + [[0,0,0];Param.LaLength, Param.LbLength, Param.LcLength;[0,0,0]];
% リンク8重心から右手先端（２つの先端球の中点）への位置ベクトル
Param.Center2JointEnd(:,8)       = Param.Center2Joint(:,8,8) + [0;Param.LdH*cos(Param.LdGamma);0];
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ロボット初期状態設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ベースの初期位置・姿勢・速度・角速度
Param.BasePosition0     = [ 0 0 0]';             % 初期位置          ;SV.R0
Param.BaseOrientation0  = [ 0 0 deg2rad( 0 ) ]'; % 初期姿勢  ラジアン ;SV.Q0
Param.BaseVelocity0     = [ 0 0 0 ]';            % 初期並進速度 ;SV.v0
Param.BaseAngVel0       = [ 0 0 0 ]';            % 初期角速度 ;SV.w0

% 初期関節角度：反時計回り正
%[pi/3 -pi*4/9 -pi*7/18 0]'
Param.LinkAngLeft  =  [pi/3 -pi*4/9 -pi*7/18 0]';  % 左手の関節角度，ベース側から ;SV.q
Param.LinkAngRight = -[pi/3 -pi*4/9 -pi*7/18 0]';  % 右手の関節角度，ベース側から ;SV.q
% Param.LinkAngLeft  =  [0 0 0 0]';  % 左手の関節角度，ベース側から ;SV.q
% Param.LinkAngRight = -[0 0 0 0]';  % 右手の関節角度，ベース側から ;SV.q


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ターゲットリンクパラメータ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ターゲット質量[kg]
% 変更時，慣性行列に注意
Param.TargetMass = 3.70;

% ロボットベース部分慣性行列[m^2kg]
Param.TargetInertia = [ 1e9     0   0;   
                          0   1e9   0;   
                          0     0   0.0172];

% ターゲットサイズ[m]
% 二次元モデルではheight = 0
Param.TargetDepth  = 0.16;
Param.TargetWidth  = 0.16;
Param.TargetHeight = 0;

% ターゲット幾何中心に対する質量重心の相対位置
Param.TargetMCenter2GCenter = [0, 0, 0]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ターゲット初期状態設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ターゲットの初期位置・姿勢・速度・角速度
Param.TargetPosition0     = [ 0 0.4 0]';             % 初期位置          ;SV.R0
Param.TargetOrientation0  = [ 0 0 deg2rad( 0 ) ]';   % 初期姿勢  ラジアン ;SV.Q0
Param.TargetVelocity0     = [ 0 0 0 ]';              % 初期並進速度 ;SV.v0
Param.TargetAngVel0       = [ 0 0 -1 ]';              % 初期角速度 ;SV.w0


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 物理係数設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Param.ContactDamp  = 8;         % 接触力減衰係数
Param.ContactElast = 9000;      % 接触力弾性係数
Param.ContactNu    = 0.3;       % 接触力摩擦係数
Param.WristDamp    = 0.3;       % 手首関節減衰係数
Param.WristElast   = 0.1;       % 手首関節弾性係数

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ファイル設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Param.DateSavePath = '/Users/akiyoshi/develop/srl/github/MATLAB_space_debri_capturing_sim/two-dimensional/uchida_program/dat';
Param.FileName     = [num2str(Param.ContactDamp),'_' ,num2str(Param.ContactElast)]; 
Param.Delimiter    = '\t';
Param.StringType   = '%10s';
Param.DataType     = '%10f';
%EOF
end