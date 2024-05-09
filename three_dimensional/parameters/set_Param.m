function param = set_Param()

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
param.EndTime   = 30;%2;    %シミュレーション終了時間[s]
param.MinusTime = 0;      %シミュレーション開始からロボット制御開始までの時間[s]
param.DivTime   = 0.001;  %シミュレーション刻み時間[s]
param.general = generalParam();


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% アニメーション条件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.MovDivTime = 0.01;  % アニメーション刻み時間



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ロボットリンクパラメータ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.robot = robotParam();

% ロボット関節条件
param.JointAngLim = repmat(pi-deg2rad(10), [12,1]);
param.JointTrqLim = repmat(10, [12,1]);


param.LdH      = 0.081393154503312;%0.05;
param.LdGamma  = 0.717235030908703;%deg2rad(36);
param.LdD      = 0.018;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ターゲットリンクパラメータ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ターゲットサイズ[m]
% 二次元モデルではheight = 0
side = 0.16;
param.TargetDepth  = side;
param.TargetWidth  = side;%0.16;
param.TargetHeight = side*2;

% ターゲット幾何中心に対する質量重心の相対位置
param.TargetMCenter2GCenter = [0, 0, .0]';

% ターゲット質量[kg]
% 変更時，慣性行列に注意
ro = 500; % kg/m^3
param.TargetMass = ro * param.TargetDepth * param.TargetWidth * param.TargetHeight; %3.70;
% ターゲット部分慣性行列[m^2kg]
param.TargetInertia = [ param.TargetMass*(param.TargetDepth^2 + param.TargetHeight^2)/12     0   0;   
                          0   param.TargetMass*(param.TargetWidth^2 + param.TargetHeight^2)/12   0;   
                          0     0   param.TargetMass*(param.TargetDepth^2 + param.TargetWidth^2)/12];%0.0172];0.0164

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ターゲット初期状態設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ターゲットの初期位置・姿勢・速度・角速度
param.TargetPosition0     = [ 0 0.35 0]';             % 初期位置          ;SV.R0
param.TargetOrientation0  = [ 0 0 0 ]';   % 初期姿勢  ラジアン ;SV.Q0
param.TargetVelocity0     = [ -.0 -.0 0 ]';              % 初期並進速度 ;SV.v0
param.TargetAngVel0       = [ 1 0 5]';              % 初期角速度 ;SV.w0


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 物理係数設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.contact = contactParam();
param.WristDamp    = .3;%0.4;       % 手首関節減衰係数
param.WristElast   = .4;%0.8;       % 手首関節弾性係数

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% コントロールパラメータ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.control = controlParam();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ファイル設定
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.DataSavePath = '\Users\akiyo\github\MATLAB_space_debri_capturing_sim\three_dimensional\dat';
param.FileName     = ['testRk'];
% param.FileName     = [num2str(param.control.mi'),'_' ,num2str(param.control.di'),'_' ,num2str(param.control.ki')];
end