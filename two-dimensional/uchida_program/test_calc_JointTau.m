%%%%%%%%%%DualArmTestBed Simulation%%%%%%%%%%
%2023/01/01 Akiyoshi Uchida
%SpaceDyn_v2r0

clear all
close all

%パラメータ設定
Parameters = ParamSetting();    % 基本的にパラメータはParamSetting内で変更する．


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% global 変数の定義 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 全ての関数及びメインルーチン内で共通で使用される変数
global d_time
global Gravity
global Ez
Ez = [ 0 0 1 ]';
d_time = Parameters.DivTime; % シミュレーション1step当たりの時間
Gravity = [ 0 0 0 ]'; % 重力（地球重力は Gravity = [0 0 -9.8]）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 双腕ロボインスタンス作成
DualArmRobo_2  = DualArmRobo(Parameters);
% ターゲットインスタンス作成
TargetSquare_1 = TargetSquare(Parameters);

% ロボット・ターゲット力初期化
RoboJointTau   = zeros(6,1);                   % ロボ関節制御トルク，手首関節を除くことに注意 
RoboExtWrench  = zeros(6,3);                   % ロボ外力[ BaseTorque   LeftEdgeTorque  RightEdgeTorque ]
                                               % 　　　　[ BaseForce    LeftEdgeForce   RightEdgeForce  ]  
TargetExtWrench= zeros(6,1);                   % タゲ外力[ BaseTorque ]
                                               % 　　　　[ BaseForce  ] 

%シミュレーションループスタート
for time = 1:20
    % 接触力計算
    [RoboExtWrench(:, 2:3), TargetExtWrench] = calc_ContactForce(DualArmRobo_2, TargetSquare_1, Parameters.ContactElast, Parameters.ContactDamp);
    

    % 運動状態更新
    DualArmRobo_2  = DualArmRobo_2.update(RoboJointTau, RoboExtWrench, Parameters);    % methodを呼び出した後自身に代入することを忘れない！
end

% f1 = @() calc_gj_2arm(DualArmRobo_2.LP, DualArmRobo_2.SV, num_eL, num_eR);
% f2 = @() calc_gj_DualArmRobo(DualArmRobo_2);
% f3 = @() calc_gj(DualArmRobo_2.LP, DualArmRobo_2.SV, 1);
% 
% timeit(f1)
% timeit(f2)
% timeit(f3)

% calc_gj_DualArmRobo(DualArmRobo_2);
% calc_gj_2arm(DualArmRobo_2.LP, );




