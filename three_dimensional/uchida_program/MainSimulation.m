%%%%%%%%%%DualArmTestBed Simulation%%%%%%%%%%
% 2023/01/01 Akiyoshi Uchida
% SpaceDyn_v2r0
% 
% main simulation
%
clc
clear 
close all
  
%%%%% シミュレーション準備
%%% パラメータ設定
% 基本的にパラメータはParamSetting内で変更する．
param  = set_Param();                   

%%% global 変数の定義 
% 全ての関数及びメインルーチン内で共通で使用される変数
global d_time
global Gravity
global Ez
Ez = [ 0 0 1 ]';
d_time = param.DivTime; % シミュレーション1step当たりの時間
Gravity = [ 0 0 0 ]'; % 重力（地球重力は Gravity = [0 0 -9.8]）

% パス設定
paths = make_dataFolder(param);              % 保存先フォルダ作成．パスはParamSettingで設定
% パラメータ変数保存
save([paths.datfile, '/parameters.m'], "param", '-mat')

% 双腕ロボインスタンス作成
dualArmRobo  = DualArmRobo(param);
% ターゲットインスタンス作成
targetCube = Target(param);
% コントローラーインスタンス作成
controller = Controller(dualArmRobo, 0, param);

% シミュレーション時間
endTime    = param.EndTime;                 % 終了時間設定．
minusTime = param.MinusTime;                % マイナス時間設定．

% ロボット・ターゲット力初期化
roboExtWrench  = zeros(6,5);                   % ロボ外力[ BaseForce    LeftTip1Force   LeftTip2Force   RightTip1Force  RightTip2Force]
                                               % 　　　　[ BaseTorque   LeftTip1Torque  LeftTip2Torque  RightTip1Torque RightTip2Torque]
targetExtWrench= zeros(6,1);                   % タゲ外力[ BaseForce  ] 
                                               % 　　　　[ BaseTorque ] 
% 状態判定用インスタンス初期化
state = State();

% データ保存用インスタンス作成
datSaver = DataSaver(paths, param);

% タイマースタート                                               
startCPUT = cputime;
startT = clock();

figure(1);

%% シミュレーションループスタート
for time = minusTime : d_time : endTime 
    clc
    time %#ok<NOPTS> 
    %%% データ更新
    datSaver = datSaver.update(dualArmRobo, targetCube, controller, time, param);

    %%% 推定フェーズ
    % 接触判定及び接触力計算
    [roboExtWrench(:, 2:5), targetExtWrench, isContact] = calc_contactForce(dualArmRobo, targetCube, param);
    
    % 手先外力センサー値計算
    roboFTsensor = roboExtWrench(:,[2,4])+roboExtWrench(:,[3,5]); % 手先の球にかかる力を足して左右のエンドエフェクタにかかる力にする 6*4->6*2

    % ターゲット運動状態推定
    estTarget = estimate_target(targetCube);

    %%% コントロールフェーズ
    % 手先目標位置計算
    % 目標関節トルク計算 要素であるtauをロボクラスに代入することで操作
    controller = controller.control(dualArmRobo, targetCube, roboFTsensor, time, state, param);

    %%% 運動計算フェーズ
    % 運動状態更新
    dualArmRobo  = dualArmRobo.update(controller.tau, roboExtWrench, param);    % methodを呼び出した後自身に代入することを忘れない！
    targetCube = targetCube.update(targetExtWrench);  

    % 状態判定更新
    state = state.update(dualArmRobo, isContact, targetCube, time, param);

    % 情報視覚化(動画作成はしない)
    if rem(time, .05) == 0
        clf('reset')
        dualArmRobo.vis(param)
        hold on
        dualArmRobo.visForce(param)
        targetCube.vis(param)
        view(param.general.viewPoint)
        pause(.001)
    end
end
%% ループ終了
%%% シミュレーション時間の計測と表示 
show_calc_time(startT, startCPUT)

%%% データ保存
datSaver.write()

%% 結果表示
% アニメーション作成
% movfileにaviファイル保存
% pngfileにpngファイル保存
make_3dAnime(datSaver, paths, param)

% グラフ作成
make_graph(datSaver.datStruct, paths)

%clear
fclose('all');
close all
%%% EOF