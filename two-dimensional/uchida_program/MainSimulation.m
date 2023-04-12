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
paths = make_DataFolder(param);              % 保存先フォルダ作成．パスはParamSettingで設定
% パラメータ変数保存
save([paths.datfile, '/parameters.m'], "param", '-mat')

% 双腕ロボインスタンス作成
dualArmRobo  = DualArmRobo(param);
% ターゲットインスタンス作成
targetSquare = TargetSquare(param);
% コントローラーインスタンス作成
controller = Controller(dualArmRobo, 0, 2);

% シミュレーション時間
endTime    = param.EndTime;                 % 終了時間設定．
minusTime = param.MinusTime;                % マイナス時間設定．
datIndex = 1;                               % データ保存用インデックス

% ロボット・ターゲット力初期化
roboExtWrench  = zeros(6,5);                   % ロボ外力[ BaseForce    LeftTip1Force   LeftTip2Force   RightTip1Force  RightTip2Force]
                                               % 　　　　[ BaseTorque   LeftTip1Torque  LeftTip2Torque  RightTip1Torque RightTip2Torque]
targetExtWrench= zeros(6,1);                   % タゲ外力[ BaseForce  ] 
                                               % 　　　　[ BaseTorque ] 
% 接触判定初期化 1*4 
state.isContact = false(1, 4);                 % isContact(1, i) はendEfec i（初期姿勢にて左から）のターゲットへの接触状態bool値
state.wasContact = state.isContact;            % 1step前のisContact
state.newContact = ~state.wasContact & state.isContact;
state.endContact = state.wasContact & ~state.isContact;
state.targetSlow = false;
state.time.lastContact = inf;
state.time.comeTargetSlow = inf;


% 捕獲判定初期化 bool schalar
state.isCapture = false;

% データ保存用インスタンス作成
datSaver = DataSaver(paths, param);

% タイマースタート                                               
startCPUT = cputime;
startT = clock();


%% シミュレーションループスタート
for time = minusTime : d_time : endTime 
    clc
    time %#ok<NOPTS> 

    %%% データ更新
    datSaver = datSaver.update(dualArmRobo, targetSquare, controller, time, datIndex, param);

    %%% 推定フェーズ
    % 接触判定及び接触力計算
    [roboExtWrench(:, 2:5), targetExtWrench, state.isContact] = calc_ContactForce(dualArmRobo, targetSquare, param);
    
    % 手先外力センサー値計算
    roboFTsensor = roboExtWrench(:,[2,4])+roboExtWrench(:,[3,5]); % 手先の球にかかる力を足して左右のエンドエフェクタにかかる力にする 6*4->6*2

    % ターゲット運動状態推定
    estTarget = estimate_Target(targetSquare);

    % ターゲット状況判定
    state.isCapture = judge_IsCapture(dualArmRobo, estTarget, param);
    if ~state.targetSlow && (abs(targetSquare.SV.w0(3)) <= param.AngularVelBorder)
        state.time.comeTargetSlow = time;
    end
    state.targetSlow = abs(targetSquare.SV.w0(3)) <= param.AngularVelBorder;

    %%% コントロールフェーズ
    % 手先目標位置計算
    % 目標関節トルク計算 要素であるtauをロボクラスに代入することで操作
    controller = controller.control(dualArmRobo, targetSquare, roboFTsensor, time, state, param);

    %%% 運動計算フェーズ
    % 運動状態更新
    dualArmRobo  = dualArmRobo.update(controller.tau, roboExtWrench, param);    % methodを呼び出した後自身に代入することを忘れない！
    targetSquare = targetSquare.update(targetExtWrench);  

    % 接触データ更新
    if any(state.isContact)
        state.time.lastContact = time;
    end
    state.newContact = ~state.wasContact & state.isContact;
    state.endContact = state.wasContact & ~state.isContact;
    state.wasContact = state.isContact;
    datIndex = datIndex + 1;
end
%%% ループ終了


%%% データ保存
datSaver.write()


%% 結果表示
% アニメーション作成
% movfileにaviファイル保存
% pngfileにpngファイル保存
make_2dAnime(datSaver, paths, param)

% グラフ作成
make_Graph(datSaver.datStruct, paths)

% ファイルクローズ
fclose('all');


%%% シミュレーション時間の計測と表示 

% シミュレーション全体時間 単位:秒
ntime = cputime - startCPUT;

nhour = floor( ntime / 3600 );                    % 単位:時間 各要素以下の最も近い整数に丸める
nmin  = floor( ( ntime - nhour * 3600 ) / 60 );   % 単位:分 残りの分，整数に丸める
nsec  = ntime - nhour * 3600 - nmin * 60;         % 単位:秒 残りの秒，整数に丸める

% 結果表示
fprintf( '\n\n %s %s', '開始時間 :', datestr( startT, 31 ) );
fprintf( '\n %s %s',   '終了時間 :', datestr( clock,  31 ) );
fprintf( '\n %s %d %s %02d %s %04.1f %s \n\n\n', '計算所要時間 :', nhour, ' 時間 ', nmin, ' 分 ', nsec, ' 秒 ' );

%clear
close all
%%% EOF



