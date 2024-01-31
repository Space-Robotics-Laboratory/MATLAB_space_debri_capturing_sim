%%%%%%%%%%DualArmTestBed Simulation%%%%%%%%%%
% 2023/01/01 Akiyoshi Uchida
% SpaceDyn_v2r0
% 
% main simulation
%
% RSJ2023用に，parametric解析を行うように改良．main_simの返り値は，捕獲結果のtex形式表現

function sim_res = main_sim(param, paths)
arguments
    %%% パラメータ設定
    % main_sim 実行時にパラメータを与えることも可能
    param = set_Param(); 
    % パス設定
    paths = make_DataFolder(param);              % 保存先フォルダ作成．パスはParamSettingで設定
end

%%%%% シミュレーション準備             
%%% global 変数の定義 
% 全ての関数及びメインルーチン内で共通で使用される変数
global d_time
global Gravity
global Ez
Ez = [ 0 0 1 ]';
d_time = param.general.divTime; % シミュレーション1step当たりの時間
Gravity = [ 0 0 0 ]'; % 重力（地球重力は Gravity = [0 0 -9.8]）
sim_res = '-                                  ';

% 双腕ロボインスタンス作成
dualArmRobo  = DualArmRobo(param);
% ターゲットインスタンス作成
targetSquare = TargetSquare(param);
% コントローラーインスタンス作成
controller = Controller(dualArmRobo, 0, param);

% シミュレーション時間
endTime    = param.general.endTime;                 % 終了時間設定．
minusTime = param.general.minusTime;                % マイナス時間設定．

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
break_time = inf;

%% シミュレーションループスタート
for time = minusTime : d_time : endTime 
    if rem(time, 0.1) == 0
        timer_count = sprintf("time : %2.2f [s]\n", time);
        fprintf(timer_count)
    end
 
    %%% データ更新
    datSaver = datSaver.update(dualArmRobo, targetSquare, controller, param);

    %%% 推定フェーズ
    % 接触判定及び接触力計算
    [roboExtWrench(:, 2:5), targetExtWrench, isContact] = calc_contactForce(dualArmRobo, targetSquare, param);
    
    % 手先外力センサー値計算
    roboFTsensor = roboExtWrench(:,[2,4])+roboExtWrench(:,[3,5]); % 手先の球にかかる力を足して左右のエンドエフェクタにかかる力にする 6*4->6*2

    % ターゲット運動状態推定
    estTarget = estimate_target(targetSquare);

    %%% コントロールフェーズ
    % 手先目標位置計算
    % 目標関節トルク計算 要素であるtauをロボクラスに代入することで操作
    controller = controller.control(dualArmRobo, targetSquare, roboFTsensor, time, state, param);

    %%% 運動計算フェーズ
    dualArmRobo  = dualArmRobo.update(controller.tau, roboExtWrench, param);    % methodを呼び出した後自身に代入することを忘れない！
    targetSquare = targetSquare.update(targetExtWrench);  

    % 状態判定更新
    state = state.update(dualArmRobo, isContact, targetSquare, time, param);

    % 以降simulation中断処理
    if time > break_time
        break
    end
    if break_time ~= inf
        continue
    end

    % ターゲット回転減衰 -> 黄∆
    if state.targetSlow
        sim_res = '\cellcolor{yellow}{$\bigtriangleup$}';
    end

    % 捕獲
    if state.targetStop
        if state.isCapture % ケージング成功 -> 緑 ✓
            sim_res = '\cellcolor{green}{$\checkmark    $}';
            break_time = time + param.general.breakTimeDuration;
        elseif state.isPinch % 力による挟み込み -> 緑o
            sim_res = '\cellcolor{green}{$\circ$         }';
            break_time = time;
        end
    end
    % ベース接触 -> 赤x
    if state.isBaseContact
        sim_res = '\cellcolor{red}{$\times          $}';
        break_time = time;
    end
    % 突き飛ばし -> 黄x
    if state.goneAway 
        sim_res = '\cellcolor{yellow}{$\times       $}';
        break_time = time;
    end

    % simulation error
    if any(isnan(dualArmRobo.SV.R0), "all") 
        sim_res = '          !                        ';
        break_time = time;
    end
end
%% ループ終了
%%% シミュレーション時間の計測と表示 
show_calc_time(startT, startCPUT)
disp(sim_res)

%%% データ保存
datSaver = datSaver.write(param);

%% 結果表示
% アニメーション作成
% movfileにaviファイル保存
% pngfileにpngファイル保存
make_2dAnime(datSaver, paths, param)

% グラフ作成
make_graph(datSaver.datStruct, datSaver.timer_length, paths)

%clear
fclose('all');
close all
%%% EOF