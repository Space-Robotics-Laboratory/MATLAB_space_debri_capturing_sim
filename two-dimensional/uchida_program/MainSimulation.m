%%%%%%%%%%DualArmTestBed Simulation%%%%%%%%%%
% 2023/01/01 Akiyoshi Uchida
% SpaceDyn_v2r0
% 
% main simulation
%

clc
clear 
close all

% パラメータ設定
% 基本的にパラメータはParamSetting内で変更する．
% ループ内で用いるパラメータはここで呼び出すことによって時間短縮？
param  = set_Param();                   
cElast = param.ContactElast;     % 接触弾性係数
cDamp  = param.ContactDamp;      % 接触減衰係数
cNu    = param.ContactNu;        % 接触摩擦係数



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% global 変数の定義 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 全ての関数及びメインルーチン内で共通で使用される変数
global d_time
global Gravity
global Ez
Ez = [ 0 0 1 ]';
d_time = param.DivTime; % シミュレーション1step当たりの時間
Gravity = [ 0 0 0 ]'; % 重力（地球重力は Gravity = [0 0 -9.8]）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% パス設定
fileNameList = ["Anime.txt"];                     % 保存するデータファイル名
paths = make_DataFolder(param);              % 保存先フォルダ作成．パスはParamSettingで設定
fileIDList = FilesOpen(paths, fileNameList);      % ファイルを開き，ファイルIDを配列に格納．現状意味ないかも

% 保存用データ見出し
animeTitle = set_AnimeTitleHeader();
DataOut(fileIDList(fileNameList=="Anime.txt"), animeTitle,  param.StringType, param.Delimiter)   % アニメデータファイルの見出しを書き出し

% 双腕ロボインスタンス作成
dualArmRobo  = DualArmRobo(param);
% ターゲットインスタンス作成
targetSquare = TargetSquare(param);

% シミュレーション準備
endTime    = param.EndTime;               % 終了時間設定．
minusTime = param.MinusTime;             % マイナス時間設定．
index = 1;

% ロボット・ターゲット力初期化
roboJointTau = zeros(8,1);                     % ロボ関節制御トルク．手首は受動関節であることに注意
roboExtWrench  = zeros(6,3);                   % ロボ外力[ BaseForce    LeftEndEfecForce   RightEndEfecForce  ]
                                               % 　　　　[ BaseTorque   LeftEndEfecTorque  RightEndEfecTorque ]
targetExtWrench= zeros(6,1);                   % タゲ外力[ BaseForce  ] 
                                               % 　　　　[ BaseTorque ] 
% 接触判定初期化 1*4 
state.isContact = false(1, 4);                 % isContact(1, i) はendEfec i（初期姿勢にて左から）のターゲットへの接触状態bool値
state.wasContact = state.isContact;            % 1step前のisContact


% 捕獲判定初期化 bool shcalar
state.isCapture = false;

% ロボット手先目標軌跡([pathwayLeft, pathwayRight]) 4*n*2初期化
% pathwayは[x(t), y(t), theta(t), t]'の形で，時刻tにおける座標を示す．
pathway(:, 1, 1) = [dualArmRobo.POS_e_L(1:2); dualArmRobo.ORI_e_L(3); 0];   % pathwayLeft
pathway(:, 1, 2) = [dualArmRobo.POS_e_R(1:2); dualArmRobo.ORI_e_R(3); 0];   % pathwayRight

% データ保存用インスタンス作成
datSaver = DataSaver(paths, param);

% タイマースタート                                               
startCPUT = cputime;
startT = clock();

% シミュレーションループスタート
for time = minusTime : d_time : endTime 
    clc
    time %#ok<NOPTS> 

    %%% データ更新
    % Anime
    dataAnime = [dualArmRobo.SV.R0', dualArmRobo.SV.Q0', reshape(dualArmRobo.POS_j_L,[1,12]), reshape(dualArmRobo.POS_j_R,[1,12]),   ...
                     reshape(dualArmRobo.POS_es_L,[1,6]), reshape(dualArmRobo.POS_es_R,[1,6]), dualArmRobo.SV.QeL', dualArmRobo.SV.QeR', ...
                     targetSquare.SV.R0', targetSquare.SV.Q0'];   
    datSaver = datSaver.update(dualArmRobo, targetSquare, time, index);
    DataOut(fileIDList(fileNameList=="Anime.txt"), dataAnime, param.DataType, param.Delimiter)

    %%% 推定フェーズ
    % 接触判定及び接触力計算
    [roboExtWrench(:, 2:3), targetExtWrench, state.isContact] = calc_ContactForce(dualArmRobo, targetSquare, cElast, cDamp, cNu);
    
    % 手先外力センサー値計算
%     roboExtEst = zeros(6, 3);
    roboExtEst = roboExtWrench;

    % ターゲット運動状態推定
    estTarget = estimate_Target(targetSquare);

    % ターゲット捕獲状況判定
    state.isCapture = judge_IsCapture(dualArmRobo, estTarget, param);

    %%% コントロールフェーズ
    % 手先目標位置計算
    % 目標関節トルク計算
    [roboJointTau, pathway] = control_JointTau(dualArmRobo, targetSquare, pathway, roboExtEst, state, param,time);

    %%% 運動計算フェーズ
    % 運動状態更新
    dualArmRobo  = dualArmRobo.update(roboJointTau, roboExtWrench, param);    % methodを呼び出した後自身に代入することを忘れない！
    targetSquare = targetSquare.update(targetExtWrench);  

    % 1step前の接触データ更新
    state.wasContact = state.isContact;
    index = index + 1;
end

%%% データ保存
datSaver.write()

%%% 結果表示
% アニメーション作成
% movfileにaviファイル保存
% pngfileにpngファイル保存
make_2dAnime("Anime.txt", paths, param)

% グラフ作成
make_Graph(datSaver.datStruct, paths)

% ファイルクローズ
fclose('all');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% シミュレーション時間の計測と表示 %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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



