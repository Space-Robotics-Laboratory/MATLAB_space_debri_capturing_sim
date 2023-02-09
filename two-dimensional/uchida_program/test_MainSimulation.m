%%%%%%%%%%DualArmTestBed Simulation%%%%%%%%%%
% 2023/01/01 Akiyoshi Uchida
% SpaceDyn_v2r0
% 
% main simulation
%

clc
clear all
close all

%パラメータ設定
% 基本的にパラメータはParamSetting内で変更する．
% ループ内で用いるパラメータはここで呼び出すことによって時間短縮？
parameters = set_Param();                   
contactElast = parameters.ContactElast;     % 接触弾性係数
contactDamp  = parameters.ContactDamp;      % 接触減衰係数


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% global 変数の定義 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 全ての関数及びメインルーチン内で共通で使用される変数
global d_time
global Gravity
global Ez
Ez = [ 0 0 1 ]';
d_time = parameters.DivTime; % シミュレーション1step当たりの時間
Gravity = [ 0 0 0 ]'; % 重力（地球重力は Gravity = [0 0 -9.8]）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% パス設定
fileNameList = ["Anime.txt"];                     % 保存するデータファイル名
paths = make_DataFolder(parameters);              % 保存先フォルダ作成．パスはParamSettingで設定
fileIDList = FilesOpen(paths, fileNameList);      % ファイルを開き，ファイルIDを配列に格納．現状意味ないかも

% 保存用データ見出し
animeTitle = set_AnimeTitleHeader();
DataOut(fileIDList(fileNameList=="Anime.txt"), animeTitle,  parameters.StringType, parameters.Delimiter)   % アニメデータファイルの見出しを書き出し

% 双腕ロボインスタンス作成
dualArmRobo_1  = DualArmRobo(parameters);
% ターゲットインスタンス作成
targetSquare_1 = TargetSquare(parameters);

% シミュレーション準備
endTime    = parameters.EndTime;               % 終了時間設定．ここで変更しない
minusTime = parameters.MinusTime;             % マイナス時間設定．ここで変更しない

% ロボット・ターゲット力初期化
roboJointTau = zeros(8,1);                     % ロボ関節制御トルク．手首を除くことに注意
roboExtWrench  = zeros(6,3);                   % ロボ外力[ BaseForce    LeftEdgeForce   RightEdgeForce  ]
                                               % 　　　　[ BaseTorque   LeftEdgeTorque  RightEdgeTorque ]
targetExtWrench= zeros(6,1);                   % タゲ外力[ BaseForce  ] 
                                               % 　　　　[ BaseTorque ] 


% ロボット初期位置
startPos = [[dualArmRobo_1.POS_e_L(1:2);dualArmRobo_1.ORI_e_L(3),]; [dualArmRobo_1.POS_e_R(1:2); dualArmRobo_1.ORI_e_R(3)]];
endPos = [[-0.01005-0.08*sqrt(2) 0.4-0.05 0]'; [0.01005+0.08*sqrt(2) 0.4-0.05 0]'];

% タイマースタート                                               
startCPUT = cputime;
startT = clock();

% シミュレーションループスタート
for time = minusTime : d_time : endTime 
    clc
    time %#ok<NOPTS> 

    % データ書き出し
    % Anime
    dataAnime = [dualArmRobo_1.SV.R0', dualArmRobo_1.SV.Q0', reshape(dualArmRobo_1.POS_j_L,[1,12]), reshape(dualArmRobo_1.POS_j_R,[1,12]),   ...
                     reshape(dualArmRobo_1.POS_es_L,[1,6]), reshape(dualArmRobo_1.POS_es_R,[1,6]), dualArmRobo_1.SV.QeL', dualArmRobo_1.SV.QeR', ...
                     targetSquare_1.SV.R0', targetSquare_1.SV.Q0'];   
    DataOut(fileIDList(fileNameList=="Anime.txt"), dataAnime, parameters.DataType, parameters.Delimiter)

    % 接触力計算
    [roboExtWrench(:, 2:3), targetExtWrench] = calc_ContactForce(dualArmRobo_1, targetSquare_1, contactElast, contactDamp);
    
    % 目標手先速度計算
    desiredHandVel = calc_DesiredHandVelocity(time, 0, 1, startPos, endPos);   % [LeftVel; RoghtVel] 6*1

    % 手先外力センサー値計算
    roboExtEst = zeros(6, 3);

    % 目標関節トルク計算
    roboJointTau = [-0.3, -0.01, 0.01, 0, 0.3, 0.01, -0.01, 0]';

    % 運動状態更新
    dualArmRobo_1  = dualArmRobo_1.update(roboJointTau, roboExtWrench, parameters);    % methodを呼び出した後自身に代入することを忘れない！
    targetSquare_1 = targetSquare_1.update(targetExtWrench);    

end

% アニメーション作成
% movfileにaviファイル保存
% pngfileにpngファイル保存
make_2dAnime("Anime.txt", paths, parameters)

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
%close all
%%% EOF



