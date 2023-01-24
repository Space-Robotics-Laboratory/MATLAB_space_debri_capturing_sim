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

% パス設定
FileNameList = ["Anime.txt"];                     % 保存するデータファイル名
paths = make_DataFolder(Parameters);              % 保存先フォルダ作成．パスはParamSettingで設定
FileIDList = FilesOpen(paths, FileNameList);      % ファイルを開き，ファイルIDを配列に格納．現状意味ないかも

% 保存用データ見出し
AnimeTitle = set_AnimeTitleHeader();
DataOut(FileIDList(FileNameList=="Anime.txt"), AnimeTitle,  Parameters.StringType, Parameters.Delimiter)   % アニメデータファイルの見出しを書き出し

% 双腕ロボインスタンス作成
DualArmRobo_1  = DualArmRobo(Parameters);
% ターゲットインスタンス作成
TargetSquare_1 = TargetSquare(Parameters);

% シミュレーション準備
endtime    = Parameters.EndTime;               % 終了時間設定．ここで変更しない
minus_time = Parameters.MinusTime;             % マイナス時間設定．ここで変更しない

% ロボット・ターゲット力初期化
% RoboJointTau   = zeros(6,1);                   % ロボ関節制御トルク，手首関節を除くことに注意 
RoboJointTau = zeros(8,1);                     % ロボ関節制御トルク．手首を能動関節に設定
RoboExtWrench  = zeros(6,3);                   % ロボ外力[ BaseForce    LeftEdgeForce   RightEdgeForce  ]
                                               % 　　　　[ BaseTorque   LeftEdgeTorque  RightEdgeTorque ]
TargetExtWrench= zeros(6,1);                   % タゲ外力[ BaseForce  ]
                                               % 　　　　[ BaseTorque ] 
% タイマースタート                                               
StartCPUT = cputime;
StartT = clock();

% シミュレーションループスタート
for time = minus_time : d_time : endtime 
    clc
    time %#ok<NOPTS> 

    % データ書き出し
    % Anime
    dataAnime = [DualArmRobo_1.SV.R0', DualArmRobo_1.SV.Q0', reshape(DualArmRobo_1.POS_j_L,[1,12]), reshape(DualArmRobo_1.POS_j_R,[1,12]),   ...
                     reshape(DualArmRobo_1.POS_es_L,[1,6]), reshape(DualArmRobo_1.POS_es_R,[1,6]), DualArmRobo_1.SV.QeL', DualArmRobo_1.SV.QeR', ...
                     TargetSquare_1.SV.R0', TargetSquare_1.SV.Q0'];   
    DataOut(FileIDList(FileNameList=="Anime.txt"), dataAnime, Parameters.DataType, Parameters.Delimiter)
    
    % 目標手先速度計算
    DesiredHandVel = calc_DesiredHandVelocity(time);   % [LeftVel; RoghtVel]

    % 手先外力センサー値計算
    % currentry, not used
    RoboExtEst = zeros(6, 3);

    % 目標関節トルク計算
    RoboJointTau = calc_JointTau(DualArmRobo_1, DesiredHandVel, RoboExtEst);

    % 運動状態更新
    DualArmRobo_1  = DualArmRobo_1.update(RoboJointTau, RoboExtWrench, Parameters);    % methodを呼び出した後自身に代入することを忘れない！
    TargetSquare_1 = TargetSquare_1.update(TargetExtWrench);    

end

% アニメーション作成
% movfileにaviファイル保存
% pngfileにpngファイル保存
make_2dAnime("Anime.txt", paths, Parameters)

% ファイルクローズ
fclose('all');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% シミュレーション時間の計測と表示 %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% シミュレーション全体時間 単位:秒
ntime = cputime - StartCPUT;

nhour = floor( ntime / 3600 );                    % 単位:時間 各要素以下の最も近い整数に丸める
nmin  = floor( ( ntime - nhour * 3600 ) / 60 );   % 単位:分 残りの分，整数に丸める
nsec  = ntime - nhour * 3600 - nmin * 60;         % 単位:秒 残りの秒，整数に丸める

% 結果表示
fprintf( '\n\n %s %s', '開始時間 :', datestr( StartT, 31 ) );
fprintf( '\n %s %s',   '終了時間 :', datestr( clock,  31 ) );
fprintf( '\n %s %d %s %02d %s %04.1f %s \n\n\n', '計算所要時間 :', nhour, ' 時間 ', nmin, ' 分 ', nsec, ' 秒 ' );

%clear
%close all
%%% EOF



