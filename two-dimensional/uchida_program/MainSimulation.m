%%%%%%%%%%DualArmTestBed Simulation%%%%%%%%%%
%2023/01/01 Akiyoshi Uchida
%SpaceDyn_v2r0

clc
clear all
close all

%パラメータ設定
Parameters = ParamSetting();


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
% 保存用フォルダ作成
FileNameList = ["RobotMotion.txt", "Target_1.txt"];                     %保存するデータファイル名
paths = PathSetting(Parameters);                                        %保存先フォルダ作成．パスはParamSettingで設定
FileIDList = FilesOpen(paths, FileNameList);                            %ファイルを開き，ファイルIDを配列に格納

%保存用データ見出し
TitleRobotMotion  = ["BasePosX","BasePosY","BasePosZ","BaseOriX","BaseOriY","BaseOriZ", ...
    "JointPos1X","JointPos1Y","JointPos1Z","JointPos2X","JointPos2Y","JointPos2Z", ...
    "JointPos3X","JointPos3Y","JointPos3Z","JointPos4X","JointPos4Y","JointPos4Z", ...
    "JointPos5X","JointPos5Y","JointPos5Z","JointPos6X","JointPos6Y","JointPos6Z", ...
    "JointPos7X","JointPos7Y","JointPos7Z","JointPos8X","JointPos8Y","JointPos8Z"];
TitleTargetMotion = ["TargetPosX", "TargetPosY", "TargetPosZ", "TargetOriX", "TargetOriY", "TargetOriZ"];

DataOut(FileIDList(FileNameList=="RobotMotion.txt"), TitleRobotMotion,  '%10s', Parameters.Delimiter)   %ロボモーションデータファイルの見出しを書き出し
DataOut(FileIDList(FileNameList=="Target_1.txt"   ), TitleTargetMotion, '%10s', Parameters.Delimiter)   %ターゲットモーションデータファイルの見出しを書き出し                  



%双腕ロボクラス定義
DualArmTestBed_1 = DualArmTestBed(Parameters);


%シミュレーションループスタート
endtime    = Parameters.EndTime;               % 終了時間設定．ここで変更しない
minus_time = Parameters.MinusTime;             % マイナス時間設定．ここで変更しない
RoboJointTau   = [40, 0, 0, 0,   -40, 0, 0, 0]';  % ロボ関節制御トルク
RoboExtWrench  = zeros(6,3);                   % ロボ外力[ BaseTorque   LeftEdgeTorque  RightEdgeTorque ]
                                               % 　　　　[ BaseForce    LeftEdgeForce   RightEdgeForce  ]  
startCPUT = cputime;
startT = clock();

for time = 0 : d_time : ( endtime + minus_time )
    clc
    time %#ok<NOPTS> 
    % 運動状態更新
    DualArmTestBed_1 = DualArmTestBed_1.Update(RoboJointTau, RoboExtWrench);            % methodを呼び出した後自身に代入することを忘れない！
   
    % データ書き出し
    % RoboMotion
    data = [DualArmTestBed_1.SV.R0', DualArmTestBed_1.SV.Q0', reshape(DualArmTestBed_1.POS_j_L,[1,12]), reshape(DualArmTestBed_1.POS_j_R,[1,12])];   
    DataOut(FileIDList(FileNameList=="RobotMotion.txt"), data, '%10f', Parameters.Delimiter)                  
                                                                                        
end

% アニメーション作成
% movfileに保存
Make2dAnime([paths.datfile, '/', char("RobotMotion.txt")], Parameters)

%ファイルクローズ
fclose('all');

%%%%%%%%%%%%%%%%%%%% シミュレーション時間の計測と表示 %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% シミュレーション全体時間 単位:秒
ntime = cputime - startCPUT;

nhour = floor( ntime / 3600 );                   % 単位:時間 各要素以下の最も近い整数に丸める
nmin = floor( ( ntime - nhour * 3600 ) / 60 );   % 単位:分 残りの分，整数に丸める
nsec = ntime - nhour * 3600 - nmin * 60;         % 単位:秒 残りの秒，整数に丸める

% 結果表示
fprintf( '\n\n %s %s', '開始時間 :', datestr( startT, 31 ) );
fprintf( '\n %s %s', '終了時間 :', datestr( clock, 31 ) );
fprintf( '\n %s %d %s %02d %s %04.1f %s \n\n\n', '計算所要時間 :', nhour, ' 時間 ', nmin, ' 分 ', nsec, ' 秒 ' );

%clear
%close all
%%% EOF



