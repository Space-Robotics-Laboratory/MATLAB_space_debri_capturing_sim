% シミュレーション設定，表示設定等

function generalParam = generalParam()
%% Color Setting
paleRed = [1, .5, .5];
paleBlue = [.5, .5, 1];

%% Graphic Setting
generalParam.viewPoint = [.2, 1, .5]';      % シミュレーションアニメ表示のカメラ視点
generalParam.bodyColor = paleBlue;          % ロボットベースカラー
generalParam.targetColor = paleRed;         % ターゲットカラー
generalParam.roboForceColor = 'g';          % ロボ力ベクトル色
generalParam.targetForceColor = 'r';        % ターゲット力ベクトル色
generalParam.sphereParticles = 10;          % 球描画の刻み数
generalParam.quiverScale = .01;             % ベクトル表示スケール


end