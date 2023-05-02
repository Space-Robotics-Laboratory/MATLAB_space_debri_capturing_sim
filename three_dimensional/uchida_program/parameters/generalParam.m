% 
function generalParam = generalParam()
%% Color Setting
paleRed = [1, .5, .5];
paleBlue = [.5, .5, 1];

%% Graphic Setting
generalParam.viewPoint = [.2, 1, .5]';    % 3d視点
generalParam.bodyColor = paleBlue;    % ベース色
generalParam.targetColor = paleRed;
generalParam.roboForceColor = 'g';
generalParam.targetForceColor = 'r';
generalParam.sphereParticles = 10;      % 球描画の刻み数
generalParam.quiverScale = .01;     % ベクトル表示スケール


end