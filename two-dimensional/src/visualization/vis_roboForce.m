% ロボットエンドエフェクターにかかる外力を可視化する関数
%
% 2023.2 akiyoshi uchida
% 
% input: enEfP(3*4), roboF 3*1


function vis_roboForce(endEffecPos, roboFL, roboFR, scale)
vecStart = endEffecPos; % 3*4
vecSize = [roboFL, roboFR] * scale; % 3*4

quiver(vecStart(1, :), vecStart(2, :), vecSize(1, :), vecSize(2, :), 0, LineWidth=3)