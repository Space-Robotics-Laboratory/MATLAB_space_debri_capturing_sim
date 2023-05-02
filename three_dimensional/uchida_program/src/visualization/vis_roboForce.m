% ロボットエンドエフェクターにかかる外力を可視化する関数
%
% 2023.2 akiyoshi uchida
% 
% input: enEfP(3*4), roboF 3*1


function vis_roboForce(endEffecPos, roboFL, roboFR, scale, color)
vecStart = endEffecPos; % 3*4
vecSize = [roboFL, roboFR] * scale; % 3*4

quiver3(vecStart(1, :), vecStart(2, :), vecStart(3,:), vecSize(1, :), vecSize(2, :), vecSize(3, :), 0, 'Color', color, LineWidth=3)