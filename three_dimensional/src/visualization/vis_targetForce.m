% ターゲット外力描画関数
% 
% 2023.3 akiyoshi uchida
% 

function vis_targetForce(targR0, targF, scale, color)
vecStart = targR0; % 3*2
vecSize = targF * scale;

quiver3(vecStart(1, :), vecStart(2, :), vecStart(3,:), vecSize(1, :), vecSize(2, :), vecSize(3, :), 0, 'Color', color,LineWidth=3)
