% ターゲット外力描画関数
% 
% 2023.3 akiyoshi uchida
% 

function vis_TargetForce(targR0, targF, scale)
vecStart = targR0; % 3*2
vecSize = targF * scale;

quiver(vecStart(1, :), vecStart(2, :), vecSize(1, :), vecSize(2, :), 0, LineWidth=3)
