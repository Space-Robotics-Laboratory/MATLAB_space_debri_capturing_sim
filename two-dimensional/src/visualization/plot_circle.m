% 円描画関数
% 2023.1 uchdia akiyoshi

function plot_circle(pos, r, color)
    plot(pos(1)+r*cos(0:0.1:2*pi+0.1), pos(2)+r*sin(0:.1:2*pi+0.1), '-', 'Color', color, 'LineWidth',1)
end