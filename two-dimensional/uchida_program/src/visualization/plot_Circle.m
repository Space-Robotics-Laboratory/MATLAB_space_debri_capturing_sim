% 円描画関数

function plot_Circle(pos, r, color)
    plot(pos(1)+r*cos(0:0.1:2*pi), pos(2)+r*sin(0:.1:2*pi), '-', 'Color', color, 'LineWidth',3)
end