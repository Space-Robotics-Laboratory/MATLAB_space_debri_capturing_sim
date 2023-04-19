% 目標手先位置を可視化する関数．デバッグに有効
%
% 2023.3 akiyoshi uchida
%
% input : despos [L, R] 2*2

function vis_DesiredPos(desPos)

plot(desPos(1,:), desPos(2,:), 'o', "Color", 'g', 'MarkerFaceColor','g')
end