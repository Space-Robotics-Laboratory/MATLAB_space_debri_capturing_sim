% 目標手先位置を可視化する関数．デバッグに有効
%
% 2023.3 akiyoshi uchida
%
% input : despos [L, R] 3*2

function vis_desiredPos(desPos)

plot3(desPos(1,:), desPos(2,:), desPos(3,:), 'o', "Color", 'g', 'MarkerFaceColor','g')
end