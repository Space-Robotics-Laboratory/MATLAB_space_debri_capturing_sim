function plot_axes(ori_mat, pos)
%PLOT_AXES この関数の概要をここに記述
% 二次元で座標系をプロット
%   詳細説明をここに記述
% ori_mat: 3*3 rot matrix
% pos: 3*1 vector
size = 0.06; % 3*4
texpos = pos + ori_mat*size;

quiver(repmat(pos(1), 1, 2), repmat(pos(2), 1, 2), ori_mat(1,1:2), ori_mat(2,1:2), ...
    size, LineWidth=2.2, Color="black", MaxHeadSize=2)

% text(texpos(1,1), texpos(2,1), "x", "FontSize",20)
% text(texpos(1,2), texpos(2,2), "y", "FontSize",20)

end

