% 球描画
%
% 2023.5 akiyoshi uchida 

function plot_sphere(n, pos, r, color)

[x0, y0, z0] = sphere(n);
x = x0 * r + pos(1);
y = y0 * r + pos(2);
z = z0 * r + pos(3);
C = repmat(color, [n, 1, n]);
C = permute(C, [1, 3, 2]);
surf(x, y, z, C)
end