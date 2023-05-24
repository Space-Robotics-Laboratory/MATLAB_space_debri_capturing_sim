% 位置，姿勢，サイズから直方体の8つの頂点の座標と6面の構成を返す関数
%
% 2023.4 akiyoshi uchida
%
% gCenter : 3*1
% vertices: 8*3 
% faces : 6*4 ( constant )

function [vertices, faces] = calc_cubic(gCenter, width, depth, height, ORI)

faces = [1, 2, 6, 5;
         2, 3, 7, 6;
         3, 4, 8, 7;
         4, 1, 5, 8;
         1, 2, 3, 4;
         5, 6, 7, 8];
initVertices = zeros(8, 3);
initVertices(faces(1,:), 2) = -depth * .5;
initVertices(faces(2,:), 1) = width * .5;
initVertices(faces(3,:), 2) = depth * .5;
initVertices(faces(4,:), 1) = -width * .5;
initVertices(faces(5,:), 3) = -height * .5;
initVertices(faces(6,:), 3) = height * .5;

vertices = gCenter' + initVertices * ORI'; 