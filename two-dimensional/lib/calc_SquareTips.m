% ベース幾何中心位置，姿勢から４つの頂点の座標を求める．
% 
% 2023.1 uchdia akiyoshi 
% 
% GCenter 2*1
% Tips : [x1 x2 x3 x4
%         y1 y2 y3 y4] 左下->左上->右上->右下

function TipsPos = calc_SquareTips(GCenter, Width, Depth, OriZ)
    Tip1 = GCenter - Width*.5*[cos(OriZ); sin(OriZ)] - Depth*.5*[-sin(OriZ); cos(OriZ)];
    Tip2 = GCenter - Width*.5*[cos(OriZ); sin(OriZ)] + Depth*.5*[-sin(OriZ); cos(OriZ)];
    Tip3 = GCenter + Width*.5*[cos(OriZ); sin(OriZ)] + Depth*.5*[-sin(OriZ); cos(OriZ)];
    Tip4 = GCenter + Width*.5*[cos(OriZ); sin(OriZ)] - Depth*.5*[-sin(OriZ); cos(OriZ)];
    TipsPos = [Tip1 Tip2 Tip3 Tip4];
end