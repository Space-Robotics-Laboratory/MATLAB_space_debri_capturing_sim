% ベース幾何中心位置，姿勢から４つの頂点の座標を求める．
% GCenter 2*1
% Tips : [x1 x2 x3 x4
%         y1 y2 y3 y4]

function TipsPos = CalcBaseTips(GCenter, Width, Depth, Orientation)
    Tip1 = GCenter - Width*.5*[cos(Orientation); sin(Orientation)] - Depth*.5*[-sin(Orientation); cos(Orientation)];
    Tip2 = GCenter - Width*.5*[cos(Orientation); sin(Orientation)] + Depth*.5*[-sin(Orientation); cos(Orientation)];
    Tip3 = GCenter + Width*.5*[cos(Orientation); sin(Orientation)] + Depth*.5*[-sin(Orientation); cos(Orientation)];
    Tip4 = GCenter + Width*.5*[cos(Orientation); sin(Orientation)] - Depth*.5*[-sin(Orientation); cos(Orientation)];
    TipsPos = [Tip1 Tip2 Tip3 Tip4];
end