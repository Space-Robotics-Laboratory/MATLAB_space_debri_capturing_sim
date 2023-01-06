% ロボット幾何中心位置，姿勢から４つの頂点の座標を求める．
% Tips : 2*4

function TipsPos = CalcTipsPos(GCenter, Width, Depth, Orientation)
    Tip1 = GCenter - Width*.5*[cos(Orientation); sin(Orientation)] - Depth*.5*[-sin(Orientation); cos(Orientation)];
    Tip2 = GCenter - Width*.5*[cos(Orientation); sin(Orientation)] + Depth*.5*[-sin(Orientation); cos(Orientation)];
    Tip3 = GCenter + Width*.5*[cos(Orientation); sin(Orientation)] + Depth*.5*[-sin(Orientation); cos(Orientation)];
    Tip4 = GCenter + Width*.5*[cos(Orientation); sin(Orientation)] - Depth*.5*[-sin(Orientation); cos(Orientation)];
    TipsPos = [Tip1 Tip2 Tip3 Tip4];
end