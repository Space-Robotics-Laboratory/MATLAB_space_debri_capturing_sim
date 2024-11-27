function wb = bodyFrameBaseAngVel(SV)
%BODYFRAMEANGVEL この関数の概要をここに記述
%   詳細説明をここに記述
ws = tilde(SV.w0);
wb_tilde = SV.A0' * ws * SV.A0;
wb = inv_tilde(wb_tilde);
end

