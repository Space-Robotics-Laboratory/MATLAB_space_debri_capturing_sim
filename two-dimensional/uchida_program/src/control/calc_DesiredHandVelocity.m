% 2023.1 uchida akiyoshi
%
% ターゲット状態から目標の手先位置を計算する
% 
% 関節自由度が6なので，desiredVelocityも6次元
%
% input : Time scaler, Pos [xL, yL, thetaL, xR, yR, thetaR]'
% output: vel [vxL, vyL, wzL, vxR, vyR, wzR]'
% 

function vel = calc_DesiredHandVelocity(CurrentTime, StartTime, DeltaTime, StartPos, EndPos)
    s = (CurrentTime - StartTime) / DeltaTime;      % 無次元化時間 0~1

    % 静止状態
    if s < 0 || s > 1
        vel = zeros(6, 1);

    % 直線軌道
    % r = r0 + s・(rg - r0 ) 
    else
        drds = (EndPos - StartPos);     % dr/ds
        dsdt = 1 / DeltaTime;           % ds/dt
        vel = drds * dsdt;           % dr/dt = dr/ds * ds/dt
    end
end