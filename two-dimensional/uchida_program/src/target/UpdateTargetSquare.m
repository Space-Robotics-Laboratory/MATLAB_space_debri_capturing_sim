% 外力をもとに，ターゲット状態を初期化

function TargetSquare = UpdateTargetSquare(TargetSquare, ExtWrench)
    %受動的な力
    TargetSquare.SV.T0 = ExtWrench(1:3, 1);                         % ターゲットトルク
    TargetSquare.SV.F0 = ExtWrench(4:6, 1);                         % ターゲット力
    
    % 順運動学によって関節位置，角度を計算
    TargetSquare.SV = f_dyn_rk2(TargetSquare.LP, TargetSquare.SV);  % ロボットに関する順動力学
    TargetSquare.SV.Q0 = dc2rpy( TargetSquare.SV.A0' );             % ベース角度のオイラー角表現
end