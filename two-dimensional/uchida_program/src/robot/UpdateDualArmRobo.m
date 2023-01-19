% 外力，関節トルクをもとにロボット状態を更新

function DualArmRobo = UpdateDualArmRobo(DualArmRobo, JointTau, ExtWrench, Param)
    %能動的な力
    DualArmRobo.SV.tau = JointTau;                  % 関節トルク代入

    %受動的な力
    DualArmRobo.SV.T0 = ExtWrench(1:3, 1);          % ベーストルク
    DualArmRobo.SV.F0 = ExtWrench(4:6, 1);          % ベース力
    DualArmRobo.SV.Te(:, 4) = ExtWrench(1:3, 2);    % 左手手先トルク
    DualArmRobo.SV.Fe(:, 4) = ExtWrench(4:6, 2);    % 左手手先力
    DualArmRobo.SV.Te(:, 8) = ExtWrench(1:3, 3);    % 右手手先トルク
    DualArmRobo.SV.Fe(:, 8) = ExtWrench(4:6, 3);    % 右手手先力

    % 順運動学によって関節位置，角度，手先位置，手先角度を計算
    DualArmRobo.SV = f_dyn_rk2(DualArmRobo.LP, DualArmRobo.SV);                                                      % ロボットに関する順動力学
    DualArmRobo.SV = calc_aa(  DualArmRobo.LP, DualArmRobo.SV );                                                     % 各リンクの座標返還行列(方向余弦行列)の計算(リンクi->慣性座標系)
    DualArmRobo.SV = calc_pos( DualArmRobo.LP, DualArmRobo.SV );                                                     % 各リンク重心位置の計算
    [ DualArmRobo.POS_j_L, DualArmRobo.ORI_j_L ] = f_kin_j( DualArmRobo.LP, DualArmRobo.SV, DualArmRobo.jointsL );   % 左手 関節位置・姿勢　jointsLは左手の関節数
    [ DualArmRobo.POS_j_R, DualArmRobo.ORI_j_R ] = f_kin_j( DualArmRobo.LP, DualArmRobo.SV, DualArmRobo.jointsR );   % 右手 関節位置・姿勢　jointsRは右手の関節数
    [ DualArmRobo.POS_e_L, DualArmRobo.ORI_e_L ] = f_kin_e( DualArmRobo.LP, DualArmRobo.SV, DualArmRobo.jointsL );   % 左手先位置・姿勢（位置は２つの手先の中点）
    [ DualArmRobo.POS_e_R, DualArmRobo.ORI_e_R ] = f_kin_e( DualArmRobo.LP, DualArmRobo.SV, DualArmRobo.jointsR );   % 右手先位置・姿勢（位置は２つの手先の中点）
    DualArmRobo.SV.Q0 = dc2rpy( DualArmRobo.SV.A0' );                                                                % ベース角度のオイラー角表現
    DualArmRobo.SV.QeL= dc2rpy( DualArmRobo.ORI_e_L' );                                                              % 左端リンクのオイラー角表現
    DualArmRobo.SV.QeR= dc2rpy( DualArmRobo.ORI_e_R' );                                                              % 右端リンクのオイラー角表現
    DualArmRobo.POS_es_L = CalcArmTips(DualArmRobo.POS_e_L, DualArmRobo.ORI_e_L, Param);                             % 左手の先端球位置 3*2
    DualArmRobo.POS_es_R = CalcArmTips(DualArmRobo.POS_e_R, DualArmRobo.ORI_e_R, Param);                             % 右手の先端球位置 3*2
end