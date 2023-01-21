% 一般化ヤコビ行列を用いて目標手先速度から関節トルクを計算する
% 手先力がある場合にも手先速度を制御可能
% 2023.1 uchida akiyoshi
%
% 阿部さん修論参照．変数名もできるだけ論文を参考しにているので，確認するときは論文と比較しながら
% 運動量，角運動量が0の場合
% 先首関節はバネダンパ系の受動関節だが，一時的に能動関節に設定
% input : DualArmRobo class, DesierdHandVelocity 8*1
% output: JointTorque 6*1（手首以外）-> 8*1

function JointTau = calc_JointTau(DualArmRobo, DesiredHandVel)
global d_time
    % 一般化慣性行列，一般化速度非線形項計算
    [H_asuta, C_asuta] = calc_asuta(DualArmRobo.LP, DualArmRobo.SV);

    % 一般化ヤコビアン計算
    Jg = calc_gj_DualArmRobo(DualArmRobo);

    % 目標自由度削減 8*8
    Jg_s = Jg;
    Jg_s([4, 5, 10, 11], :) = [];

    % 目標関節角速度計算
    qd_des = pinv(Jg_s) * DesiredHandVel;
    qdd_des = (qd_des - DualArmRobo.SV.qd) / d_time;
    JointTau = H_asuta * qdd_des + C_asuta;
end
