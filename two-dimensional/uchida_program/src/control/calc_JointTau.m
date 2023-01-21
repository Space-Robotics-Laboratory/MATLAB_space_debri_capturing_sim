% 一般化ヤコビ行列を用いて目標手先速度から関節トルクを計算する
%
% 2023.1 uchida akiyoshi
%
% SpaceDynを用いて簡略化できると思われる
%
% 阿部さん修論参照．変数名もできるだけ論文を参考しにているので，確認するときは論文と比較しながら
% 運動量，角運動量が0の場合
% 先首関節はバネダンパ系の受動関節
% input : DualArmRobo class, DesierdHandVelocity 3*2
% output: JointTorque 6*1（手首以外）

function JointTau = calc_JointTau(DualArmRobo, DesiredHandVel, Parameters)
global d_time
    JointTau = [0, 0, 0, -0, 0, 0];
end
