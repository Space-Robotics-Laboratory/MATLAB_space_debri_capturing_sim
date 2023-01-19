% 一般化ヤコビ行列を用いて目標手先速度から関節トルクを計算する
% 先首関節はバネダンパ系の受動関節
% input : DualArmRobo class, DesierdHandVelocity 3*2
% output: JointTorque 8*1

function JointTau = calc_JointTau(DualArmRobo, DesiredHandVel, Parameters)
    JointTau = zeros(6,1);
end
