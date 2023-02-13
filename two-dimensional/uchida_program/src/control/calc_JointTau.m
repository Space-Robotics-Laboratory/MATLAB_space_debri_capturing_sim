% 一般化ヤコビ行列を用いて目標手先速度から関節トルクを計算する
% 
% 2023.1 uchida akiyoshi
%
% 阿部さん修論参照．変数名もできるだけ論文を参考しにているので，確認するときは論文と比較しながら
% 運動量，角運動量が0の場合
% 先首関節はバネダンパ系の受動関節であるので，関節速度に制約が加わる
% input : DualArmRobo class, DesierdHandVelocity 6*1, 
%         RoboExtWrench 6*3(base, leftEdgem, rightEdge) 
% output: JointTorque 8*1, but only used 6*1
%
% to do: HHを計算する際，複数の関数で計算しているのでロスがある．関数の構造を変更することを検討．
% 

function jointTau = calc_JointTau(DualArmRobo, Target, RoboExtEst, mode, time)
global d_time
robo = DualArmRobo;
target = Target;
switch mode
    case 1
        % 初めに絶対位置で制御
        pathWayL = [robo.PATHWAY_e(:, 1), [-0.1, 0.3, pi/10, 1]'];
        pathWayR = [robo.PATHWAY_e(:, 2), [ 0.1, 0.3, 0, 1]'];
        desiredVelL = calc_StraightVel(pathWayL, time);
        desiredVelR = calc_StraightVel(pathWayR, time);
        
        % 次に相対位置で制御
        if time >= 3
            jointTau = calc_Tau_RelatVel(robo, zeros(6,1));
            return
        end

        jointTau = calc_Tau_AbsVel(robo, [desiredVelL; desiredVelR], RoboExtEst);
end
end
