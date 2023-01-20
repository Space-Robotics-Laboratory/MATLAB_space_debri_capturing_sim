Parameters = ParamSetting();
DualArmRobo_2 = DualArmRobo(Parameters);
DualArmRobo_2.update(zeros(6,1), zeros(6,3), Parameters)
TargetSquare_2 = TargetSquare(Parameters);
DesiredHandVel = [0, 0, 0, 0, 0, 0]';

calc_JointTau(DualArmRobo_2, DesiredHandVel, Parameters);