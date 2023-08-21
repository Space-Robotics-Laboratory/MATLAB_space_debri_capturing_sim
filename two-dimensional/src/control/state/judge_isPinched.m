% ケージングではなく，挟み込んでいる
function [isPinched, force_holder_] = judge_isPinched(force_holder, robo, param)
border = 2; % [s]

force_applied = any(robo.SV.Fes~=0, 1);
force_holder_(force_applied)  = force_holder(force_applied) + param.general.divTime;
force_holder_(~force_applied) = 0;
isPinched = any(force_holder_ > border, "all");

end