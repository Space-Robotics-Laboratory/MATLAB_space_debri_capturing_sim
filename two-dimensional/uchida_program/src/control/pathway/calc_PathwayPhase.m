% pathwayのどの段階にいるか返す．
% phase の切り替わりのタイミングも返すが，計算誤差に注意
%
% 2023.2 akiyoshi uchida
%

function [phase, phaseStarting, phaseEnding] = calc_PathwayPhase(pathway, time, param)
[~, n, ~] = size(pathway);
index = (pathway(4, :, 1) <= time) & (pathway(4, [2:n, 1], 1) > time);  % 時刻をもとに経路のフェーズを判定
% 時刻(pathway(4,:)が順番通りでない場合，エラーを排出する
if sum(index) >= 2                                                           
    error("you have to set pathway in order of time")
end

% pathwayの初めの時刻より前であれば（通常負の時刻），phaseは０
if time < pathway(4, 1, 1)
    phase = 0;
    phaseStarting = (time - param.MinusTime) < param.DivTime * 1.01 && ...
                    (time - param.MinusTime) >= 0;
    phaseEnding = (pathway(4, 1, 1) - time) <= param.DivTime * 1.01 && ...
                  (pathway(4, 1, 1) - time) > 0;
    return

% pathwayの最後の事項より後であれば，phaseは-1
elseif time >= pathway(4, n, 1)
    phase = -1;
    phaseStarting = (time - pathway(4, n, 1)) < param.DivTime * 1.01 && ...
                    (time - pathway(4, n, 1)) >= 0;
    phaseEnding = true;
    return
end
IND = 1:n;
phase = IND(index);
phaseStarting = (time - pathway(4, phase, 1)) < param.DivTime * 1.01 && ... 
                (time - pathway(4, phase, 1)) >= 0; % おそらく余分
phaseEnding = (pathway(4, phase + 1, 1) - time) <= param.DivTime * 1.01 && ...
              (pathway(4, phase + 1, 1) - time) > 0; % おそらく余分
