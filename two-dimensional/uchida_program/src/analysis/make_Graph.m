% struct データからグラフを作成する関数
%
% 2023.2 akiyoshi uchida
%

function make_Graph(datStruct, paths)
time = datStruct.time;

%%% make endEffector FT graph
% ベクトルの大きさで評価
endEffecLF = vecnorm(datStruct.endEffecLForce, 2, 2);
endEffecRF = vecnorm(datStruct.endEffecRForce, 2, 2);
endEffecLT = vecnorm(datStruct.endEffecLTorque, 2, 2);
endEffecRT = vecnorm(datStruct.endEffecRTorque, 2, 2);

% force
figureNumber = 102;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, endEffecLF )
hold on
plot(time, endEffecRF)
title("End Effector Force")
legend('Left Force', 'Right Force')
ylabel("Force [N]")
xlabel("time [sec]")
hold off

figName = 'endEffecForce.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % png保存

% torque
figureNumber = 103;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, endEffecLT )
hold on
plot(time, endEffecRT)
title("End Effector Torque")
legend('Left Torque', 'Right Torque')
ylabel("Torque [Nm]")
xlabel("time [sec]")
hold off

figName = 'endEffecTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % png保存

%%% make joint torque graph
% 関節トルクの最大値で評価
jointLTorque = abs(datStruct.jointTorque(:, 1:4));
jointRTorque = abs(datStruct.jointTorque(:, 5:8));
jointLTorque = max(jointLTorque, [], 2);
jointRTorque = max(jointRTorque, [], 2);

figureNumber = 104;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointLTorque)
hold on
plot(time, jointRTorque)
title("Joint Motor Torque Abslute Value")
legend('Left Torque', 'Right Torque')
ylabel("Torque [Nm]")
xlabel("time [sec]")
hold off

figName = 'jointTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % png保存


