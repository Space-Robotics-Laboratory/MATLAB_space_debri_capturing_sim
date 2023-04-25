% struct データからグラフを作成する関数
%
% 2023.2 akiyoshi uchida
%

function make_graph(datStruct, paths)
time = datStruct.time;
startFigNum = 102;

%%% make endEffector FT graph
% ベクトルの大きさで評価
endTipL1F = vecnorm(datStruct.endTipL1Force, 2, 2);
endTipL2F = vecnorm(datStruct.endTipL2Force, 2, 2);
endTipR1F = vecnorm(datStruct.endTipR1Force, 2, 2);
endTipR2F = vecnorm(datStruct.endTipR2Force, 2, 2);

endTipL1T = vecnorm(datStruct.endTipL1Torque, 2, 2);
endTipL2T = vecnorm(datStruct.endTipL2Torque, 2, 2);
endTipR1T = vecnorm(datStruct.endTipR1Torque, 2, 2);
endTipR2T = vecnorm(datStruct.endTipR2Torque, 2, 2);

% force
figureNumber = startFigNum;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, endTipL1F )
hold on
plot(time, endTipL2F )
plot(time, endTipR1F )
plot(time, endTipR2F )

title("End Effector Force")
legend('Left Tip1 Force', 'Left Tip2 Force', 'Right Tip1 Force', 'Right Tip1 Force')
ylabel("Force [N]")
xlabel("time [sec]")
hold off

figName = 'endEffecForce.fig';                                  % fig名定義
pngName = 'endEffecForce.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

% torque
figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, endTipL1T )
hold on
plot(time, endTipL2T )
plot(time, endTipR1T )
plot(time, endTipR2T )

title("End Effector Torque")
legend('Left Tip1 Torque', 'Left Tip2 Torque', 'Right Tip1 Torque', 'Right Tip1 Torque')
ylabel("Torque [Nm]")
xlabel("time [sec]")
hold off

figName = 'endEffecTorque.fig';                                  % fig名定義
pngName = 'endEffecTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%%% make joint torque graph
% active joints
jointsTorque = datStruct.jointTorque';

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointsTorque([1:3,5:7], :))
hold off
title("Active Joint Torque")
legend('MotorJ1', 'MotorJ2', 'MotorJ3', 'MotorJ5', 'MotorJ6', 'MotorJ7')
ylabel("Torque [Nm]")
xlabel("time [sec]")

figName = 'motorTorque.fig';                                  % fig名定義
pngName = 'motorTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

% passive joints
figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointsTorque([4,8], :))
hold off
title("Passive Joint Torque")
legend('LeftWrist', 'RightWrist')
ylabel("Torque [Nm]")
xlabel("time [sec]")

figName = 'wristTorque.fig';                                  % fig名定義
pngName = 'wristTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%%% make target angular velocity graph
targW = datStruct.targetW(:, 3);

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, targW)
title("Target Angular Velocity")
ylabel("Angular Velocity [rad/sec]")
xlabel("time [sec]")

figName = 'targetAngVel.fig';                                  % fig名定義
pngName = 'targetAngVel.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存
