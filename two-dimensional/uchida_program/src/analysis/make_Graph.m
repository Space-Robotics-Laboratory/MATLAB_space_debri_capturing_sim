% struct データからグラフを作成する関数
%
% 2023.2 akiyoshi uchida
%

function make_Graph(datStruct, paths)
time = datStruct.time;

%%% make endEffector FT graph
% ベクトルの大きさで評価
endTipL1F = vecnorm(datStruct.endTipL1Force, 2, 2);
endTipL2F = vecnorm(datStruct.endTipL2Force, 2, 2);
endTipR1F = vecnorm(datStruct.endTipL1Force, 2, 2);
endTipR2F = vecnorm(datStruct.endTipL2Force, 2, 2);

endTipL1T = vecnorm(datStruct.endTipL1Torque, 2, 2);
endTipL2T = vecnorm(datStruct.endTipL2Torque, 2, 2);
endTipR1T = vecnorm(datStruct.endTipL1Torque, 2, 2);
endTipR2T = vecnorm(datStruct.endTipL2Torque, 2, 2);

% force
figureNumber = 102;     % 図番号設定
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

figName = 'endEffecForce.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % png保存

% torque
figureNumber = 103;     % 図番号設定
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

figName = 'endEffecTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % png保存

%%% make joint torque graph
jointsTorque = datStruct.jointTorque';

figureNumber = 104;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointsTorque)
title("Joint Torque Value")
legend('MotorJ1', 'MotorJ2', 'MotorJ3', 'LeftWrist', 'MotorJ5', 'MotorJ6', 'MotorJ7', 'RightWrist')
ylabel("Torque [Nm]")
xlabel("time [sec]")
hold off

figName = 'jointTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % png保存

%%% make target angular velocity graph
targW = datStruct.targetW(:, 3);

figureNumber = 105;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, targW)
title("Target Angular Velocity")
ylabel("Angular Velocity [rad/sec]")
xlabel("time [sec]")

figName = 'targetAngVel.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % png保存

