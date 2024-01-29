% struct データからグラフを作成する関数
%
% 2023.2 akiyoshi uchida
%

function make_graph(datStruct, time_length, paths)
time = datStruct.time;
startFigNum = 102;
fontSize = 22;

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
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold on
plot(time, endTipL2F )
plot(time, endTipR1F )
plot(time, endTipR2F )

title("End Effector Force")
legend('Left Tip 1 Force', 'Left Tip 2 Force', 'Right Tip 1 Force', 'Right Tip 1 Force')
ylabel("Force [N]")
xlabel("Time [sec]")
xlim([time(1), time(time_length)])
hold off

figName = 'endEffecForce.fig';                                  % fig名定義
pngName = 'endEffecForce.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

% torque
figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, endTipL1T )
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold on
plot(time, endTipL2T )
plot(time, endTipR1T )
plot(time, endTipR2T )

title("End Effector Torque")
legend('Left Tip 1 Torque', 'Left Tip 2 Torque', 'Right Tip 1 Torque', 'Right Tip 1 Torque')
ylabel("Torque [Nm]")
xlabel("Time [sec]")
xlim([time(1), time(time_length)])
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
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
title("Active Joint Torque")
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')
ylabel("Torque [Nm]")
xlabel("Time [sec]")
xlim([time(1), time(time_length)])

figName = 'motorTorque.fig';                                  % fig名定義
pngName = 'motorTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

% passive joints
figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointsTorque([4,8], :))
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
title("Passive Joint Torque")
legend('LeftWrist', 'RightWrist')
ylabel("Torque [Nm]")
xlabel("Time [sec]")
xlim([time(1), time(time_length)])

figName = 'wristTorque.fig';                                  % fig名定義
pngName = 'wristTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%%% make target angular velocity graph
targW = datStruct.targetW(:, 3);

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, targW)
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
title("Target Angular Velocity")
ylabel("Angular Velocity [rad/sec]")
xlabel("Time [sec]")
xlim([time(1), time(time_length)])

figName = 'targetAngVel.fig';                                  % fig名定義
pngName = 'targetAngVel.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%ロボット手先速度グラフ化（変更点）
vel = vecnorm(datStruct.robo_tipVEL_L,2,2);
plot(time, vel)
title("endTipvelocity");
xlabel("time");
ylabel("velocity");

figName = 'dualArmRobo.VEL_e_L.fig';                                  % fig名定義
pngName = 'dualArmRobo.VEL_e_L.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存