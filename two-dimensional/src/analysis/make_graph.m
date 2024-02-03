% struct データからグラフを作成する関数
%
% 2023.2 akiyoshi uchida
%

function make_graph(datStruct, time_length, paths)
time = datStruct.time;
startFigNum = 102;
fontSize = 22;
lineWidth = 2;
showTitle = false;

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

plot(time, endTipL1F, "LineWidth", lineWidth)
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold on
plot(time, endTipL2F, "LineWidth", lineWidth)
plot(time, endTipR1F, "LineWidth", lineWidth)
plot(time, endTipR2F, "LineWidth", lineWidth)

if(showTitle)
    title("End Effector Force")
end
legend('Left Tip 1', 'Left Tip 2', 'Right Tip 1', 'Right Tip 1')
ylabel("Force [N]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])
hold off

figName = 'endEffecForce.fig';                                  % fig名定義
pngName = 'endEffecForce.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

% torque
figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, endTipL1T, "LineWidth", lineWidth)
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold on
plot(time, endTipL2T, "LineWidth", lineWidth)
plot(time, endTipR1T, "LineWidth", lineWidth)
plot(time, endTipR2T, "LineWidth", lineWidth)

if(showTitle)
    title("End Effector Torque")
end
legend('Left Tip 1', 'Left Tip 2', 'Right Tip 1', 'Right Tip 1')
ylabel("Torque [Nm]")
xlabel("Time [s]")
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

plot(time, jointsTorque([1:3,5:7], :), "LineWidth", lineWidth)
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
if(showTitle)
    title("Active Joint Torque")
end
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')
ylabel("Torque [Nm]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])

figName = 'motorTorque.fig';                                  % fig名定義
pngName = 'motorTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

% passive joints
figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointsTorque([4,8], :), "LineWidth", lineWidth)
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
if(showTitle)
    title("Passive Joint Torque")
end
legend('LeftWrist', 'RightWrist')
ylabel("Torque [Nm]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])

figName = 'wristTorque.fig';                                  % fig名定義
pngName = 'wristTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%%% make target angular velocity graph
targW = datStruct.targetW(:, 3);

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, targW, "LineWidth", lineWidth)
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
if(showTitle)
    title("Target Angular Velocity")
end
ylabel("Angular Velocity [rad/s]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])

figName = 'targetAngVel.fig';                                  % fig名定義
pngName = 'targetAngVel.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%ロボット手先速度グラフ化
velL = vecnorm(datStruct.roboEndEffecLVel,2,2);
velR = vecnorm(datStruct.roboEndEffecRVel,2,2);

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, velL, "LineWidth", lineWidth)
hold on
plot(time, velR, "LineWidth", lineWidth)
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
if(showTitle)
    title("End Effector Velocity");
end
legend("Left Arm", "Right Arm")
xlabel("Time [s]");
ylabel("Velocity [m/s]");

figName = 'endEffecVel.fig';                                  % fig名定義
pngName = 'endEffecVel.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存