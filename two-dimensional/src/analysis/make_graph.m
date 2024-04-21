% struct データからグラフを作成する関数
%
% 2023.2 akiyoshi uchida
%

function make_graph(datStruct, time_length, paths)
time = datStruct.time;
startFigNum = 102;
fontSize = 32;
lineWidth = 2;
showTitle = false;
plotBaseAngVel = true;

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
box on
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold on
plot(time, endTipL2F, "LineWidth", lineWidth)
plot(time, endTipR1F, "LineWidth", lineWidth)
plot(time, endTipR2F, "LineWidth", lineWidth)

if(showTitle)
    title("End Effector Force")
end
legend('Left tip 1', 'Left tip 2', 'Right tip 1', 'Right tip 2')
ylabel("Force [N]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])
box off
hold off

figName = 'endEffecForce.fig';                                  % fig名定義
pngName = 'endEffecForce.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

% torque
figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, endTipL1T, "LineWidth", lineWidth)
box on
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold on
plot(time, endTipL2T, "LineWidth", lineWidth)
plot(time, endTipR1T, "LineWidth", lineWidth)
plot(time, endTipR2T, "LineWidth", lineWidth)

if(showTitle)
    title("End Effector Torque")
end
legend('Left tip 1', 'Left tip 2', 'Right tip 1', 'Right tip 2')
ylabel("Torque [Nm]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])
box off
hold off

figName = 'endEffecTorque.fig';                                  % fig名定義
pngName = 'endEffecTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%%% make joint velocity graph
% active joints
jointsVelocity = datStruct.jointVelocity';

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointsVelocity([1:3,5:7], :), "LineWidth", lineWidth)
box on
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
if(showTitle)
    title("Active Joint Vecloity")
end
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')
ylabel("Velocity [rad/sec]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])
box off

figName = 'motorVelocity.fig';                                  % fig名定義
pngName = 'motorVelocity.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

% passive joints
figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointsVelocity([4,8], :), "LineWidth", lineWidth)
box on
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
if(showTitle)
    title("Passive Joint Velocity")
end
legend('Left wrist', 'Right wrist')
ylabel("Velocity [rad/sec]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])
box off

figName = 'wristVelocity.fig';                                  % fig名定義
pngName = 'wristVelocity.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存


%%% make joint torque graph
% active joints
jointsTorque = datStruct.jointTorque';

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointsTorque([1:3,5:7], :), "LineWidth", lineWidth)
box on
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
if(showTitle)
    title("Active Joint Torque")
end
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6')
ylabel("Torque [Nm]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])
box off

figName = 'motorTorque.fig';                                  % fig名定義
pngName = 'motorTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

% passive joints
figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, jointsTorque([4,8], :), "LineWidth", lineWidth)
box on
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
if(showTitle)
    title("Passive Joint Torque")
end
legend('Left wrist', 'Right wrist')
ylabel("Torque [Nm]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])
box off

figName = 'wristTorque.fig';                                  % fig名定義
pngName = 'wristTorque.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%%% make target angular velocity graph
targW = datStruct.targetW(:, 3);
baseW = datStruct.baseW(:, 3);

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, targW, "LineWidth", lineWidth)
box on
if plotBaseAngVel
  hold on
  plot(time, baseW, "LineWidth", lineWidth)
  legend("Target", "Base")
end
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
if(showTitle)
    title("Target Angular Velocity")
end
ylabel("Angular velocity [rad/s]")
xlabel("Time [s]")
xlim([time(1), time(time_length)])
box off

figName = 'angVel.fig';                                  % fig名定義
pngName = 'angVel.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%%% make robot base attitude graph
%%% make velocity-based manipulability ellipsoid are size graph
baseQ = datStruct.roboQ0(:, 3);
velManip = datStruct.velocityManipulability;

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

yyaxis left
plot(time, baseQ, "LineWidth", lineWidth)
box on
hold on
yyaxis right
plot(time, velManip(:,1), "LineWidth", lineWidth)
plot(time, velManip(:,2), "LineWidth", lineWidth)
legend('Base attitude', 'Left-arm', 'Right-arm')
plotSequenceStateArea(datStruct)
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
yyaxis left
if(showTitle)
    title("Robot Base Attitude")
end
ylabel("Attitude [rad]")
xlabel("Time [s]")
yyaxis right
ylabel("Measure of manipulability [-]")
xlim([time(1), time(time_length)])
box off

figName = 'baseOri.fig';                                  % fig名定義
pngName = 'baseOri.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存

%ロボット手先速度グラフ化
velL = vecnorm(datStruct.roboEndEffecLVel,2,2);
velR = vecnorm(datStruct.roboEndEffecRVel,2,2);

figureNumber = figureNumber+1;     % 図番号設定
figure(figureNumber);   % 図定義

plot(time, velL, "LineWidth", lineWidth)
box on
hold on
plot(time, velR, "LineWidth", lineWidth)
set(gca, 'FontSize', fontSize);  % 軸目盛りのフォントサイズを設定
hold off
if(showTitle)
    title("End Effector Velocity");
end
legend("Left arm", "Right arm")
xlabel("Time [s]");
ylabel("Velocity [m/s]");
box off

figName = 'endEffecVel.fig';                                  % fig名定義
pngName = 'endEffecVel.png';                                  % png名定義
saveas(figure(figureNumber), [paths.figfile, '/', figName]);    % fig保存
saveas(figure(figureNumber), [paths.figfile, '/', pngName]);    % png保存