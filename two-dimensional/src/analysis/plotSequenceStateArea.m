function plotSequenceStateArea(time, state)
%PLOTSEQUENCESTATEAREA この関数の概要をここに記述
%   詳細説明をここに記述
% time = datStruct.time;
% state = datStruct.sequenceState;

area_detumbling = [-1.3; -1.2];
area_tryCapturing = [-1.2; -1.1];
area_captureCompleted = [-1.1; -1];

if time(state == 1)
  area_detumbling = time(state == 1);
end
if time(state == 2)
  area_tryCapturing = time(state == 2);
end
if time(state == 3)
  area_captureCompleted = time(state == 3);
end

X = [area_detumbling([1, end]), area_tryCapturing([1, end]), area_captureCompleted([1, end])];

xr = xregion(X);
xr(1).FaceColor = "r";
xr(1).DisplayName = "Detumbling";
xr(2).FaceColor = "c";
xr(2).DisplayName = "Capturing";
xr(3).FaceColor = "g";
xr(3).DisplayName = "Captured";
end

