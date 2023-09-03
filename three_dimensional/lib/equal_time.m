% 誤差を考慮して時間を比較する

function res = equal_time(time1, time2, dtime)
    res = abs(time1 - time2) < dtime;
end