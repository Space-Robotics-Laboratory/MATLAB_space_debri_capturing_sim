% parameterを変更してmain_sim.mをループするプログラム．
% csvとして捕獲状態を記録する
%
% 2023.6 Akiyoshi Uchida
% for RSJ2023

% initialize parameters
param = set_Param();
data_path = './tables/';

% w_s = [4, 6, 8];
% mi_s = 0.1:0.6:2.5;
% ki_s = 0.1:0.2:2;
% di_s = 1:2:20;

w_s = 4;
mi_s = 0.1:0.1:0.201;
ki_s = 0.1:0.2:0.3;
di_s = 1:2;

raw = length(ki_s);
col = length(di_s);

v_names = string(di_s);
r_names = string(ki_s);
varTypes = repmat({'string'}, 1, col);

for w = w_s
    for mi = mi_s
        ind_c = 0;
        tab_name_ = 'mi' + string(mi) + '___w' + string(w);
        tab_name = data_path + strrep(tab_name_, '.', '_') + '.txt';
        res_table = table('Size',[raw col] ,'VariableTypes', varTypes, 'VariableNames', v_names, 'RowNames', r_names);
        for di = di_s
            ind_r = 0;
            ind_c = ind_c + 1;
            for ki = ki_s
                ind_r = ind_r + 1;
                param.target.initial_angular_velocity = [0, 0, w]';
                param.control.mi = [1, 1, 1]' * mi;
                param.control.ki = [1, 1, 1]' * ki;
                param.control.di = [1, 1, 1]' * di;
                capture_res = main_sim(param);
                res_table(ind_r, ind_c) = {capture_res};
                writetable(res_table, tab_name, 'WriteRowNames',true)
            end
        end
    end
end