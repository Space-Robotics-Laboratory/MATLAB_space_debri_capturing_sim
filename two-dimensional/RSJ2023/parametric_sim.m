% parameterを変更してmain_sim.mをループするプログラム．
% csvとして捕獲状態を記録する
%
% 2023.6 Akiyoshi Uchida
% for RSJ2023

%% initialize parameters
param = set_Param();

%% modify unlooped param
param.general.endTime = 30;
param.general.anime_frameRate = 10;
param.control.controlMode = 'MULTIPLE';

%% path setting
timepath = datestr( now, 'yyyy-mmdd-HHMMSS' );  % yはyear，mはmonth,dはday,Hはhour,Mはminute,Sはsecond.それぞれの文字数分出力する
datepath = datestr( now, 'yyyy-mmdd' );
dataSavePath = '/Users/akiyoshi/develop/srl/github/MATLAB_space_debri_capturing_sim/two-dimensional/RSJ2023';
path_parametric = [ dataSavePath '/' datepath '/' timepath ];
table_path = [path_parametric, '/tables/'];
mkdir(table_path)

%% parameters to be looped
% 0617
w_s = 3;
mi_s = 0.001:0.1:1.001;
ki_s = 0:1:5;
di_s = 1:1:20;

raw = length(mi_s);
col = length(di_s);

v_names = string(di_s);
r_names = string(mi_s);
varTypes = repmat({'string'}, 1, col);


%% タイマースタート                                               
startCPUT = cputime;
startT = clock();

%% loop loop 
for w = w_s
    for ki = ki_s
        ind_c = 0;
        tab_name_ = 'ki' + string(ki) + '___w' + string(w);
        tab_name = table_path + strrep(tab_name_, '.', '_');
        res_table = table('Size',[raw col] ,'VariableTypes', varTypes, 'VariableNames', v_names, 'RowNames', r_names);
        for di = di_s
            ind_r = 0;
            ind_c = ind_c + 1;
            for mi = mi_s
                ind_r = ind_r + 1;
                % display
                value = sprintf("w : %2.2f ki : %2.2f mi : %2.2f di : %2.2f\n", w, ki, mi, di);
                fprintf(value)

                % set param
                param.target.initial_angular_velocity = [0, 0, w]';
                param.control.mi = [1, 1, 1]' * mi;
                param.control.ki = [1, 1, 1]' * ki;
                param.control.di = [1, 1, 1]' * di;
                data_name_temp = '__di' + string(di) + '___mi' + string(mi);
                data_name_temp = strrep(tab_name_, '.', '_') + strrep(data_name_temp, '.', '_');
                data_name = char(data_name_temp);
                paths = make_dataFolder_Parametric(data_name, path_parametric);

                % main_sim
                capture_res = main_sim(param, paths);
                res_table(ind_r, ind_c) = {capture_res};
                writetable(res_table, tab_name, 'WriteRowNames',true)
            end
        end
    end
end

%% ループ終了
%%% シミュレーション時間の計測と表示 
disp("All Parameters Have Been Calculated")
show_calc_time(startT, startCPUT)