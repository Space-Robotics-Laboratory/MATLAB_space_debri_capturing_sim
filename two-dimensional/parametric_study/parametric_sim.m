% parameterを変更してmain_sim.mをループするプログラム．
% csvとして捕獲状態を記録する
%
% 2023.6 Akiyoshi Uchida
% for RSJ2023

%% initialize parameters
param = set_Param();

%% modify unlooped param
param.general.endTime = 45;
param.general.visualizeAnimation = false;
param.general.saveData = false;
param.general.makeAnimation = false;
param.general.makeGraph = false;
param.control.controlMode = 'MULTIPLE';
param.robot.initial_jointsAngle(1:4,  1) =   [ pi/3 -pi/3 -pi/2 0 ]';
param.robot.initial_jointsAngle(5:8, 1)  = - [ pi/3 -pi/3 -pi/2 0 ]';
param.control.maxContactForce = 1.12969; % max force in direct case
    % - 1.047   # joint La :PI/3                    ___    ___        
    % - -1.047  # joint Lb :-PI/3                  |          |                             
    % - -1.571  # joint Lc :-PI/2                  \ ________ /                        
    % - -1.047  # joint Ra :-PI/3                   |        |      
    % - 1.047   # joint Rb :PI/3                    |________| wide
    % - 1.571   # joint Rc :PI/2 

%% path setting
timepath = datestr( now, 'yyyy-mmdd-HHMMSS' );  % yはyear，mはmonth,dはday,Hはhour,Mはminute,Sはsecond.それぞれの文字数分出力する
datepath = datestr( now, 'yyyy-mmdd' );
dataSavePath = './iSpaRo2024';
path_parametric = [ dataSavePath '/' datepath '/' timepath ];
table_path = [path_parametric, '/tables/'];
mkdir(table_path)

%% parameters to be looped
w_s = [3];
mi_s = [0.01, 0.05, 0.2];
di_s = 0:0.5:5;
ki_s = 0:2:20 ;

raw = length(ki_s);
col = length(di_s);

v_names = string(di_s);
r_names = string(ki_s);
varTypes = repmat({'string'}, 1, col);


%% タイマースタート                                               
startCPUT = cputime;
startT = clock();

%% loop loop 
for w = w_s
    for mi = mi_s
        ind_c = 0;
        tab_name_ = 'w' + string(w) + '___mi' + string(mi);
        tab_name = table_path + strrep(tab_name_, '.', '_');
        res_table = table('Size',[raw col] ,'VariableTypes', varTypes, 'VariableNames', v_names, 'RowNames', r_names);
        for di = di_s
            ind_r = 0;
            ind_c = ind_c + 1;
            for ki = ki_s
                ind_r = ind_r + 1;
                % display
                value = sprintf("w : %3.4f mi : %3.4f di : %3.4f ki : %3.4f\n", w, mi, di, ki);
                fprintf(value)

                % set param
                param.target.initial_angular_velocity = [0, 0, w]';
                param.control.mi = [1, 1, 1]' * mi;
                param.control.ki = [1, 1, 1]' * ki;
                param.control.di = [1, 1, 1]' * di;
                data_name_temp = '__di' + string(di) + '___ki' + string(ki);
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