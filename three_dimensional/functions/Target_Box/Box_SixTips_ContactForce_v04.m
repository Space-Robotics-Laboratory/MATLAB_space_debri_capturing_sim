function [ contactflag, F_NN, FR_N, FR_T, tang, FR, delta, delta_vel] ...
= Box_SixTips_ContactForce_v04(contactflag, SV_ts, SV_tsR0_tmp, d_time, r_tip, cof, kw, cw, ...
                         Contact_point, normal_vector, D_close, Contact_point_tmp, delta_tmp)

 tang = 0;
 delta = 0;
 delta_vel = 0;

%接触しているとき　接触力Fを返す
if contactflag == 1
    
        %接触時、めり込み量計算（後々ここはめり込み状態、ターゲットの形状等によって個別に計算する）
        if  contactflag == 1  &&  D_close <= r_tip%（一応）
            delta = abs( r_tip - D_close );  %%deltaは正となるが、念のため正値にする             delta(i,j) = r_tip - distance(i,j); 
        else    
            delta = 0;
        end
        
        %めり込み速度
        if  delta - delta_tmp > 0  %めり込み続けている場合
            delta_vel = (delta - delta_tmp) / d_time;
        else                                %離れている途中（法線力は受けない）
            delta_vel = 0;
        end

        %接触点に加わる力 force    
        F_NN = norm(kw * delta + cw * delta_vel);  % =forcenorm
        
        %
        if norm(Contact_point(1:3,1) - SV_ts.R0 - Contact_point_tmp(1:3,1) + SV_tsR0_tmp) ~= 0 && contactflag > 0  %&& contactflag_tmp(i) > 0  
            tang = (Contact_point(1:3,1) - SV_ts.R0 - Contact_point_tmp(1:3,1) + SV_tsR0_tmp)...
                   / norm(Contact_point(1:3,1) - SV_ts.R0 - Contact_point_tmp(1:3,1) + SV_tsR0_tmp);
        else
            tang = 0;
        end
        
else
    
    F_NN = 0.0;

% else   % 接触していないとき(一応)
%     F_NN = 0.0;

end


% FR_N = F_NN * norm;   % ロボット側が受ける法線力
FR_N =  F_NN  * normal_vector(1:3,1) / norm(normal_vector(1:3,1));  %force
FR_T = cof * F_NN * tang;   % 手先が受ける摩擦力

FR = FR_N + FR_T;

end