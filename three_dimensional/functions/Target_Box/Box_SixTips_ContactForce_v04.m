function [ contactflag, F_NN, FR_N, FR_T, tang, FR, delta, delta_vel] ...
= Box_SixTips_ContactForce_v04(contactflag, SV_ts, SV_tsR0_tmp, d_time, r_tip, cof, kw, cw, ...
                         Contact_point, normal_vector, D_close, Contact_point_tmp, delta_tmp)

 tang = 0;
 delta = 0;
 delta_vel = 0;

%�ڐG���Ă���Ƃ��@�ڐG��F��Ԃ�
if contactflag == 1
    
        %�ڐG���A�߂荞�ݗʌv�Z�i��X�����͂߂荞�ݏ�ԁA�^�[�Q�b�g�̌`�󓙂ɂ���ČʂɌv�Z����j
        if  contactflag == 1  &&  D_close <= r_tip%�i�ꉞ�j
            delta = abs( r_tip - D_close );  %%delta�͐��ƂȂ邪�A�O�̂��ߐ��l�ɂ���             delta(i,j) = r_tip - distance(i,j); 
        else    
            delta = 0;
        end
        
        %�߂荞�ݑ��x
        if  delta - delta_tmp > 0  %�߂荞�ݑ����Ă���ꍇ
            delta_vel = (delta - delta_tmp) / d_time;
        else                                %����Ă���r���i�@���͎͂󂯂Ȃ��j
            delta_vel = 0;
        end

        %�ڐG�_�ɉ����� force    
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

% else   % �ڐG���Ă��Ȃ��Ƃ�(�ꉞ)
%     F_NN = 0.0;

end


% FR_N = F_NN * norm;   % ���{�b�g�����󂯂�@����
FR_N =  F_NN  * normal_vector(1:3,1) / norm(normal_vector(1:3,1));  %force
FR_T = cof * F_NN * tang;   % ��悪�󂯂門�C��

FR = FR_N + FR_T;

end