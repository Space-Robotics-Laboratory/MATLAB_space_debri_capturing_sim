function [ shikaku1, shikaku2, shikaku3, shikaku4, shikaku1_2, shikaku2_3, shikaku3_4, shikaku4_1, shikaku2_1, shikaku3_2, shikaku4_3, shikaku1_4, ...
           l1, l2, l3, l4, g1, g2, g3, g4, contact_flag_L1, contact_flag_L2, contact_flag_R1, contact_flag_R2 ] ...
= Chohou_FourTips_CONTACT_FLAG_v02( SV_tc, q0, half_taikaku_chohou, theta_chohou, r_tip, tc_geo, POS_eL1, POS_eL2, POS_eR1, POS_eR2 )

% 半径
% half_side_tan : 半短辺
% half_side_cho : 半長辺
% taikaku_chohou : 半対角線
% theta_chohou : atan(半長辺/半短辺)

contact_flag_L1 = zeros(5,5);
contact_flag_L2 = zeros(5,5);
contact_flag_R1 = zeros(5,5);
contact_flag_R2 = zeros(5,5);

% ターゲット頂点設定
    shikaku1(1,1) = tc_geo(1,1) + half_taikaku_chohou * cos( q0 + pi - theta_chohou );
    shikaku1(2,1) = tc_geo(2,1) + half_taikaku_chohou * sin( q0 + pi - theta_chohou );
    shikaku1(3,1) = 0;
    shikaku2(1,1) = tc_geo(1,1) + half_taikaku_chohou * cos( q0 - pi + theta_chohou );
    shikaku2(2,1) = tc_geo(2,1) + half_taikaku_chohou * sin( q0 - pi + theta_chohou );
    shikaku2(3,1) = 0;
    shikaku3(1,1) = tc_geo(1,1) + half_taikaku_chohou * cos( -theta_chohou + q0 );
    shikaku3(2,1) = tc_geo(2,1) + half_taikaku_chohou * sin( -theta_chohou + q0 );
    shikaku3(3,1) = 0;
    shikaku4(1,1) = tc_geo(1,1) + half_taikaku_chohou * cos( theta_chohou + q0 );
    shikaku4(2,1) = tc_geo(2,1) + half_taikaku_chohou * sin( theta_chohou + q0 );
    shikaku4(3,1) = 0;

% ターゲット重心からの各頂点の角度
    l1 = sqrt( ( shikaku1(1,1) - tc_geo(1,1) )^2 + ( shikaku1(2,1) - tc_geo(2,1) )^2 );
    l2 = sqrt( ( shikaku2(1,1) - tc_geo(1,1) )^2 + ( shikaku2(2,1) - tc_geo(2,1) )^2 );
    l3 = sqrt( ( shikaku3(1,1) - tc_geo(1,1) )^2 + ( shikaku3(2,1) - tc_geo(2,1) )^2 );
    l4 = sqrt( ( shikaku4(1,1) - tc_geo(1,1) )^2 + ( shikaku4(2,1) - tc_geo(2,1) )^2 );
    g1 = acos( abs( SV_tc.R0(2,1) - shikaku1(2,1) ) / l1 );
    g2 = acos( abs( SV_tc.R0(2,1) - shikaku2(2,1) ) / l2 );
    g3 = acos( abs( SV_tc.R0(2,1) - shikaku3(2,1) ) / l3 );
    g4 = acos( abs( SV_tc.R0(2,1) - shikaku4(2,1) ) / l4 );

% ターゲット外側頂点設定
    shikaku1_4(1,1) = shikaku1(1,1) + r_tip * cos( pi/2 + q0 );
    shikaku1_4(2,1) = shikaku1(2,1) + r_tip * sin( pi/2 + q0 );
    shikaku1_4(3,1) = 0;
    shikaku1_2(1,1) = shikaku1(1,1) + r_tip * cos( pi + q0 );
    shikaku1_2(2,1) = shikaku1(2,1) + r_tip * sin( pi + q0 );
    shikaku1_2(3,1) = 0;
    shikaku2_1(1,1) = shikaku2(1,1) + r_tip * cos( pi + q0 );
    shikaku2_1(2,1) = shikaku2(2,1) + r_tip * sin( pi + q0 );
    shikaku2_1(3,1) = 0;
    shikaku2_3(1,1) = shikaku2(1,1) + r_tip * cos( -pi/2 + q0 );
    shikaku2_3(2,1) = shikaku2(2,1) + r_tip * sin( -pi/2 + q0 );
    shikaku2_3(3,1) = 0;
    shikaku3_2(1,1) = shikaku3(1,1) + r_tip * cos( -pi/2 + q0 );
    shikaku3_2(2,1) = shikaku3(2,1) + r_tip * sin( -pi/2 + q0 );
    shikaku3_2(3,1) = 0;
    shikaku3_4(1,1) = shikaku3(1,1) + r_tip * cos( q0 );
    shikaku3_4(2,1) = shikaku3(2,1) + r_tip * sin( q0 );
    shikaku3_4(3,1) = 0;
    shikaku4_3(1,1) = shikaku4(1,1) + r_tip * cos( q0 );
    shikaku4_3(2,1) = shikaku4(2,1) + r_tip * sin( q0 );
    shikaku4_3(3,1) = 0;
    shikaku4_1(1,1) = shikaku4(1,1) + r_tip * cos( pi/2 + q0 );
    shikaku4_1(2,1) = shikaku4(2,1) + r_tip * sin( pi/2 + q0 );
    shikaku4_1(3,1) = 0;


% 接触領域定義

%%%%%% ターゲットの姿勢が0のとき
if q0 == 0

a_13 = ( shikaku1(2,1) - shikaku3(2,1) ) / ( shikaku1(1,1) - shikaku3(1,1) );
a_24 = ( shikaku2(2,1) - shikaku4(2,1) ) / ( shikaku2(1,1) - shikaku4(1,1) );
b_13 = shikaku1(2,1) - a_13*shikaku1(1,1);
b_24 = shikaku2(2,1) - a_24*shikaku2(1,1);

% L1
 if     shikaku1_2(1,1) <= POS_eL1(1,1)  &&  shikaku1(2,1) <= POS_eL1(2,1)  &&  POS_eL1(1,1) <= shikaku1(1,1)  &&  POS_eL1(2,1) <= shikaku1_4(2,1)  &&  sqrt( (POS_eL1(1,1)-shikaku1(1,1))^2 + (POS_eL1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L1(1,1) = 111;   % 領域1,1
 elseif shikaku2_1(1,1) <= POS_eL1(1,1)  &&  shikaku2_3(2,1) <= POS_eL1(2,1)  &&  POS_eL1(1,1) <= shikaku2(1,1)  &&  POS_eL1(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_eL1(1,1)-shikaku2(1,1))^2 + (POS_eL1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L1(2,2) = 111;   % 領域2,2
 elseif shikaku3(1,1) <= POS_eL1(1,1)  &&  shikaku3_2(2,1) <= POS_eL1(2,1)  &&  POS_eL1(1,1) <= shikaku3_4(1,1)  &&  POS_eL1(2,1) <= shikaku3(2,1)  &&  sqrt( (POS_eL1(1,1)-shikaku3(1,1))^2 + (POS_eL1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L1(3,3) = 111;   % 領域3,3
 elseif shikaku4(1,1) <= POS_eL1(1,1)  &&  shikaku4(2,1) <= POS_eL1(2,1)  &&  POS_eL1(1,1) <= shikaku4_3(1,1)  &&  POS_eL1(2,1) <= shikaku4_1(2,1)  &&  sqrt( (POS_eL1(1,1)-shikaku4(1,1))^2 + (POS_eL1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L1(4,4) = 111;   % 領域4,4
 elseif shikaku1_2(1,1) <= POS_eL1(1,1)  &&  POS_eL1(2,1) < shikaku1(2,1)  &&  shikaku2(2,1) < POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_13*POS_eL1(1,1)+b_13)  &&  (a_24*POS_eL1(1,1)+b_24) < POS_eL1(2,1)
     contact_flag_L1(1,2) = 111;   % 領域1,2
 elseif shikaku2_3(2,1) <= POS_eL1(2,1)  &&  shikaku2(1,1) < POS_eL1(1,1)  &&  POS_eL1(1,1) < shikaku3(1,1)  &&  POS_eL1(2,1) < (a_13*POS_eL1(1,1)+b_13)  &&  POS_eL1(2,1) <= (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(2,3) = 111;   % 領域2,3
 elseif POS_eL1(1,1) <= shikaku3_4(1,1)  &&  shikaku3(2,1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < shikaku4(2,1)  &&  (a_13*POS_eL1(1,1)+b_13) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(3,4) = 111;   % 領域3,4
 elseif POS_eL1(2,1) <= shikaku4_1(2,1)  &&  shikaku1(1,1) < POS_eL1(1,1)  &&  POS_eL1(1,1) < shikaku4(1,1)  &&  (a_13*POS_eL1(1,1)+b_13) < POS_eL1(2,1)  &&  (a_24*POS_eL1(1,1)+b_24) <= POS_eL1(2,1)
     contact_flag_L1(4,1) = 111;   % 領域4,1
 else
     contact_flag_L1(5,5) = 111;   % 非接触
 end
% L2
 if     shikaku1_2(1,1) <= POS_eL2(1,1)  &&  shikaku1(2,1) <= POS_eL2(2,1)  &&  POS_eL2(1,1) <= shikaku1(1,1)  &&  POS_eL2(2,1) <= shikaku1_4(2,1)  &&  sqrt( (POS_eL2(1,1)-shikaku1(1,1))^2 + (POS_eL2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L2(1,1) = 111;   % 領域1,1
 elseif shikaku2_1(1,1) <= POS_eL2(1,1)  &&  shikaku2_3(2,1) <= POS_eL2(2,1)  &&  POS_eL2(1,1) <= shikaku2(1,1)  &&  POS_eL2(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_eL2(1,1)-shikaku2(1,1))^2 + (POS_eL2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L2(2,2) = 111;   % 領域2,2
 elseif shikaku3(1,1) <= POS_eL2(1,1)  &&  shikaku3_2(2,1) <= POS_eL2(2,1)  &&  POS_eL2(1,1) <= shikaku3_4(1,1)  &&  POS_eL2(2,1) <= shikaku3(2,1)  &&  sqrt( (POS_eL2(1,1)-shikaku3(1,1))^2 + (POS_eL2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L2(3,3) = 111;   % 領域3,3
 elseif shikaku4(1,1) <= POS_eL2(1,1)  &&  shikaku4(2,1) <= POS_eL2(2,1)  &&  POS_eL2(1,1) <= shikaku4_3(1,1)  &&  POS_eL2(2,1) <= shikaku4_1(2,1)  &&  sqrt( (POS_eL2(1,1)-shikaku4(1,1))^2 + (POS_eL2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L2(4,4) = 111;   % 領域4,4
 elseif shikaku1_2(1,1) <= POS_eL2(1,1)  &&  POS_eL2(2,1) < shikaku1(2,1)  &&  shikaku2(2,1) < POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_13*POS_eL2(1,1)+b_13)  &&  (a_24*POS_eL2(1,1)+b_24) < POS_eL2(2,1)
     contact_flag_L2(1,2) = 111;   % 領域1,2
 elseif shikaku2_3(2,1) <= POS_eL2(2,1)  &&  shikaku2(1,1) < POS_eL2(1,1)  &&  POS_eL2(1,1) < shikaku3(1,1)  &&  POS_eL2(2,1) < (a_13*POS_eL2(1,1)+b_13)  &&  POS_eL2(2,1) <= (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(2,3) = 111;   % 領域2,3
 elseif POS_eL2(1,1) <= shikaku3_4(1,1)  &&  shikaku3(2,1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < shikaku4(2,1)  &&  (a_13*POS_eL2(1,1)+b_13) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(3,4) = 111;   % 領域3,4
 elseif POS_eL2(2,1) <= shikaku4_1(2,1)  &&  shikaku1(1,1) < POS_eL2(1,1)  &&  POS_eL2(1,1) < shikaku4(1,1)  &&  (a_13*POS_eL2(1,1)+b_13) < POS_eL2(2,1)  &&  (a_24*POS_eL2(1,1)+b_24) <= POS_eL2(2,1)
     contact_flag_L2(4,1) = 111;   % 領域4,1
 else
     contact_flag_L2(5,5) = 111;   % 非接触
 end
% R1
 if     shikaku1_2(1,1) <= POS_eR1(1,1)  &&  shikaku1(2,1) <= POS_eR1(2,1)  &&  POS_eR1(1,1) <= shikaku1(1,1)  &&  POS_eR1(2,1) <= shikaku1_4(2,1)  &&  sqrt( (POS_eR1(1,1)-shikaku1(1,1))^2 + (POS_eR1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R1(1,1) = 111;   % 領域1,1
 elseif shikaku2_1(1,1) <= POS_eR1(1,1)  &&  shikaku2_3(2,1) <= POS_eR1(2,1)  &&  POS_eR1(1,1) <= shikaku2(1,1)  &&  POS_eR1(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_eR1(1,1)-shikaku2(1,1))^2 + (POS_eR1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R1(2,2) = 111;   % 領域2,2
 elseif shikaku3(1,1) <= POS_eR1(1,1)  &&  shikaku3_2(2,1) <= POS_eR1(2,1)  &&  POS_eR1(1,1) <= shikaku3_4(1,1)  &&  POS_eR1(2,1) <= shikaku3(2,1)  &&  sqrt( (POS_eR1(1,1)-shikaku3(1,1))^2 + (POS_eR1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R1(3,3) = 111;   % 領域3,3
 elseif shikaku4(1,1) <= POS_eR1(1,1)  &&  shikaku4(2,1) <= POS_eR1(2,1)  &&  POS_eR1(1,1) <= shikaku4_3(1,1)  &&  POS_eR1(2,1) <= shikaku4_1(2,1)  &&  sqrt( (POS_eR1(1,1)-shikaku4(1,1))^2 + (POS_eR1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R1(4,4) = 111;   % 領域4,4
 elseif shikaku1_2(1,1) <= POS_eR1(1,1)  &&  POS_eR1(2,1) < shikaku1(2,1)  &&  shikaku2(2,1) < POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_13*POS_eR1(1,1)+b_13)  &&  (a_24*POS_eR1(1,1)+b_24) < POS_eR1(2,1)
     contact_flag_R1(1,2) = 111;   % 領域1,2
 elseif shikaku2_3(2,1) <= POS_eR1(2,1)  &&  shikaku2(1,1) < POS_eR1(1,1)  &&  POS_eR1(1,1) < shikaku3(1,1)  &&  POS_eR1(2,1) < (a_13*POS_eR1(1,1)+b_13)  &&  POS_eR1(2,1) <= (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(2,3) = 111;   % 領域2,3
 elseif POS_eR1(1,1) <= shikaku3_4(1,1)  &&  shikaku3(2,1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < shikaku4(2,1)  &&  (a_13*POS_eR1(1,1)+b_13) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(3,4) = 111;   % 領域3,4
 elseif POS_eR1(2,1) <= shikaku4_1(2,1)  &&  shikaku1(1,1) < POS_eR1(1,1)  &&  POS_eR1(1,1) < shikaku4(1,1)  &&  (a_13*POS_eR1(1,1)+b_13) < POS_eR1(2,1)  &&  (a_24*POS_eR1(1,1)+b_24) <= POS_eR1(2,1)
     contact_flag_R1(4,1) = 111;   % 領域4,1
 else
     contact_flag_R1(5,5) = 111;   % 非接触
 end
% R2
 if     shikaku1_2(1,1) <= POS_eR2(1,1)  &&  shikaku1(2,1) <= POS_eR2(2,1)  &&  POS_eR2(1,1) <= shikaku1(1,1)  &&  POS_eR2(2,1) <= shikaku1_4(2,1)  &&  sqrt( (POS_eR2(1,1)-shikaku1(1,1))^2 + (POS_eR2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R2(1,1) = 111;   % 領域1,1
 elseif shikaku2_1(1,1) <= POS_eR2(1,1)  &&  shikaku2_3(2,1) <= POS_eR2(2,1)  &&  POS_eR2(1,1) <= shikaku2(1,1)  &&  POS_eR2(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_eR2(1,1)-shikaku2(1,1))^2 + (POS_eR2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R2(2,2) = 111;   % 領域2,2
 elseif shikaku3(1,1) <= POS_eR2(1,1)  &&  shikaku3_2(2,1) <= POS_eR2(2,1)  &&  POS_eR2(1,1) <= shikaku3_4(1,1)  &&  POS_eR2(2,1) <= shikaku3(2,1)  &&  sqrt( (POS_eR2(1,1)-shikaku3(1,1))^2 + (POS_eR2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R2(3,3) = 111;   % 領域3,3
 elseif shikaku4(1,1) <= POS_eR2(1,1)  &&  shikaku4(2,1) <= POS_eR2(2,1)  &&  POS_eR2(1,1) <= shikaku4_3(1,1)  &&  POS_eR2(2,1) <= shikaku4_1(2,1)  &&  sqrt( (POS_eR2(1,1)-shikaku4(1,1))^2 + (POS_eR2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R2(4,4) = 111;   % 領域4,4
 elseif shikaku1_2(1,1) <= POS_eR2(1,1)  &&  POS_eR2(2,1) < shikaku1(2,1)  &&  shikaku2(2,1) < POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_13*POS_eR2(1,1)+b_13)  &&  (a_24*POS_eR2(1,1)+b_24) < POS_eR2(2,1)
     contact_flag_R2(1,2) = 111;   % 領域1,2
 elseif shikaku2_3(2,1) <= POS_eR2(2,1)  &&  shikaku2(1,1) < POS_eR2(1,1)  &&  POS_eR2(1,1) < shikaku3(1,1)  &&  POS_eR2(2,1) < (a_13*POS_eR2(1,1)+b_13)  &&  POS_eR2(2,1) <= (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(2,3) = 111;   % 領域2,3
 elseif POS_eR2(1,1) <= shikaku3_4(1,1)  &&  shikaku3(2,1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < shikaku4(2,1)  &&  (a_13*POS_eR2(1,1)+b_13) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(3,4) = 111;   % 領域3,4
 elseif POS_eR2(2,1) <= shikaku4_1(2,1)  &&  shikaku1(1,1) < POS_eR2(1,1)  &&  POS_eR2(1,1) < shikaku4(1,1)  &&  (a_13*POS_eR2(1,1)+b_13) < POS_eR2(2,1)  &&  (a_24*POS_eR2(1,1)+b_24) <= POS_eR2(2,1)
     contact_flag_R2(4,1) = 111;   % 領域4,1
 else
     contact_flag_R2(5,5) = 111;   % 非接触
 end


%%%%%% ターゲットの姿勢がπ/2-θcのとき   _1 : 内側　_2 : 外側
elseif q0 == ( pi/2 - theta_chohou )

a_12_1 = ( shikaku1(2,1) - shikaku2(2,1) ) / ( shikaku1(1,1) - shikaku2(1,1) );
a_12_2 = ( shikaku1_2(2,1) - shikaku2_1(2,1) ) / ( shikaku1_2(1,1) - shikaku2_1(1,1) );
a_23_1 = ( shikaku2(2,1) - shikaku3(2,1) ) / ( shikaku2(1,1) - shikaku3(1,1) );
a_23_2 = ( shikaku2_3(2,1) - shikaku3_2(2,1) ) / ( shikaku2_3(1,1) - shikaku3_2(1,1) );
a_34_1 = ( shikaku3(2,1) - shikaku4(2,1) ) / ( shikaku3(1,1) - shikaku4(1,1) );
a_34_2 = ( shikaku3_4(2,1) - shikaku4_3(2,1) ) / ( shikaku3_4(1,1) - shikaku4_3(1,1) );
a_41_1 = ( shikaku4(2,1) - shikaku1(2,1) ) / ( shikaku4(1,1) - shikaku1(1,1) );
a_41_2 = ( shikaku4_1(2,1) - shikaku1_4(2,1) ) / ( shikaku4_1(1,1) - shikaku1_4(1,1) );
b_12_1 = shikaku1(2,1) - a_12_1*shikaku1(1,1);
b_12_2 = shikaku1_2(2,1) - a_12_2*shikaku1_2(1,1);
b_23_1 = shikaku2(2,1) - a_23_1*shikaku2(1,1);
b_23_2 = shikaku2_3(2,1) - a_23_2*shikaku2_3(1,1);
b_34_1 = shikaku3(2,1) - a_34_1*shikaku3(1,1);
b_34_2 = shikaku3_4(2,1) - a_34_2*shikaku3_4(1,1);
b_41_1 = shikaku4(2,1) - a_41_1*shikaku4(1,1);
b_41_2 = shikaku4_1(2,1) - a_41_2*shikaku4_1(1,1);
a_13 = ( shikaku1(2,1) - shikaku3(2,1) ) / ( shikaku1(1,1) - shikaku3(1,1) );
b_13 = shikaku1(2,1) - a_13*shikaku1(1,1);

% L1
 if     (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  (a_41_1*POS_eL1(1,1)+b_41_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  POS_eL1(2,1) <= (a_41_2*POS_eL1(1,1)+b_41_2)  &&  sqrt( (POS_eL1(1,1)-shikaku1(1,1))^2 + (POS_eL1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L1(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  (a_23_2*POS_eL1(1,1)+b_23_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  POS_eL1(2,1) <= (a_23_1*POS_eL1(1,1)+b_23_1)  &&  sqrt( (POS_eL1(1,1)-shikaku2(1,1))^2 + (POS_eL1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L1(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  (a_23_2*POS_eL1(1,1)+b_23_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(2,1) <= (a_23_1*POS_eL1(1,1)+b_23_1)  &&  sqrt( (POS_eL1(1,1)-shikaku3(1,1))^2 + (POS_eL1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L1(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  (a_41_1*POS_eL1(1,1)+b_41_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(2,1) <= (a_41_2*POS_eL1(1,1)+b_41_2)  &&  sqrt( (POS_eL1(1,1)-shikaku4(1,1))^2 + (POS_eL1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L1(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  (a_23_1*POS_eL1(1,1)+b_23_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_41_1*POS_eL1(1,1)+b_41_1)  &&  POS_eL1(2,1) <= (a_13*POS_eL1(1,1)+b_13)  &&  POS_eL1(1,1) < tc_geo(1,1)
     contact_flag_L1(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eL1(1,1)+b_23_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_13*POS_eL1(1,1)+b_13)  &&  tc_geo(1,1) <= POS_eL1(1,1)
     contact_flag_L1(2,3) = 111;   % 領域2,3
 elseif POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  (a_23_1*POS_eL1(1,1)+b_23_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_41_1*POS_eL1(1,1)+b_41_1)  &&  (a_13*POS_eL1(1,1)+b_13) <= POS_eL1(2,1)  &&  tc_geo(1,1) < POS_eL1(1,1)
     contact_flag_L1(3,4) = 111;   % 領域3,4
 elseif POS_eL1(2,1) <= (a_41_2*POS_eL1(1,1)+b_41_2)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  (a_13*POS_eL1(1,1)+b_13) < POS_eL1(2,1)  &&  POS_eL1(1,1) <= tc_geo(1,1)
     contact_flag_L1(4,1) = 111;   % 領域4,1
 else
     contact_flag_L1(5,5) = 111;   % 非接触
 end
% L2
 if     (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  (a_41_1*POS_eL2(1,1)+b_41_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  POS_eL2(2,1) <= (a_41_2*POS_eL2(1,1)+b_41_2)  &&  sqrt( (POS_eL2(1,1)-shikaku1(1,1))^2 + (POS_eL2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L2(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  (a_23_2*POS_eL2(1,1)+b_23_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  POS_eL2(2,1) <= (a_23_1*POS_eL2(1,1)+b_23_1)  &&  sqrt( (POS_eL2(1,1)-shikaku2(1,1))^2 + (POS_eL2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L2(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  (a_23_2*POS_eL2(1,1)+b_23_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(2,1) <= (a_23_1*POS_eL2(1,1)+b_23_1)  &&  sqrt( (POS_eL2(1,1)-shikaku3(1,1))^2 + (POS_eL2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L2(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  (a_41_1*POS_eL2(1,1)+b_41_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(2,1) <= (a_41_2*POS_eL2(1,1)+b_41_2)  &&  sqrt( (POS_eL2(1,1)-shikaku4(1,1))^2 + (POS_eL2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L2(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  (a_23_1*POS_eL2(1,1)+b_23_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_41_1*POS_eL2(1,1)+b_41_1)  &&  POS_eL2(2,1) <= (a_13*POS_eL2(1,1)+b_13)  &&  POS_eL2(1,1) < tc_geo(1,1)
     contact_flag_L2(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eL2(1,1)+b_23_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_13*POS_eL2(1,1)+b_13)  &&  tc_geo(1,1) <= POS_eL2(1,1)
     contact_flag_L2(2,3) = 111;   % 領域2,3
 elseif POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  (a_23_1*POS_eL2(1,1)+b_23_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_41_1*POS_eL2(1,1)+b_41_1)  &&  (a_13*POS_eL2(1,1)+b_13) <= POS_eL2(2,1)  &&  tc_geo(1,1) < POS_eL2(1,1)
     contact_flag_L2(3,4) = 111;   % 領域3,4
 elseif POS_eL2(2,1) <= (a_41_2*POS_eL2(1,1)+b_41_2)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  (a_13*POS_eL2(1,1)+b_13) < POS_eL2(2,1)  &&  POS_eL2(1,1) <= tc_geo(1,1)
     contact_flag_L2(4,1) = 111;   % 領域4,1
 else
     contact_flag_L2(5,5) = 111;   % 非接触
 end
% R1
 if     (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  (a_41_1*POS_eR1(1,1)+b_41_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  POS_eR1(2,1) <= (a_41_2*POS_eR1(1,1)+b_41_2)  &&  sqrt( (POS_eR1(1,1)-shikaku1(1,1))^2 + (POS_eR1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R1(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  (a_23_2*POS_eR1(1,1)+b_23_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  POS_eR1(2,1) <= (a_23_1*POS_eR1(1,1)+b_23_1)  &&  sqrt( (POS_eR1(1,1)-shikaku2(1,1))^2 + (POS_eR1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R1(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  (a_23_2*POS_eR1(1,1)+b_23_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(2,1) <= (a_23_1*POS_eR1(1,1)+b_23_1)  &&  sqrt( (POS_eR1(1,1)-shikaku3(1,1))^2 + (POS_eR1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R1(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  (a_41_1*POS_eR1(1,1)+b_41_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(2,1) <= (a_41_2*POS_eR1(1,1)+b_41_2)  &&  sqrt( (POS_eR1(1,1)-shikaku4(1,1))^2 + (POS_eR1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R1(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  (a_23_1*POS_eR1(1,1)+b_23_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_41_1*POS_eR1(1,1)+b_41_1)  &&  POS_eR1(2,1) <= (a_13*POS_eR1(1,1)+b_13)  &&  POS_eR1(1,1) < tc_geo(1,1)
     contact_flag_R1(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eR1(1,1)+b_23_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_13*POS_eR1(1,1)+b_13)  &&  tc_geo(1,1) <= POS_eR1(1,1)
     contact_flag_R1(2,3) = 111;   % 領域2,3
 elseif POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  (a_23_1*POS_eR1(1,1)+b_23_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_41_1*POS_eR1(1,1)+b_41_1)  &&  (a_13*POS_eR1(1,1)+b_13) <= POS_eR1(2,1)  &&  tc_geo(1,1) < POS_eR1(1,1)
     contact_flag_R1(3,4) = 111;   % 領域3,4
 elseif POS_eR1(2,1) <= (a_41_2*POS_eR1(1,1)+b_41_2)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  (a_13*POS_eR1(1,1)+b_13) < POS_eR1(2,1)  &&  POS_eR1(1,1) <= tc_geo(1,1)
     contact_flag_R1(4,1) = 111;   % 領域4,1
 else
     contact_flag_R1(5,5) = 111;   % 非接触
 end
% R2
 if     (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  (a_41_1*POS_eR2(1,1)+b_41_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  POS_eR2(2,1) <= (a_41_2*POS_eR2(1,1)+b_41_2)  &&  sqrt( (POS_eR2(1,1)-shikaku1(1,1))^2 + (POS_eR2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R2(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  (a_23_2*POS_eR2(1,1)+b_23_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  POS_eR2(2,1) <= (a_23_1*POS_eR2(1,1)+b_23_1)  &&  sqrt( (POS_eR2(1,1)-shikaku2(1,1))^2 + (POS_eR2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R2(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  (a_23_2*POS_eR2(1,1)+b_23_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(2,1) <= (a_23_1*POS_eR2(1,1)+b_23_1)  &&  sqrt( (POS_eR2(1,1)-shikaku3(1,1))^2 + (POS_eR2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R2(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  (a_41_1*POS_eR2(1,1)+b_41_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(2,1) <= (a_41_2*POS_eR2(1,1)+b_41_2)  &&  sqrt( (POS_eR2(1,1)-shikaku4(1,1))^2 + (POS_eR2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R2(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  (a_23_1*POS_eR2(1,1)+b_23_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_41_1*POS_eR2(1,1)+b_41_1)  &&  POS_eR2(2,1) <= (a_13*POS_eR2(1,1)+b_13)  &&  POS_eR2(1,1) < tc_geo(1,1)
     contact_flag_R2(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eR2(1,1)+b_23_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_13*POS_eR2(1,1)+b_13)  &&  tc_geo(1,1) <= POS_eR2(1,1)
     contact_flag_R2(2,3) = 111;   % 領域2,3
 elseif POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  (a_23_1*POS_eR2(1,1)+b_23_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_41_1*POS_eR2(1,1)+b_41_1)  &&  (a_13*POS_eR2(1,1)+b_13) <= POS_eR2(2,1)  &&  tc_geo(1,1) < POS_eR2(1,1)
     contact_flag_R2(3,4) = 111;   % 領域3,4
 elseif POS_eR2(2,1) <= (a_41_2*POS_eR2(1,1)+b_41_2)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  (a_13*POS_eR2(1,1)+b_13) < POS_eR2(2,1)  &&  POS_eR2(1,1) <= tc_geo(1,1)
     contact_flag_R2(4,1) = 111;   % 領域4,1
 else
     contact_flag_R2(5,5) = 111;   % 非接触
 end


%%%%%% ターゲットの姿勢がπ/2のとき
elseif q0 == pi/2

a_13 = ( shikaku1(2,1) - shikaku3(2,1) ) / ( shikaku1(1,1) - shikaku3(1,1) );
a_24 = ( shikaku2(2,1) - shikaku4(2,1) ) / ( shikaku2(1,1) - shikaku4(1,1) );
b_13 = shikaku1(2,1) - a_13*shikaku1(1,1);
b_24 = shikaku2(2,1) - a_24*shikaku2(1,1);

% L1
 if     shikaku1_4(1,1) <= POS_eL1(1,1)  &&  shikaku1_2(2,1) <= POS_eL1(2,1)  &&  POS_eL1(1,1) <= shikaku1(1,1)  &&  POS_eL1(2,1) <= shikaku1(2,1)  &&  sqrt( (POS_eL1(1,1)-shikaku1(1,1))^2 + (POS_eL1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L1(1,1) = 111;   % 領域1,1
 elseif shikaku2(1,1) <= POS_eL1(1,1)  &&  shikaku2_1(2,1) <= POS_eL1(2,1)  &&  POS_eL1(1,1) <= shikaku2_3(1,1)  &&  POS_eL1(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_eL1(1,1)-shikaku2(1,1))^2 + (POS_eL1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L1(2,2) = 111;   % 領域2,2
 elseif shikaku3(1,1) <= POS_eL1(1,1)  &&  shikaku3(2,1) <= POS_eL1(2,1)  &&  POS_eL1(1,1) <= shikaku3_2(1,1)  &&  POS_eL1(2,1) <= shikaku3_4(2,1)  &&  sqrt( (POS_eL1(1,1)-shikaku3(1,1))^2 + (POS_eL1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L1(3,3) = 111;   % 領域3,3
 elseif shikaku4_1(1,1) <= POS_eL1(1,1)  &&  shikaku4(2,1) <= POS_eL1(2,1)  &&  POS_eL1(1,1) <= shikaku4(1,1)  &&  POS_eL1(2,1) <= shikaku4_3(2,1)  &&  sqrt( (POS_eL1(1,1)-shikaku4(1,1))^2 + (POS_eL1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L1(4,4) = 111;   % 領域4,4
 elseif shikaku1(1,1) < POS_eL1(1,1)  &&  shikaku1_2(2,1) <= POS_eL1(2,1)  &&  POS_eL1(1,1) < shikaku2(1,1)  &&  POS_eL1(2,1) <= (a_13*POS_eL1(1,1)+b_13)  &&  POS_eL1(2,1) < (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(1,2) = 111;   % 領域1,2
 elseif shikaku2(2,1) < POS_eL1(2,1)  &&  POS_eL1(1,1) <= shikaku2_3(1,1)  &&  POS_eL1(2,1) < shikaku3(2,1)  &&  POS_eL1(2,1) < (a_13*POS_eL1(1,1)+b_13)  &&  (a_24*POS_eL1(1,1)+b_24) <= POS_eL1(2,1)
     contact_flag_L1(2,3) = 111;   % 領域2,3
 elseif shikaku4(1,1) < POS_eL1(1,1)  &&  POS_eL1(1,1) < shikaku3(1,1)  &&  POS_eL1(2,1) <= shikaku4_3(2,1)  &&  (a_13*POS_eL1(1,1)+b_13) <= POS_eL1(2,1)  &&  (a_24*POS_eL1(1,1)+b_24) < POS_eL1(2,1)
     contact_flag_L1(3,4) = 111;   % 領域3,4
 elseif POS_eL1(1,1) <= shikaku1_4(1,1)  &&  shikaku1(2,1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < shikaku4(2,1)  &&  (a_13*POS_eL1(1,1)+b_13) < POS_eL1(2,1)  &&  (a_24*POS_eL1(1,1)+b_24) <= POS_eL1(2,1)
     contact_flag_L1(4,1) = 111;   % 領域4,1
 else
     contact_flag_L1(5,5) = 111;   % 非接触
 end
% L2
 if     shikaku1_4(1,1) <= POS_eL2(1,1)  &&  shikaku1_2(2,1) <= POS_eL2(2,1)  &&  POS_eL2(1,1) <= shikaku1(1,1)  &&  POS_eL2(2,1) <= shikaku1(2,1)  &&  sqrt( (POS_eL2(1,1)-shikaku1(1,1))^2 + (POS_eL2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L2(1,1) = 111;   % 領域1,1
 elseif shikaku2(1,1) <= POS_eL2(1,1)  &&  shikaku2_1(2,1) <= POS_eL2(2,1)  &&  POS_eL2(1,1) <= shikaku2_3(1,1)  &&  POS_eL2(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_eL2(1,1)-shikaku2(1,1))^2 + (POS_eL2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L2(2,2) = 111;   % 領域2,2
 elseif shikaku3(1,1) <= POS_eL2(1,1)  &&  shikaku3(2,1) <= POS_eL2(2,1)  &&  POS_eL2(1,1) <= shikaku3_2(1,1)  &&  POS_eL2(2,1) <= shikaku3_4(2,1)  &&  sqrt( (POS_eL2(1,1)-shikaku3(1,1))^2 + (POS_eL2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L2(3,3) = 111;   % 領域3,3
 elseif shikaku4_1(1,1) <= POS_eL2(1,1)  &&  shikaku4(2,1) <= POS_eL2(2,1)  &&  POS_eL2(1,1) <= shikaku4(1,1)  &&  POS_eL2(2,1) <= shikaku4_3(2,1)  &&  sqrt( (POS_eL2(1,1)-shikaku4(1,1))^2 + (POS_eL2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L2(4,4) = 111;   % 領域4,4
 elseif shikaku1(1,1) < POS_eL2(1,1)  &&  shikaku1_2(2,1) <= POS_eL2(2,1)  &&  POS_eL2(1,1) < shikaku2(1,1)  &&  POS_eL2(2,1) <= (a_13*POS_eL2(1,1)+b_13)  &&  POS_eL2(2,1) < (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(1,2) = 111;   % 領域1,2
 elseif shikaku2(2,1) < POS_eL2(2,1)  &&  POS_eL2(1,1) <= shikaku2_3(1,1)  &&  POS_eL2(2,1) < shikaku3(2,1)  &&  POS_eL2(2,1) < (a_13*POS_eL2(1,1)+b_13)  &&  (a_24*POS_eL2(1,1)+b_24) <= POS_eL2(2,1)
     contact_flag_L2(2,3) = 111;   % 領域2,3
 elseif shikaku4(1,1) < POS_eL2(1,1)  &&  POS_eL2(1,1) < shikaku3(1,1)  &&  POS_eL2(2,1) <= shikaku4_3(2,1)  &&  (a_13*POS_eL2(1,1)+b_13) <= POS_eL2(2,1)  &&  (a_24*POS_eL2(1,1)+b_24) < POS_eL2(2,1)
     contact_flag_L2(3,4) = 111;   % 領域3,4
 elseif POS_eL2(1,1) <= shikaku1_4(1,1)  &&  shikaku1(2,1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < shikaku4(2,1)  &&  (a_13*POS_eL2(1,1)+b_13) < POS_eL2(2,1)  &&  (a_24*POS_eL2(1,1)+b_24) <= POS_eL2(2,1)
     contact_flag_L2(4,1) = 111;   % 領域4,1
 else
     contact_flag_L2(5,5) = 111;   % 非接触
 end
% R1
 if     shikaku1_4(1,1) <= POS_eR1(1,1)  &&  shikaku1_2(2,1) <= POS_eR1(2,1)  &&  POS_eR1(1,1) <= shikaku1(1,1)  &&  POS_eR1(2,1) <= shikaku1(2,1)  &&  sqrt( (POS_eR1(1,1)-shikaku1(1,1))^2 + (POS_eR1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R1(1,1) = 111;   % 領域1,1
 elseif shikaku2(1,1) <= POS_eR1(1,1)  &&  shikaku2_1(2,1) <= POS_eR1(2,1)  &&  POS_eR1(1,1) <= shikaku2_3(1,1)  &&  POS_eR1(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_eR1(1,1)-shikaku2(1,1))^2 + (POS_eR1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R1(2,2) = 111;   % 領域2,2
 elseif shikaku3(1,1) <= POS_eR1(1,1)  &&  shikaku3(2,1) <= POS_eR1(2,1)  &&  POS_eR1(1,1) <= shikaku3_2(1,1)  &&  POS_eR1(2,1) <= shikaku3_4(2,1)  &&  sqrt( (POS_eR1(1,1)-shikaku3(1,1))^2 + (POS_eR1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R1(3,3) = 111;   % 領域3,3
 elseif shikaku4_1(1,1) <= POS_eR1(1,1)  &&  shikaku4(2,1) <= POS_eR1(2,1)  &&  POS_eR1(1,1) <= shikaku4(1,1)  &&  POS_eR1(2,1) <= shikaku4_3(2,1)  &&  sqrt( (POS_eR1(1,1)-shikaku4(1,1))^2 + (POS_eR1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R1(4,4) = 111;   % 領域4,4
 elseif shikaku1(1,1) < POS_eR1(1,1)  &&  shikaku1_2(2,1) <= POS_eR1(2,1)  &&  POS_eR1(1,1) < shikaku2(1,1)  &&  POS_eR1(2,1) <= (a_13*POS_eR1(1,1)+b_13)  &&  POS_eR1(2,1) < (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(1,2) = 111;   % 領域1,2
 elseif shikaku2(2,1) < POS_eR1(2,1)  &&  POS_eR1(1,1) <= shikaku2_3(1,1)  &&  POS_eR1(2,1) < shikaku3(2,1)  &&  POS_eR1(2,1) < (a_13*POS_eR1(1,1)+b_13)  &&  (a_24*POS_eR1(1,1)+b_24) <= POS_eR1(2,1)
     contact_flag_R1(2,3) = 111;   % 領域2,3
 elseif shikaku4(1,1) < POS_eR1(1,1)  &&  POS_eR1(1,1) < shikaku3(1,1)  &&  POS_eR1(2,1) <= shikaku4_3(2,1)  &&  (a_13*POS_eR1(1,1)+b_13) <= POS_eR1(2,1)  &&  (a_24*POS_eR1(1,1)+b_24) < POS_eR1(2,1)
     contact_flag_R1(3,4) = 111;   % 領域3,4
 elseif POS_eR1(1,1) <= shikaku1_4(1,1)  &&  shikaku1(2,1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < shikaku4(2,1)  &&  (a_13*POS_eR1(1,1)+b_13) < POS_eR1(2,1)  &&  (a_24*POS_eR1(1,1)+b_24) <= POS_eR1(2,1)
     contact_flag_R1(4,1) = 111;   % 領域4,1
 else
     contact_flag_R1(5,5) = 111;   % 非接触
 end
% R2
 if     shikaku1_4(1,1) <= POS_eR2(1,1)  &&  shikaku1_2(2,1) <= POS_eR2(2,1)  &&  POS_eR2(1,1) <= shikaku1(1,1)  &&  POS_eR2(2,1) <= shikaku1(2,1)  &&  sqrt( (POS_eR2(1,1)-shikaku1(1,1))^2 + (POS_eR2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R2(1,1) = 111;   % 領域1,1
 elseif shikaku2(1,1) <= POS_eR2(1,1)  &&  shikaku2_1(2,1) <= POS_eR2(2,1)  &&  POS_eR2(1,1) <= shikaku2_3(1,1)  &&  POS_eR2(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_eR2(1,1)-shikaku2(1,1))^2 + (POS_eR2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R2(2,2) = 111;   % 領域2,2
 elseif shikaku3(1,1) <= POS_eR2(1,1)  &&  shikaku3(2,1) <= POS_eR2(2,1)  &&  POS_eR2(1,1) <= shikaku3_2(1,1)  &&  POS_eR2(2,1) <= shikaku3_4(2,1)  &&  sqrt( (POS_eR2(1,1)-shikaku3(1,1))^2 + (POS_eR2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R2(3,3) = 111;   % 領域3,3
 elseif shikaku4_1(1,1) <= POS_eR2(1,1)  &&  shikaku4(2,1) <= POS_eR2(2,1)  &&  POS_eR2(1,1) <= shikaku4(1,1)  &&  POS_eR2(2,1) <= shikaku4_3(2,1)  &&  sqrt( (POS_eR2(1,1)-shikaku4(1,1))^2 + (POS_eR2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R2(4,4) = 111;   % 領域4,4
 elseif shikaku1(1,1) < POS_eR2(1,1)  &&  shikaku1_2(2,1) <= POS_eR2(2,1)  &&  POS_eR2(1,1) < shikaku2(1,1)  &&  POS_eR2(2,1) <= (a_13*POS_eR2(1,1)+b_13)  &&  POS_eR2(2,1) < (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(1,2) = 111;   % 領域1,2
 elseif shikaku2(2,1) < POS_eR2(2,1)  &&  POS_eR2(1,1) <= shikaku2_3(1,1)  &&  POS_eR2(2,1) < shikaku3(2,1)  &&  POS_eR2(2,1) < (a_13*POS_eR2(1,1)+b_13)  &&  (a_24*POS_eR2(1,1)+b_24) <= POS_eR2(2,1)
     contact_flag_R2(2,3) = 111;   % 領域2,3
 elseif shikaku4(1,1) < POS_eR2(1,1)  &&  POS_eR2(1,1) < shikaku3(1,1)  &&  POS_eR2(2,1) <= shikaku4_3(2,1)  &&  (a_13*POS_eR2(1,1)+b_13) <= POS_eR2(2,1)  &&  (a_24*POS_eR2(1,1)+b_24) < POS_eR2(2,1)
     contact_flag_R2(3,4) = 111;   % 領域3,4
 elseif POS_eR2(1,1) <= shikaku1_4(1,1)  &&  shikaku1(2,1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < shikaku4(2,1)  &&  (a_13*POS_eR2(1,1)+b_13) < POS_eR2(2,1)  &&  (a_24*POS_eR2(1,1)+b_24) <= POS_eR2(2,1)
     contact_flag_R2(4,1) = 111;   % 領域4,1
 else
     contact_flag_R2(5,5) = 111;   % 非接触
 end


%%%%%% ターゲットの姿勢がπ/2+θcのとき
elseif q0 == ( pi/2 + theta_chohou )

a_12_1 = ( shikaku1(2,1) - shikaku2(2,1) ) / ( shikaku1(1,1) - shikaku2(1,1) );
a_12_2 = ( shikaku1_2(2,1) - shikaku2_1(2,1) ) / ( shikaku1_2(1,1) - shikaku2_1(1,1) );
a_23_1 = ( shikaku2(2,1) - shikaku3(2,1) ) / ( shikaku2(1,1) - shikaku3(1,1) );
a_23_2 = ( shikaku2_3(2,1) - shikaku3_2(2,1) ) / ( shikaku2_3(1,1) - shikaku3_2(1,1) );
a_34_1 = ( shikaku3(2,1) - shikaku4(2,1) ) / ( shikaku3(1,1) - shikaku4(1,1) );
a_34_2 = ( shikaku3_4(2,1) - shikaku4_3(2,1) ) / ( shikaku3_4(1,1) - shikaku4_3(1,1) );
a_41_1 = ( shikaku4(2,1) - shikaku1(2,1) ) / ( shikaku4(1,1) - shikaku1(1,1) );
a_41_2 = ( shikaku4_1(2,1) - shikaku1_4(2,1) ) / ( shikaku4_1(1,1) - shikaku1_4(1,1) );
b_12_1 = shikaku1(2,1) - a_12_1*shikaku1(1,1);
b_12_2 = shikaku1_2(2,1) - a_12_2*shikaku1_2(1,1);
b_23_1 = shikaku2(2,1) - a_23_1*shikaku2(1,1);
b_23_2 = shikaku2_3(2,1) - a_23_2*shikaku2_3(1,1);
b_34_1 = shikaku3(2,1) - a_34_1*shikaku3(1,1);
b_34_2 = shikaku3_4(2,1) - a_34_2*shikaku3_4(1,1);
b_41_1 = shikaku4(2,1) - a_41_1*shikaku4(1,1);
b_41_2 = shikaku4_1(2,1) - a_41_2*shikaku4_1(1,1);
a_24 = ( shikaku2(2,1) - shikaku4(2,1) ) / ( shikaku2(1,1) - shikaku4(1,1) );
b_24 = shikaku2(2,1) - a_24*shikaku2(1,1);

% L1
 if     (a_41_2*POS_eL1(1,1)+b_41_2) <= POS_eL1(2,1)  &&  (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_41_1*POS_eL1(1,1)+b_41_1)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  sqrt( (POS_eL1(1,1)-shikaku1(1,1))^2 + (POS_eL1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L1(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eL1(1,1)+b_23_1) <= POS_eL1(2,1)  &&  (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_23_2*POS_eL1(1,1)+b_23_2)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  sqrt( (POS_eL1(1,1)-shikaku2(1,1))^2 + (POS_eL1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L1(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eL1(1,1)+b_23_1) <= POS_eL1(2,1)  &&  (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_23_2*POS_eL1(1,1)+b_23_2)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  sqrt( (POS_eL1(1,1)-shikaku3(1,1))^2 + (POS_eL1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L1(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eL1(1,1)+b_41_2) <= POS_eL1(2,1)  &&  (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_41_1*POS_eL1(1,1)+b_41_1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  sqrt( (POS_eL1(1,1)-shikaku4(1,1))^2 + (POS_eL1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L1(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eL1(1,1)+b_41_1) < POS_eL1(2,1)  &&  (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_23_1*POS_eL1(1,1)+b_23_1)  &&  tc_geo(1,1) <= POS_eL1(1,1)  &&  POS_eL1(2,1) < (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_23_2*POS_eL1(1,1)+b_23_2)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  tc_geo(1,1) < POS_eL1(1,1)  &&  (a_24*POS_eL1(1,1)+b_24) <= POS_eL1(2,1)
     contact_flag_L1(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eL1(1,1)+b_41_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_23_1*POS_eL1(1,1)+b_23_1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(1,1) <= tc_geo(1,1)  &&  (a_24*POS_eL1(1,1)+b_24) < POS_eL1(2,1)
     contact_flag_L1(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eL1(1,1)+b_41_2) <= POS_eL1(2,1)  &&  (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  POS_eL1(1,1) < tc_geo(1,1)  &&  POS_eL1(2,1) <= (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(4,1) = 111;   % 領域4,1
 else
     contact_flag_L1(5,5) = 111;   % 非接触
 end
% L2
 if     (a_41_2*POS_eL2(1,1)+b_41_2) <= POS_eL2(2,1)  &&  (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_41_1*POS_eL2(1,1)+b_41_1)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  sqrt( (POS_eL2(1,1)-shikaku1(1,1))^2 + (POS_eL2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L2(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eL2(1,1)+b_23_1) <= POS_eL2(2,1)  &&  (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_23_2*POS_eL2(1,1)+b_23_2)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  sqrt( (POS_eL2(1,1)-shikaku2(1,1))^2 + (POS_eL2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L2(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eL2(1,1)+b_23_1) <= POS_eL2(2,1)  &&  (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_23_2*POS_eL2(1,1)+b_23_2)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  sqrt( (POS_eL2(1,1)-shikaku3(1,1))^2 + (POS_eL2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L2(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eL2(1,1)+b_41_2) <= POS_eL2(2,1)  &&  (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_41_1*POS_eL2(1,1)+b_41_1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  sqrt( (POS_eL2(1,1)-shikaku4(1,1))^2 + (POS_eL2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L2(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eL2(1,1)+b_41_1) < POS_eL2(2,1)  &&  (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_23_1*POS_eL2(1,1)+b_23_1)  &&  tc_geo(1,1) <= POS_eL2(1,1)  &&  POS_eL2(2,1) < (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_23_2*POS_eL2(1,1)+b_23_2)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  tc_geo(1,1) < POS_eL2(1,1)  &&  (a_24*POS_eL2(1,1)+b_24) <= POS_eL2(2,1)
     contact_flag_L2(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eL2(1,1)+b_41_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_23_1*POS_eL2(1,1)+b_23_1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(1,1) <= tc_geo(1,1)  &&  (a_24*POS_eL2(1,1)+b_24) < POS_eL2(2,1)
     contact_flag_L2(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eL2(1,1)+b_41_2) <= POS_eL2(2,1)  &&  (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  POS_eL2(1,1) < tc_geo(1,1)  &&  POS_eL2(2,1) <= (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(4,1) = 111;   % 領域4,1
 else
     contact_flag_L2(5,5) = 111;   % 非接触
 end
% R1
 if     (a_41_2*POS_eR1(1,1)+b_41_2) <= POS_eR1(2,1)  &&  (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_41_1*POS_eR1(1,1)+b_41_1)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  sqrt( (POS_eR1(1,1)-shikaku1(1,1))^2 + (POS_eR1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R1(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eR1(1,1)+b_23_1) <= POS_eR1(2,1)  &&  (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_23_2*POS_eR1(1,1)+b_23_2)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  sqrt( (POS_eR1(1,1)-shikaku2(1,1))^2 + (POS_eR1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R1(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eR1(1,1)+b_23_1) <= POS_eR1(2,1)  &&  (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_23_2*POS_eR1(1,1)+b_23_2)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  sqrt( (POS_eR1(1,1)-shikaku3(1,1))^2 + (POS_eR1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R1(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eR1(1,1)+b_41_2) <= POS_eR1(2,1)  &&  (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_41_1*POS_eR1(1,1)+b_41_1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  sqrt( (POS_eR1(1,1)-shikaku4(1,1))^2 + (POS_eR1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R1(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eR1(1,1)+b_41_1) < POS_eR1(2,1)  &&  (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_23_1*POS_eR1(1,1)+b_23_1)  &&  tc_geo(1,1) <= POS_eR1(1,1)  &&  POS_eR1(2,1) < (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_23_2*POS_eR1(1,1)+b_23_2)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  tc_geo(1,1) < POS_eR1(1,1)  &&  (a_24*POS_eR1(1,1)+b_24) <= POS_eR1(2,1)
     contact_flag_R1(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eR1(1,1)+b_41_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_23_1*POS_eR1(1,1)+b_23_1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(1,1) <= tc_geo(1,1)  &&  (a_24*POS_eR1(1,1)+b_24) < POS_eR1(2,1)
     contact_flag_R1(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eR1(1,1)+b_41_2) <= POS_eR1(2,1)  &&  (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  POS_eR1(1,1) < tc_geo(1,1)  &&  POS_eR1(2,1) <= (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(4,1) = 111;   % 領域4,1
 else
     contact_flag_R1(5,5) = 111;   % 非接触
 end
% R2
 if     (a_41_2*POS_eR2(1,1)+b_41_2) <= POS_eR2(2,1)  &&  (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_41_1*POS_eR2(1,1)+b_41_1)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  sqrt( (POS_eR2(1,1)-shikaku1(1,1))^2 + (POS_eR2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R2(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eR2(1,1)+b_23_1) <= POS_eR2(2,1)  &&  (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_23_2*POS_eR2(1,1)+b_23_2)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  sqrt( (POS_eR2(1,1)-shikaku2(1,1))^2 + (POS_eR2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R2(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eR2(1,1)+b_23_1) <= POS_eR2(2,1)  &&  (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_23_2*POS_eR2(1,1)+b_23_2)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  sqrt( (POS_eR2(1,1)-shikaku3(1,1))^2 + (POS_eR2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R2(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eR2(1,1)+b_41_2) <= POS_eR2(2,1)  &&  (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_41_1*POS_eR2(1,1)+b_41_1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  sqrt( (POS_eR2(1,1)-shikaku4(1,1))^2 + (POS_eR2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R2(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eR2(1,1)+b_41_1) < POS_eR2(2,1)  &&  (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_23_1*POS_eR2(1,1)+b_23_1)  &&  tc_geo(1,1) <= POS_eR2(1,1)  &&  POS_eR2(2,1) < (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_23_2*POS_eR2(1,1)+b_23_2)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  tc_geo(1,1) < POS_eR2(1,1)  &&  (a_24*POS_eR2(1,1)+b_24) <= POS_eR2(2,1)
     contact_flag_R2(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eR2(1,1)+b_41_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_23_1*POS_eR2(1,1)+b_23_1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(1,1) <= tc_geo(1,1)  &&  (a_24*POS_eR2(1,1)+b_24) < POS_eR2(2,1)
     contact_flag_R2(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eR2(1,1)+b_41_2) <= POS_eR2(2,1)  &&  (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  POS_eR2(1,1) < tc_geo(1,1)  &&  POS_eR2(2,1) <= (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(4,1) = 111;   % 領域4,1
 else
     contact_flag_R2(5,5) = 111;   % 非接触
 end


%%%%%% ターゲットの姿勢(q0)が0〜π/2-θcのとき
elseif 0 < q0  &&  q0 < ( pi/2 - theta_chohou )

a_12_1 = ( shikaku1(2,1) - shikaku2(2,1) ) / ( shikaku1(1,1) - shikaku2(1,1) );
a_12_2 = ( shikaku1_2(2,1) - shikaku2_1(2,1) ) / ( shikaku1_2(1,1) - shikaku2_1(1,1) );
a_23_1 = ( shikaku2(2,1) - shikaku3(2,1) ) / ( shikaku2(1,1) - shikaku3(1,1) );
a_23_2 = ( shikaku2_3(2,1) - shikaku3_2(2,1) ) / ( shikaku2_3(1,1) - shikaku3_2(1,1) );
a_34_1 = ( shikaku3(2,1) - shikaku4(2,1) ) / ( shikaku3(1,1) - shikaku4(1,1) );
a_34_2 = ( shikaku3_4(2,1) - shikaku4_3(2,1) ) / ( shikaku3_4(1,1) - shikaku4_3(1,1) );
a_41_1 = ( shikaku4(2,1) - shikaku1(2,1) ) / ( shikaku4(1,1) - shikaku1(1,1) );
a_41_2 = ( shikaku4_1(2,1) - shikaku1_4(2,1) ) / ( shikaku4_1(1,1) - shikaku1_4(1,1) );
b_12_1 = shikaku1(2,1) - a_12_1*shikaku1(1,1);
b_12_2 = shikaku1_2(2,1) - a_12_2*shikaku1_2(1,1);
b_23_1 = shikaku2(2,1) - a_23_1*shikaku2(1,1);
b_23_2 = shikaku2_3(2,1) - a_23_2*shikaku2_3(1,1);
b_34_1 = shikaku3(2,1) - a_34_1*shikaku3(1,1);
b_34_2 = shikaku3_4(2,1) - a_34_2*shikaku3_4(1,1);
b_41_1 = shikaku4(2,1) - a_41_1*shikaku4(1,1);
b_41_2 = shikaku4_1(2,1) - a_41_2*shikaku4_1(1,1);
a_13 = ( shikaku1(2,1) - shikaku3(2,1) ) / ( shikaku1(1,1) - shikaku3(1,1) );
a_24 = ( shikaku2(2,1) - shikaku4(2,1) ) / ( shikaku2(1,1) - shikaku4(1,1) );
b_13 = shikaku1(2,1) - a_13*shikaku1(1,1);
b_24 = shikaku2(2,1) - a_24*shikaku2(1,1);

% L1
 if     (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  (a_41_1*POS_eL1(1,1)+b_41_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  POS_eL1(2,1) <= (a_41_2*POS_eL1(1,1)+b_41_2)  &&  sqrt( (POS_eL1(1,1)-shikaku1(1,1))^2 + (POS_eL1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L1(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  (a_23_2*POS_eL1(1,1)+b_23_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  POS_eL1(2,1) <= (a_23_1*POS_eL1(1,1)+b_23_1)  &&  sqrt( (POS_eL1(1,1)-shikaku2(1,1))^2 + (POS_eL1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L1(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  (a_23_2*POS_eL1(1,1)+b_23_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(2,1) <= (a_23_1*POS_eL1(1,1)+b_23_1)  &&  sqrt( (POS_eL1(1,1)-shikaku3(1,1))^2 + (POS_eL1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L1(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  (a_41_1*POS_eL1(1,1)+b_41_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(2,1) <= (a_41_2*POS_eL1(1,1)+b_41_2)  &&  sqrt( (POS_eL1(1,1)-shikaku4(1,1))^2 + (POS_eL1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L1(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  (a_23_1*POS_eL1(1,1)+b_23_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_41_1*POS_eL1(1,1)+b_41_1)  &&  POS_eL1(2,1) <= (a_13*POS_eL1(1,1)+b_13)  &&  (a_24*POS_eL1(1,1)+b_24) < POS_eL1(2,1)
     contact_flag_L1(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eL1(1,1)+b_23_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_24*POS_eL1(1,1)+b_24)  &&  POS_eL1(2,1) < (a_13*POS_eL1(1,1)+b_13)
     contact_flag_L1(2,3) = 111;   % 領域2,3
 elseif POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(2,1) < (a_41_1*POS_eL1(1,1)+b_41_1)  &&  (a_23_1*POS_eL1(1,1)+b_23_1) < POS_eL1(2,1)  &&  (a_13*POS_eL1(1,1)+b_13) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(3,4) = 111;   % 領域3,4
 elseif POS_eL1(2,1) <= (a_41_2*POS_eL1(1,1)+b_41_2)  &&  (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  (a_24*POS_eL1(1,1)+b_24) <= POS_eL1(2,1)  &&  (a_13*POS_eL1(1,1)+b_13) < POS_eL1(2,1)
     contact_flag_L1(4,1) = 111;   % 領域4,1
 else
     contact_flag_L1(5,5) = 111;   % 非接触
 end
% L2
 if     (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  (a_41_1*POS_eL2(1,1)+b_41_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  POS_eL2(2,1) <= (a_41_2*POS_eL2(1,1)+b_41_2)  &&  sqrt( (POS_eL2(1,1)-shikaku1(1,1))^2 + (POS_eL2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L2(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  (a_23_2*POS_eL2(1,1)+b_23_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  POS_eL2(2,1) <= (a_23_1*POS_eL2(1,1)+b_23_1)  &&  sqrt( (POS_eL2(1,1)-shikaku2(1,1))^2 + (POS_eL2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L2(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  (a_23_2*POS_eL2(1,1)+b_23_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(2,1) <= (a_23_1*POS_eL2(1,1)+b_23_1)  &&  sqrt( (POS_eL2(1,1)-shikaku3(1,1))^2 + (POS_eL2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L2(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  (a_41_1*POS_eL2(1,1)+b_41_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(2,1) <= (a_41_2*POS_eL2(1,1)+b_41_2)  &&  sqrt( (POS_eL2(1,1)-shikaku4(1,1))^2 + (POS_eL2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L2(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  (a_23_1*POS_eL2(1,1)+b_23_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_41_1*POS_eL2(1,1)+b_41_1)  &&  POS_eL2(2,1) <= (a_13*POS_eL2(1,1)+b_13)  &&  (a_24*POS_eL2(1,1)+b_24) < POS_eL2(2,1)
     contact_flag_L2(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eL2(1,1)+b_23_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_24*POS_eL2(1,1)+b_24)  &&  POS_eL2(2,1) < (a_13*POS_eL2(1,1)+b_13)
     contact_flag_L2(2,3) = 111;   % 領域2,3
 elseif POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(2,1) < (a_41_1*POS_eL2(1,1)+b_41_1)  &&  (a_23_1*POS_eL2(1,1)+b_23_1) < POS_eL2(2,1)  &&  (a_13*POS_eL2(1,1)+b_13) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(3,4) = 111;   % 領域3,4
 elseif POS_eL2(2,1) <= (a_41_2*POS_eL2(1,1)+b_41_2)  &&  (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  (a_24*POS_eL2(1,1)+b_24) <= POS_eL2(2,1)  &&  (a_13*POS_eL2(1,1)+b_13) < POS_eL2(2,1)
     contact_flag_L2(4,1) = 111;   % 領域4,1
 else
     contact_flag_L2(5,5) = 111;   % 非接触
 end
% R1
 if     (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  (a_41_1*POS_eR1(1,1)+b_41_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  POS_eR1(2,1) <= (a_41_2*POS_eR1(1,1)+b_41_2)  &&  sqrt( (POS_eR1(1,1)-shikaku1(1,1))^2 + (POS_eR1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R1(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  (a_23_2*POS_eR1(1,1)+b_23_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  POS_eR1(2,1) <= (a_23_1*POS_eR1(1,1)+b_23_1)  &&  sqrt( (POS_eR1(1,1)-shikaku2(1,1))^2 + (POS_eR1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R1(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  (a_23_2*POS_eR1(1,1)+b_23_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(2,1) <= (a_23_1*POS_eR1(1,1)+b_23_1)  &&  sqrt( (POS_eR1(1,1)-shikaku3(1,1))^2 + (POS_eR1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R1(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  (a_41_1*POS_eR1(1,1)+b_41_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(2,1) <= (a_41_2*POS_eR1(1,1)+b_41_2)  &&  sqrt( (POS_eR1(1,1)-shikaku4(1,1))^2 + (POS_eR1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R1(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  (a_23_1*POS_eR1(1,1)+b_23_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_41_1*POS_eR1(1,1)+b_41_1)  &&  POS_eR1(2,1) <= (a_13*POS_eR1(1,1)+b_13)  &&  (a_24*POS_eR1(1,1)+b_24) < POS_eR1(2,1)
     contact_flag_R1(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eR1(1,1)+b_23_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_24*POS_eR1(1,1)+b_24)  &&  POS_eR1(2,1) < (a_13*POS_eR1(1,1)+b_13)
     contact_flag_R1(2,3) = 111;   % 領域2,3
 elseif POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(2,1) < (a_41_1*POS_eR1(1,1)+b_41_1)  &&  (a_23_1*POS_eR1(1,1)+b_23_1) < POS_eR1(2,1)  &&  (a_13*POS_eR1(1,1)+b_13) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(3,4) = 111;   % 領域3,4
 elseif POS_eR1(2,1) <= (a_41_2*POS_eR1(1,1)+b_41_2)  &&  (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  (a_24*POS_eR1(1,1)+b_24) <= POS_eR1(2,1)  &&  (a_13*POS_eR1(1,1)+b_13) < POS_eR1(2,1)
     contact_flag_R1(4,1) = 111;   % 領域4,1
 else
     contact_flag_R1(5,5) = 111;   % 非接触
 end
% R2
 if     (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  (a_41_1*POS_eR2(1,1)+b_41_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  POS_eR2(2,1) <= (a_41_2*POS_eR2(1,1)+b_41_2)  &&  sqrt( (POS_eR2(1,1)-shikaku1(1,1))^2 + (POS_eR2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R2(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  (a_23_2*POS_eR2(1,1)+b_23_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  POS_eR2(2,1) <= (a_23_1*POS_eR2(1,1)+b_23_1)  &&  sqrt( (POS_eR2(1,1)-shikaku2(1,1))^2 + (POS_eR2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R2(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  (a_23_2*POS_eR2(1,1)+b_23_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(2,1) <= (a_23_1*POS_eR2(1,1)+b_23_1)  &&  sqrt( (POS_eR2(1,1)-shikaku3(1,1))^2 + (POS_eR2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R2(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  (a_41_1*POS_eR2(1,1)+b_41_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(2,1) <= (a_41_2*POS_eR2(1,1)+b_41_2)  &&  sqrt( (POS_eR2(1,1)-shikaku4(1,1))^2 + (POS_eR2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R2(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  (a_23_1*POS_eR2(1,1)+b_23_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_41_1*POS_eR2(1,1)+b_41_1)  &&  POS_eR2(2,1) <= (a_13*POS_eR2(1,1)+b_13)  &&  (a_24*POS_eR2(1,1)+b_24) < POS_eR2(2,1)
     contact_flag_R2(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eR2(1,1)+b_23_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_24*POS_eR2(1,1)+b_24)  &&  POS_eR2(2,1) < (a_13*POS_eR2(1,1)+b_13)
     contact_flag_R2(2,3) = 111;   % 領域2,3
 elseif POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(2,1) < (a_41_1*POS_eR2(1,1)+b_41_1)  &&  (a_23_1*POS_eR2(1,1)+b_23_1) < POS_eR2(2,1)  &&  (a_13*POS_eR2(1,1)+b_13) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(3,4) = 111;   % 領域3,4
 elseif POS_eR2(2,1) <= (a_41_2*POS_eR2(1,1)+b_41_2)  &&  (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  (a_24*POS_eR2(1,1)+b_24) <= POS_eR2(2,1)  &&  (a_13*POS_eR2(1,1)+b_13) < POS_eR2(2,1)
     contact_flag_R2(4,1) = 111;   % 領域4,1
 else
     contact_flag_R2(5,5) = 111;   % 非接触
 end


%%%%%% ターゲットの姿勢(q0)がπ/2-θc〜π/2のとき
elseif ( pi/2 - theta_chohou ) < q0  &&  q0 < pi/2

a_12_1 = ( shikaku1(2,1) - shikaku2(2,1) ) / ( shikaku1(1,1) - shikaku2(1,1) );
a_12_2 = ( shikaku1_2(2,1) - shikaku2_1(2,1) ) / ( shikaku1_2(1,1) - shikaku2_1(1,1) );
a_23_1 = ( shikaku2(2,1) - shikaku3(2,1) ) / ( shikaku2(1,1) - shikaku3(1,1) );
a_23_2 = ( shikaku2_3(2,1) - shikaku3_2(2,1) ) / ( shikaku2_3(1,1) - shikaku3_2(1,1) );
a_34_1 = ( shikaku3(2,1) - shikaku4(2,1) ) / ( shikaku3(1,1) - shikaku4(1,1) );
a_34_2 = ( shikaku3_4(2,1) - shikaku4_3(2,1) ) / ( shikaku3_4(1,1) - shikaku4_3(1,1) );
a_41_1 = ( shikaku4(2,1) - shikaku1(2,1) ) / ( shikaku4(1,1) - shikaku1(1,1) );
a_41_2 = ( shikaku4_1(2,1) - shikaku1_4(2,1) ) / ( shikaku4_1(1,1) - shikaku1_4(1,1) );
b_12_1 = shikaku1(2,1) - a_12_1*shikaku1(1,1);
b_12_2 = shikaku1_2(2,1) - a_12_2*shikaku1_2(1,1);
b_23_1 = shikaku2(2,1) - a_23_1*shikaku2(1,1);
b_23_2 = shikaku2_3(2,1) - a_23_2*shikaku2_3(1,1);
b_34_1 = shikaku3(2,1) - a_34_1*shikaku3(1,1);
b_34_2 = shikaku3_4(2,1) - a_34_2*shikaku3_4(1,1);
b_41_1 = shikaku4(2,1) - a_41_1*shikaku4(1,1);
b_41_2 = shikaku4_1(2,1) - a_41_2*shikaku4_1(1,1);
a_13 = ( shikaku1(2,1) - shikaku3(2,1) ) / ( shikaku1(1,1) - shikaku3(1,1) );
a_24 = ( shikaku2(2,1) - shikaku4(2,1) ) / ( shikaku2(1,1) - shikaku4(1,1) );
b_13 = shikaku1(2,1) - a_13*shikaku1(1,1);
b_24 = shikaku2(2,1) - a_24*shikaku2(1,1);

% L1
 if     (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  (a_41_1*POS_eL1(1,1)+b_41_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  POS_eL1(2,1) <= (a_41_2*POS_eL1(1,1)+b_41_2)  &&  sqrt( (POS_eL1(1,1)-shikaku1(1,1))^2 + (POS_eL1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L1(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  (a_23_2*POS_eL1(1,1)+b_23_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  POS_eL1(2,1) <= (a_23_1*POS_eL1(1,1)+b_23_1)  &&  sqrt( (POS_eL1(1,1)-shikaku2(1,1))^2 + (POS_eL1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L1(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  (a_23_2*POS_eL1(1,1)+b_23_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(2,1) <= (a_23_1*POS_eL1(1,1)+b_23_1)  &&  sqrt( (POS_eL1(1,1)-shikaku3(1,1))^2 + (POS_eL1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L1(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  (a_41_1*POS_eL1(1,1)+b_41_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(2,1) <= (a_41_2*POS_eL1(1,1)+b_41_2)  &&  sqrt( (POS_eL1(1,1)-shikaku4(1,1))^2 + (POS_eL1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L1(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  (a_23_1*POS_eL1(1,1)+b_23_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_41_1*POS_eL1(1,1)+b_41_1)  &&  POS_eL1(2,1) <= (a_13*POS_eL1(1,1)+b_13)  &&  POS_eL1(2,1) < (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eL1(1,1)+b_23_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  (a_24*POS_eL1(1,1)+b_24) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_13*POS_eL1(1,1)+b_13)
     contact_flag_L1(2,3) = 111;   % 領域2,3
 elseif POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(2,1) < (a_41_1*POS_eL1(1,1)+b_41_1)  &&  (a_23_1*POS_eL1(1,1)+b_23_1) < POS_eL1(2,1)  &&  (a_13*POS_eL1(1,1)+b_13) <= POS_eL1(2,1)  &&  (a_24*POS_eL1(1,1)+b_24) < POS_eL1(2,1)
     contact_flag_L1(3,4) = 111;   % 領域3,4
 elseif POS_eL1(2,1) <= (a_41_2*POS_eL1(1,1)+b_41_2)  &&  (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  POS_eL1(2,1) <= (a_24*POS_eL1(1,1)+b_24)  &&  (a_13*POS_eL1(1,1)+b_13) < POS_eL1(2,1)
     contact_flag_L1(4,1) = 111;   % 領域4,1
 else
     contact_flag_L1(5,5) = 111;   % 非接触
 end
% L2
 if     (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  (a_41_1*POS_eL2(1,1)+b_41_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  POS_eL2(2,1) <= (a_41_2*POS_eL2(1,1)+b_41_2)  &&  sqrt( (POS_eL2(1,1)-shikaku1(1,1))^2 + (POS_eL2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L2(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  (a_23_2*POS_eL2(1,1)+b_23_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  POS_eL2(2,1) <= (a_23_1*POS_eL2(1,1)+b_23_1)  &&  sqrt( (POS_eL2(1,1)-shikaku2(1,1))^2 + (POS_eL2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L2(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  (a_23_2*POS_eL2(1,1)+b_23_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(2,1) <= (a_23_1*POS_eL2(1,1)+b_23_1)  &&  sqrt( (POS_eL2(1,1)-shikaku3(1,1))^2 + (POS_eL2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L2(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  (a_41_1*POS_eL2(1,1)+b_41_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(2,1) <= (a_41_2*POS_eL2(1,1)+b_41_2)  &&  sqrt( (POS_eL2(1,1)-shikaku4(1,1))^2 + (POS_eL2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L2(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  (a_23_1*POS_eL2(1,1)+b_23_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_41_1*POS_eL2(1,1)+b_41_1)  &&  POS_eL2(2,1) <= (a_13*POS_eL2(1,1)+b_13)  &&  POS_eL2(2,1) < (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eL2(1,1)+b_23_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  (a_24*POS_eL2(1,1)+b_24) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_13*POS_eL2(1,1)+b_13)
     contact_flag_L2(2,3) = 111;   % 領域2,3
 elseif POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(2,1) < (a_41_1*POS_eL2(1,1)+b_41_1)  &&  (a_23_1*POS_eL2(1,1)+b_23_1) < POS_eL2(2,1)  &&  (a_13*POS_eL2(1,1)+b_13) <= POS_eL2(2,1)  &&  (a_24*POS_eL2(1,1)+b_24) < POS_eL2(2,1)
     contact_flag_L2(3,4) = 111;   % 領域3,4
 elseif POS_eL2(2,1) <= (a_41_2*POS_eL2(1,1)+b_41_2)  &&  (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  POS_eL2(2,1) <= (a_24*POS_eL2(1,1)+b_24)  &&  (a_13*POS_eL2(1,1)+b_13) < POS_eL2(2,1)
     contact_flag_L2(4,1) = 111;   % 領域4,1
 else
     contact_flag_L2(5,5) = 111;   % 非接触
 end
% R1
 if     (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  (a_41_1*POS_eR1(1,1)+b_41_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  POS_eR1(2,1) <= (a_41_2*POS_eR1(1,1)+b_41_2)  &&  sqrt( (POS_eR1(1,1)-shikaku1(1,1))^2 + (POS_eR1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R1(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  (a_23_2*POS_eR1(1,1)+b_23_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  POS_eR1(2,1) <= (a_23_1*POS_eR1(1,1)+b_23_1)  &&  sqrt( (POS_eR1(1,1)-shikaku2(1,1))^2 + (POS_eR1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R1(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  (a_23_2*POS_eR1(1,1)+b_23_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(2,1) <= (a_23_1*POS_eR1(1,1)+b_23_1)  &&  sqrt( (POS_eR1(1,1)-shikaku3(1,1))^2 + (POS_eR1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R1(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  (a_41_1*POS_eR1(1,1)+b_41_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(2,1) <= (a_41_2*POS_eR1(1,1)+b_41_2)  &&  sqrt( (POS_eR1(1,1)-shikaku4(1,1))^2 + (POS_eR1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R1(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  (a_23_1*POS_eR1(1,1)+b_23_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_41_1*POS_eR1(1,1)+b_41_1)  &&  POS_eR1(2,1) <= (a_13*POS_eR1(1,1)+b_13)  &&  POS_eR1(2,1) < (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eR1(1,1)+b_23_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  (a_24*POS_eR1(1,1)+b_24) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_13*POS_eR1(1,1)+b_13)
     contact_flag_R1(2,3) = 111;   % 領域2,3
 elseif POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(2,1) < (a_41_1*POS_eR1(1,1)+b_41_1)  &&  (a_23_1*POS_eR1(1,1)+b_23_1) < POS_eR1(2,1)  &&  (a_13*POS_eR1(1,1)+b_13) <= POS_eR1(2,1)  &&  (a_24*POS_eR1(1,1)+b_24) < POS_eR1(2,1)
     contact_flag_R1(3,4) = 111;   % 領域3,4
 elseif POS_eR1(2,1) <= (a_41_2*POS_eR1(1,1)+b_41_2)  &&  (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  POS_eR1(2,1) <= (a_24*POS_eR1(1,1)+b_24)  &&  (a_13*POS_eR1(1,1)+b_13) < POS_eR1(2,1)
     contact_flag_R1(4,1) = 111;   % 領域4,1
 else
     contact_flag_R1(5,5) = 111;   % 非接触
 end
% R2
 if     (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  (a_41_1*POS_eR2(1,1)+b_41_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  POS_eR2(2,1) <= (a_41_2*POS_eR2(1,1)+b_41_2)  &&  sqrt( (POS_eR2(1,1)-shikaku1(1,1))^2 + (POS_eR2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R2(1,1) = 111;   % 領域1,1
 elseif (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  (a_23_2*POS_eR2(1,1)+b_23_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  POS_eR2(2,1) <= (a_23_1*POS_eR2(1,1)+b_23_1)  &&  sqrt( (POS_eR2(1,1)-shikaku2(1,1))^2 + (POS_eR2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R2(2,2) = 111;   % 領域2,2
 elseif (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  (a_23_2*POS_eR2(1,1)+b_23_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(2,1) <= (a_23_1*POS_eR2(1,1)+b_23_1)  &&  sqrt( (POS_eR2(1,1)-shikaku3(1,1))^2 + (POS_eR2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R2(3,3) = 111;   % 領域3,3
 elseif (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  (a_41_1*POS_eR2(1,1)+b_41_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(2,1) <= (a_41_2*POS_eR2(1,1)+b_41_2)  &&  sqrt( (POS_eR2(1,1)-shikaku4(1,1))^2 + (POS_eR2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R2(4,4) = 111;   % 領域4,4
 elseif (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  (a_23_1*POS_eR2(1,1)+b_23_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_41_1*POS_eR2(1,1)+b_41_1)  &&  POS_eR2(2,1) <= (a_13*POS_eR2(1,1)+b_13)  &&  POS_eR2(2,1) < (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(1,2) = 111;   % 領域1,2
 elseif (a_23_2*POS_eR2(1,1)+b_23_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  (a_24*POS_eR2(1,1)+b_24) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_13*POS_eR2(1,1)+b_13)
     contact_flag_R2(2,3) = 111;   % 領域2,3
 elseif POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(2,1) < (a_41_1*POS_eR2(1,1)+b_41_1)  &&  (a_23_1*POS_eR2(1,1)+b_23_1) < POS_eR2(2,1)  &&  (a_13*POS_eR2(1,1)+b_13) <= POS_eR2(2,1)  &&  (a_24*POS_eR2(1,1)+b_24) < POS_eR2(2,1)
     contact_flag_R2(3,4) = 111;   % 領域3,4
 elseif POS_eR2(2,1) <= (a_41_2*POS_eR2(1,1)+b_41_2)  &&  (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  POS_eR2(2,1) <= (a_24*POS_eR2(1,1)+b_24)  &&  (a_13*POS_eR2(1,1)+b_13) < POS_eR2(2,1)
     contact_flag_R2(4,1) = 111;   % 領域4,1
 else
     contact_flag_R2(5,5) = 111;   % 非接触
 end


%%%%%% ターゲットの姿勢(q0)がπ/2-θc〜π/2のとき
elseif pi/2 < q0  &&  q0 < ( pi/2 + theta_chohou )

a_12_1 = ( shikaku1(2,1) - shikaku2(2,1) ) / ( shikaku1(1,1) - shikaku2(1,1) );
a_12_2 = ( shikaku1_2(2,1) - shikaku2_1(2,1) ) / ( shikaku1_2(1,1) - shikaku2_1(1,1) );
a_23_1 = ( shikaku2(2,1) - shikaku3(2,1) ) / ( shikaku2(1,1) - shikaku3(1,1) );
a_23_2 = ( shikaku2_3(2,1) - shikaku3_2(2,1) ) / ( shikaku2_3(1,1) - shikaku3_2(1,1) );
a_34_1 = ( shikaku3(2,1) - shikaku4(2,1) ) / ( shikaku3(1,1) - shikaku4(1,1) );
a_34_2 = ( shikaku3_4(2,1) - shikaku4_3(2,1) ) / ( shikaku3_4(1,1) - shikaku4_3(1,1) );
a_41_1 = ( shikaku4(2,1) - shikaku1(2,1) ) / ( shikaku4(1,1) - shikaku1(1,1) );
a_41_2 = ( shikaku4_1(2,1) - shikaku1_4(2,1) ) / ( shikaku4_1(1,1) - shikaku1_4(1,1) );
b_12_1 = shikaku1(2,1) - a_12_1*shikaku1(1,1);
b_12_2 = shikaku1_2(2,1) - a_12_2*shikaku1_2(1,1);
b_23_1 = shikaku2(2,1) - a_23_1*shikaku2(1,1);
b_23_2 = shikaku2_3(2,1) - a_23_2*shikaku2_3(1,1);
b_34_1 = shikaku3(2,1) - a_34_1*shikaku3(1,1);
b_34_2 = shikaku3_4(2,1) - a_34_2*shikaku3_4(1,1);
b_41_1 = shikaku4(2,1) - a_41_1*shikaku4(1,1);
b_41_2 = shikaku4_1(2,1) - a_41_2*shikaku4_1(1,1);
a_13 = ( shikaku1(2,1) - shikaku3(2,1) ) / ( shikaku1(1,1) - shikaku3(1,1) );
a_24 = ( shikaku2(2,1) - shikaku4(2,1) ) / ( shikaku2(1,1) - shikaku4(1,1) );
b_13 = shikaku1(2,1) - a_13*shikaku1(1,1);
b_24 = shikaku2(2,1) - a_24*shikaku2(1,1);

% L1
 if     (a_41_2*POS_eL1(1,1)+b_41_2) <= POS_eL1(2,1)  &&  (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_41_1*POS_eL1(1,1)+b_41_1)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  sqrt( (POS_eL1(1,1)-shikaku1(1,1))^2 + (POS_eL1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L1(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eL1(1,1)+b_23_1) <= POS_eL1(2,1)  &&  (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_23_2*POS_eL1(1,1)+b_23_2)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  sqrt( (POS_eL1(1,1)-shikaku2(1,1))^2 + (POS_eL1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L1(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eL1(1,1)+b_23_1) <= POS_eL1(2,1)  &&  (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_23_2*POS_eL1(1,1)+b_23_2)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  sqrt( (POS_eL1(1,1)-shikaku3(1,1))^2 + (POS_eL1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L1(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eL1(1,1)+b_41_2) <= POS_eL1(2,1)  &&  (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_41_1*POS_eL1(1,1)+b_41_1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&   sqrt( (POS_eL1(1,1)-shikaku4(1,1))^2 + (POS_eL1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L1(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eL1(1,1)+b_41_1) < POS_eL1(2,1)  &&  (a_12_2*POS_eL1(1,1)+b_12_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_23_1*POS_eL1(1,1)+b_23_1)  &&  POS_eL1(2,1) <= (a_13*POS_eL1(1,1)+b_13)  &&  POS_eL1(2,1) < (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_23_2*POS_eL1(1,1)+b_23_2)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  POS_eL1(2,1) < (a_13*POS_eL1(1,1)+b_13)  &&  (a_24*POS_eL1(1,1)+b_24) <= POS_eL1(2,1)
     contact_flag_L1(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eL1(1,1)+b_41_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_23_1*POS_eL1(1,1)+b_23_1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  (a_13*POS_eL1(1,1)+b_13) <= POS_eL1(2,1)  &&  (a_24*POS_eL1(1,1)+b_24) < POS_eL1(2,1)
     contact_flag_L1(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eL1(1,1)+b_41_2) <= POS_eL1(2,1)  &&  (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  (a_13*POS_eL1(1,1)+b_13) < POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(4,1) = 111;   % 領域4,1
 else
     contact_flag_L1(5,5) = 111;   % 非接触
 end
% L2
 if     (a_41_2*POS_eL2(1,1)+b_41_2) <= POS_eL2(2,1)  &&  (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_41_1*POS_eL2(1,1)+b_41_1)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  sqrt( (POS_eL2(1,1)-shikaku1(1,1))^2 + (POS_eL2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L2(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eL2(1,1)+b_23_1) <= POS_eL2(2,1)  &&  (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_23_2*POS_eL2(1,1)+b_23_2)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  sqrt( (POS_eL2(1,1)-shikaku2(1,1))^2 + (POS_eL2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L2(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eL2(1,1)+b_23_1) <= POS_eL2(2,1)  &&  (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_23_2*POS_eL2(1,1)+b_23_2)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  sqrt( (POS_eL2(1,1)-shikaku3(1,1))^2 + (POS_eL2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L2(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eL2(1,1)+b_41_2) <= POS_eL2(2,1)  &&  (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_41_1*POS_eL2(1,1)+b_41_1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&   sqrt( (POS_eL2(1,1)-shikaku4(1,1))^2 + (POS_eL2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L2(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eL2(1,1)+b_41_1) < POS_eL2(2,1)  &&  (a_12_2*POS_eL2(1,1)+b_12_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_23_1*POS_eL2(1,1)+b_23_1)  &&  POS_eL2(2,1) <= (a_13*POS_eL2(1,1)+b_13)  &&  POS_eL2(2,1) < (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_23_2*POS_eL2(1,1)+b_23_2)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  POS_eL2(2,1) < (a_13*POS_eL2(1,1)+b_13)  &&  (a_24*POS_eL2(1,1)+b_24) <= POS_eL2(2,1)
     contact_flag_L2(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eL2(1,1)+b_41_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_23_1*POS_eL2(1,1)+b_23_1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  (a_13*POS_eL2(1,1)+b_13) <= POS_eL2(2,1)  &&  (a_24*POS_eL2(1,1)+b_24) < POS_eL2(2,1)
     contact_flag_L2(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eL2(1,1)+b_41_2) <= POS_eL2(2,1)  &&  (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  (a_13*POS_eL2(1,1)+b_13) < POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(4,1) = 111;   % 領域4,1
 else
     contact_flag_L2(5,5) = 111;   % 非接触
 end
% R1
 if     (a_41_2*POS_eR1(1,1)+b_41_2) <= POS_eR1(2,1)  &&  (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_41_1*POS_eR1(1,1)+b_41_1)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  sqrt( (POS_eR1(1,1)-shikaku1(1,1))^2 + (POS_eR1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R1(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eR1(1,1)+b_23_1) <= POS_eR1(2,1)  &&  (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_23_2*POS_eR1(1,1)+b_23_2)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  sqrt( (POS_eR1(1,1)-shikaku2(1,1))^2 + (POS_eR1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R1(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eR1(1,1)+b_23_1) <= POS_eR1(2,1)  &&  (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_23_2*POS_eR1(1,1)+b_23_2)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  sqrt( (POS_eR1(1,1)-shikaku3(1,1))^2 + (POS_eR1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R1(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eR1(1,1)+b_41_2) <= POS_eR1(2,1)  &&  (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_41_1*POS_eR1(1,1)+b_41_1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&   sqrt( (POS_eR1(1,1)-shikaku4(1,1))^2 + (POS_eR1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R1(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eR1(1,1)+b_41_1) < POS_eR1(2,1)  &&  (a_12_2*POS_eR1(1,1)+b_12_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_23_1*POS_eR1(1,1)+b_23_1)  &&  POS_eR1(2,1) <= (a_13*POS_eR1(1,1)+b_13)  &&  POS_eR1(2,1) < (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_23_2*POS_eR1(1,1)+b_23_2)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  POS_eR1(2,1) < (a_13*POS_eR1(1,1)+b_13)  &&  (a_24*POS_eR1(1,1)+b_24) <= POS_eR1(2,1)
     contact_flag_R1(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eR1(1,1)+b_41_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_23_1*POS_eR1(1,1)+b_23_1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  (a_13*POS_eR1(1,1)+b_13) <= POS_eR1(2,1)  &&  (a_24*POS_eR1(1,1)+b_24) < POS_eR1(2,1)
     contact_flag_R1(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eR1(1,1)+b_41_2) <= POS_eR1(2,1)  &&  (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  (a_13*POS_eR1(1,1)+b_13) < POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(4,1) = 111;   % 領域4,1
 else
     contact_flag_R1(5,5) = 111;   % 非接触
 end
% R2
 if     (a_41_2*POS_eR2(1,1)+b_41_2) <= POS_eR2(2,1)  &&  (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_41_1*POS_eR2(1,1)+b_41_1)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  sqrt( (POS_eR2(1,1)-shikaku1(1,1))^2 + (POS_eR2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R2(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eR2(1,1)+b_23_1) <= POS_eR2(2,1)  &&  (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_23_2*POS_eR2(1,1)+b_23_2)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  sqrt( (POS_eR2(1,1)-shikaku2(1,1))^2 + (POS_eR2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R2(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eR2(1,1)+b_23_1) <= POS_eR2(2,1)  &&  (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_23_2*POS_eR2(1,1)+b_23_2)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  sqrt( (POS_eR2(1,1)-shikaku3(1,1))^2 + (POS_eR2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R2(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eR2(1,1)+b_41_2) <= POS_eR2(2,1)  &&  (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_41_1*POS_eR2(1,1)+b_41_1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&   sqrt( (POS_eR2(1,1)-shikaku4(1,1))^2 + (POS_eR2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R2(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eR2(1,1)+b_41_1) < POS_eR2(2,1)  &&  (a_12_2*POS_eR2(1,1)+b_12_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_23_1*POS_eR2(1,1)+b_23_1)  &&  POS_eR2(2,1) <= (a_13*POS_eR2(1,1)+b_13)  &&  POS_eR2(2,1) < (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_23_2*POS_eR2(1,1)+b_23_2)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  POS_eR2(2,1) < (a_13*POS_eR2(1,1)+b_13)  &&  (a_24*POS_eR2(1,1)+b_24) <= POS_eR2(2,1)
     contact_flag_R2(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eR2(1,1)+b_41_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_23_1*POS_eR2(1,1)+b_23_1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  (a_13*POS_eR2(1,1)+b_13) <= POS_eR2(2,1)  &&  (a_24*POS_eR2(1,1)+b_24) < POS_eR2(2,1)
     contact_flag_R2(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eR2(1,1)+b_41_2) <= POS_eR2(2,1)  &&  (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  (a_13*POS_eR2(1,1)+b_13) < POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(4,1) = 111;   % 領域4,1
 else
     contact_flag_R2(5,5) = 111;   % 非接触
 end


%%%%%% ターゲットの姿勢(q0)がπ/2+θc〜πのとき   % elseif pi/4 < q0 < pi/2
else   % elseif ( pi/2 + theta_chohou) < q0 < pi

a_12_1 = ( shikaku1(2,1) - shikaku2(2,1) ) / ( shikaku1(1,1) - shikaku2(1,1) );
a_12_2 = ( shikaku1_2(2,1) - shikaku2_1(2,1) ) / ( shikaku1_2(1,1) - shikaku2_1(1,1) );
a_23_1 = ( shikaku2(2,1) - shikaku3(2,1) ) / ( shikaku2(1,1) - shikaku3(1,1) );
a_23_2 = ( shikaku2_3(2,1) - shikaku3_2(2,1) ) / ( shikaku2_3(1,1) - shikaku3_2(1,1) );
a_34_1 = ( shikaku3(2,1) - shikaku4(2,1) ) / ( shikaku3(1,1) - shikaku4(1,1) );
a_34_2 = ( shikaku3_4(2,1) - shikaku4_3(2,1) ) / ( shikaku3_4(1,1) - shikaku4_3(1,1) );
a_41_1 = ( shikaku4(2,1) - shikaku1(2,1) ) / ( shikaku4(1,1) - shikaku1(1,1) );
a_41_2 = ( shikaku4_1(2,1) - shikaku1_4(2,1) ) / ( shikaku4_1(1,1) - shikaku1_4(1,1) );
b_12_1 = shikaku1(2,1) - a_12_1*shikaku1(1,1);
b_12_2 = shikaku1_2(2,1) - a_12_2*shikaku1_2(1,1);
b_23_1 = shikaku2(2,1) - a_23_1*shikaku2(1,1);
b_23_2 = shikaku2_3(2,1) - a_23_2*shikaku2_3(1,1);
b_34_1 = shikaku3(2,1) - a_34_1*shikaku3(1,1);
b_34_2 = shikaku3_4(2,1) - a_34_2*shikaku3_4(1,1);
b_41_1 = shikaku4(2,1) - a_41_1*shikaku4(1,1);
b_41_2 = shikaku4_1(2,1) - a_41_2*shikaku4_1(1,1);
a_13 = ( shikaku1(2,1) - shikaku3(2,1) ) / ( shikaku1(1,1) - shikaku3(1,1) );
a_24 = ( shikaku2(2,1) - shikaku4(2,1) ) / ( shikaku2(1,1) - shikaku4(1,1) );
b_13 = shikaku1(2,1) - a_13*shikaku1(1,1);
b_24 = shikaku2(2,1) - a_24*shikaku2(1,1);

% L1
 if     (a_41_2*POS_eL1(1,1)+b_41_2) <= POS_eL1(2,1)  &&  (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_41_1*POS_eL1(1,1)+b_41_1)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  sqrt( (POS_eL1(1,1)-shikaku1(1,1))^2 + (POS_eL1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L1(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eL1(1,1)+b_23_1) <= POS_eL1(2,1)  &&  (a_12_2*POS_eL1(1,1)+b_12_2) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_23_2*POS_eL1(1,1)+b_23_2)  &&  POS_eL1(2,1) <= (a_12_1*POS_eL1(1,1)+b_12_1)  &&  sqrt( (POS_eL1(1,1)-shikaku2(1,1))^2 + (POS_eL1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L1(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eL1(1,1)+b_23_1) <= POS_eL1(2,1)  &&  (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_23_2*POS_eL1(1,1)+b_23_2)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  sqrt( (POS_eL1(1,1)-shikaku3(1,1))^2 + (POS_eL1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L1(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eL1(1,1)+b_41_2) <= POS_eL1(2,1)  &&  (a_34_1*POS_eL1(1,1)+b_34_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_41_1*POS_eL1(1,1)+b_41_1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&   sqrt( (POS_eL1(1,1)-shikaku4(1,1))^2 + (POS_eL1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L1(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eL1(1,1)+b_41_1) < POS_eL1(2,1)  &&  (a_12_2*POS_eL1(1,1)+b_12_1) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_23_1*POS_eL1(1,1)+b_23_1)  &&  (a_13*POS_eL1(1,1)+b_13) <= POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) <= (a_23_2*POS_eL1(1,1)+b_23_2)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  (a_13*POS_eL1(1,1)+b_13) < POS_eL1(2,1)  &&  (a_24*POS_eL1(1,1)+b_24) <= POS_eL1(2,1)
     contact_flag_L1(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eL1(1,1)+b_41_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_23_1*POS_eL1(1,1)+b_23_1)  &&  POS_eL1(2,1) <= (a_34_2*POS_eL1(1,1)+b_34_2)  &&  POS_eL1(2,1) <= (a_13*POS_eL1(1,1)+b_13)  &&  (a_24*POS_eL1(1,1)+b_24) < POS_eL1(2,1)
     contact_flag_L1(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eL1(1,1)+b_41_2) <= POS_eL1(2,1)  &&  (a_12_1*POS_eL1(1,1)+b_12_1) < POS_eL1(2,1)  &&  POS_eL1(2,1) < (a_34_1*POS_eL1(1,1)+b_34_1)  &&  POS_eL1(2,1) < (a_13*POS_eL1(1,1)+b_13)  &&  POS_eL1(2,1) <= (a_24*POS_eL1(1,1)+b_24)
     contact_flag_L1(4,1) = 111;   % 領域4,1
 else
     contact_flag_L1(5,5) = 111;   % 非接触
 end
% L2
 if     (a_41_2*POS_eL2(1,1)+b_41_2) <= POS_eL2(2,1)  &&  (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_41_1*POS_eL2(1,1)+b_41_1)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  sqrt( (POS_eL2(1,1)-shikaku1(1,1))^2 + (POS_eL2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_L2(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eL2(1,1)+b_23_1) <= POS_eL2(2,1)  &&  (a_12_2*POS_eL2(1,1)+b_12_2) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_23_2*POS_eL2(1,1)+b_23_2)  &&  POS_eL2(2,1) <= (a_12_1*POS_eL2(1,1)+b_12_1)  &&  sqrt( (POS_eL2(1,1)-shikaku2(1,1))^2 + (POS_eL2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_L2(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eL2(1,1)+b_23_1) <= POS_eL2(2,1)  &&  (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_23_2*POS_eL2(1,1)+b_23_2)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  sqrt( (POS_eL2(1,1)-shikaku3(1,1))^2 + (POS_eL2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_L2(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eL2(1,1)+b_41_2) <= POS_eL2(2,1)  &&  (a_34_1*POS_eL2(1,1)+b_34_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_41_1*POS_eL2(1,1)+b_41_1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&   sqrt( (POS_eL2(1,1)-shikaku4(1,1))^2 + (POS_eL2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_L2(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eL2(1,1)+b_41_1) < POS_eL2(2,1)  &&  (a_12_2*POS_eL2(1,1)+b_12_1) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_23_1*POS_eL2(1,1)+b_23_1)  &&  (a_13*POS_eL2(1,1)+b_13) <= POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) <= (a_23_2*POS_eL2(1,1)+b_23_2)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  (a_13*POS_eL2(1,1)+b_13) < POS_eL2(2,1)  &&  (a_24*POS_eL2(1,1)+b_24) <= POS_eL2(2,1)
     contact_flag_L2(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eL2(1,1)+b_41_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_23_1*POS_eL2(1,1)+b_23_1)  &&  POS_eL2(2,1) <= (a_34_2*POS_eL2(1,1)+b_34_2)  &&  POS_eL2(2,1) <= (a_13*POS_eL2(1,1)+b_13)  &&  (a_24*POS_eL2(1,1)+b_24) < POS_eL2(2,1)
     contact_flag_L2(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eL2(1,1)+b_41_2) <= POS_eL2(2,1)  &&  (a_12_1*POS_eL2(1,1)+b_12_1) < POS_eL2(2,1)  &&  POS_eL2(2,1) < (a_34_1*POS_eL2(1,1)+b_34_1)  &&  POS_eL2(2,1) < (a_13*POS_eL2(1,1)+b_13)  &&  POS_eL2(2,1) <= (a_24*POS_eL2(1,1)+b_24)
     contact_flag_L2(4,1) = 111;   % 領域4,1
 else
     contact_flag_L2(5,5) = 111;   % 非接触
 end
% R1
 if     (a_41_2*POS_eR1(1,1)+b_41_2) <= POS_eR1(2,1)  &&  (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_41_1*POS_eR1(1,1)+b_41_1)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  sqrt( (POS_eR1(1,1)-shikaku1(1,1))^2 + (POS_eR1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R1(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eR1(1,1)+b_23_1) <= POS_eR1(2,1)  &&  (a_12_2*POS_eR1(1,1)+b_12_2) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_23_2*POS_eR1(1,1)+b_23_2)  &&  POS_eR1(2,1) <= (a_12_1*POS_eR1(1,1)+b_12_1)  &&  sqrt( (POS_eR1(1,1)-shikaku2(1,1))^2 + (POS_eR1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R1(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eR1(1,1)+b_23_1) <= POS_eR1(2,1)  &&  (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_23_2*POS_eR1(1,1)+b_23_2)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  sqrt( (POS_eR1(1,1)-shikaku3(1,1))^2 + (POS_eR1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R1(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eR1(1,1)+b_41_2) <= POS_eR1(2,1)  &&  (a_34_1*POS_eR1(1,1)+b_34_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_41_1*POS_eR1(1,1)+b_41_1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&   sqrt( (POS_eR1(1,1)-shikaku4(1,1))^2 + (POS_eR1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R1(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eR1(1,1)+b_41_1) < POS_eR1(2,1)  &&  (a_12_2*POS_eR1(1,1)+b_12_1) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_23_1*POS_eR1(1,1)+b_23_1)  &&  (a_13*POS_eR1(1,1)+b_13) <= POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) <= (a_23_2*POS_eR1(1,1)+b_23_2)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  (a_13*POS_eR1(1,1)+b_13) < POS_eR1(2,1)  &&  (a_24*POS_eR1(1,1)+b_24) <= POS_eR1(2,1)
     contact_flag_R1(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eR1(1,1)+b_41_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_23_1*POS_eR1(1,1)+b_23_1)  &&  POS_eR1(2,1) <= (a_34_2*POS_eR1(1,1)+b_34_2)  &&  POS_eR1(2,1) <= (a_13*POS_eR1(1,1)+b_13)  &&  (a_24*POS_eR1(1,1)+b_24) < POS_eR1(2,1)
     contact_flag_R1(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eR1(1,1)+b_41_2) <= POS_eR1(2,1)  &&  (a_12_1*POS_eR1(1,1)+b_12_1) < POS_eR1(2,1)  &&  POS_eR1(2,1) < (a_34_1*POS_eR1(1,1)+b_34_1)  &&  POS_eR1(2,1) < (a_13*POS_eR1(1,1)+b_13)  &&  POS_eR1(2,1) <= (a_24*POS_eR1(1,1)+b_24)
     contact_flag_R1(4,1) = 111;   % 領域4,1
 else
     contact_flag_R1(5,5) = 111;   % 非接触
 end
% R2
 if     (a_41_2*POS_eR2(1,1)+b_41_2) <= POS_eR2(2,1)  &&  (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_41_1*POS_eR2(1,1)+b_41_1)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  sqrt( (POS_eR2(1,1)-shikaku1(1,1))^2 + (POS_eR2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_R2(1,1) = 111;   % 領域1,1
 elseif (a_23_1*POS_eR2(1,1)+b_23_1) <= POS_eR2(2,1)  &&  (a_12_2*POS_eR2(1,1)+b_12_2) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_23_2*POS_eR2(1,1)+b_23_2)  &&  POS_eR2(2,1) <= (a_12_1*POS_eR2(1,1)+b_12_1)  &&  sqrt( (POS_eR2(1,1)-shikaku2(1,1))^2 + (POS_eR2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_R2(2,2) = 111;   % 領域2,2
 elseif (a_23_1*POS_eR2(1,1)+b_23_1) <= POS_eR2(2,1)  &&  (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_23_2*POS_eR2(1,1)+b_23_2)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  sqrt( (POS_eR2(1,1)-shikaku3(1,1))^2 + (POS_eR2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_R2(3,3) = 111;   % 領域3,3
 elseif (a_41_2*POS_eR2(1,1)+b_41_2) <= POS_eR2(2,1)  &&  (a_34_1*POS_eR2(1,1)+b_34_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_41_1*POS_eR2(1,1)+b_41_1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&   sqrt( (POS_eR2(1,1)-shikaku4(1,1))^2 + (POS_eR2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_R2(4,4) = 111;   % 領域4,4
 elseif (a_41_1*POS_eR2(1,1)+b_41_1) < POS_eR2(2,1)  &&  (a_12_2*POS_eR2(1,1)+b_12_1) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_23_1*POS_eR2(1,1)+b_23_1)  &&  (a_13*POS_eR2(1,1)+b_13) <= POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(1,2) = 111;   % 領域1,2
 elseif (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) <= (a_23_2*POS_eR2(1,1)+b_23_2)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  (a_13*POS_eR2(1,1)+b_13) < POS_eR2(2,1)  &&  (a_24*POS_eR2(1,1)+b_24) <= POS_eR2(2,1)
     contact_flag_R2(2,3) = 111;   % 領域2,3
 elseif (a_41_1*POS_eR2(1,1)+b_41_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_23_1*POS_eR2(1,1)+b_23_1)  &&  POS_eR2(2,1) <= (a_34_2*POS_eR2(1,1)+b_34_2)  &&  POS_eR2(2,1) <= (a_13*POS_eR2(1,1)+b_13)  &&  (a_24*POS_eR2(1,1)+b_24) < POS_eR2(2,1)
     contact_flag_R2(3,4) = 111;   % 領域3,4
 elseif (a_41_2*POS_eR2(1,1)+b_41_2) <= POS_eR2(2,1)  &&  (a_12_1*POS_eR2(1,1)+b_12_1) < POS_eR2(2,1)  &&  POS_eR2(2,1) < (a_34_1*POS_eR2(1,1)+b_34_1)  &&  POS_eR2(2,1) < (a_13*POS_eR2(1,1)+b_13)  &&  POS_eR2(2,1) <= (a_24*POS_eR2(1,1)+b_24)
     contact_flag_R2(4,1) = 111;   % 領域4,1
 else
     contact_flag_R2(5,5) = 111;   % 非接触
 end


end


end