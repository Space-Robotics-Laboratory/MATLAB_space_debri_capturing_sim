function [ shikaku1, shikaku2, shikaku3, shikaku4, shikaku1_2, shikaku2_3, shikaku3_4, shikaku4_1, shikaku2_1, shikaku3_2, shikaku4_3, shikaku1_4, ...
           l1, l2, l3, l4, gamma1, gamma2, gamma3, gamma4, contact_flag_left1, contact_flag_left2, contact_flag_right3, contact_flag_right4 ] ...
= Shikaku_FourTips_CONTACT_FLAG( SV_ts, q0, side_target, r_tip, ts_geo_center, POS_e1, POS_e2, POS_e3, POS_e4 )

% 半径
half_side = side_target / 2;

% 接触領域判定フラグ行列設定
contact_flag_left1 = zeros(5,5);
contact_flag_left2 = zeros(5,5);
contact_flag_right3 = zeros(5,5);
contact_flag_right4 = zeros(5,5);

% ターゲット頂点設定
    shikaku1(1,1) = ts_geo_center(1,1) + sqrt(2) * half_side * cos( 3*pi/4 + q0 );
    shikaku1(2,1) = ts_geo_center(2,1) + sqrt(2) * half_side * sin( 3*pi/4 + q0 );
    shikaku1(3,1) = 0;
    shikaku2(1,1) = ts_geo_center(1,1) + sqrt(2) * half_side * cos( -3*pi/4 + q0 );
    shikaku2(2,1) = ts_geo_center(2,1) + sqrt(2) * half_side * sin( -3*pi/4 + q0 );
    shikaku2(3,1) = 0;
    shikaku3(1,1) = ts_geo_center(1,1) + sqrt(2) * half_side * cos( -pi/4 + q0 );
    shikaku3(2,1) = ts_geo_center(2,1) + sqrt(2) * half_side * sin( -pi/4 + q0 );
    shikaku3(3,1) = 0;
    shikaku4(1,1) = ts_geo_center(1,1) + sqrt(2) * half_side * cos( pi/4 + q0 );
    shikaku4(2,1) = ts_geo_center(2,1) + sqrt(2) * half_side * sin( pi/4 + q0 );
    shikaku4(3,1) = 0;

% ターゲット重心からの各頂点の角度
    l1 = sqrt( ( shikaku1(1,1) - ts_geo_center(1,1) )^2 + ( shikaku1(2,1) - ts_geo_center(2,1) )^2 );
    l2 = sqrt( ( shikaku2(1,1) - ts_geo_center(1,1) )^2 + ( shikaku2(2,1) - ts_geo_center(2,1) )^2 );
    l3 = sqrt( ( shikaku3(1,1) - ts_geo_center(1,1) )^2 + ( shikaku3(2,1) - ts_geo_center(2,1) )^2 );
    l4 = sqrt( ( shikaku4(1,1) - ts_geo_center(1,1) )^2 + ( shikaku4(2,1) - ts_geo_center(2,1) )^2 );
    gamma1 = acos( abs( SV_ts.R0(2,1) - shikaku1(2,1) ) / l1 );
    gamma2 = acos( abs( SV_ts.R0(2,1) - shikaku2(2,1) ) / l2 );
    gamma3 = acos( abs( SV_ts.R0(2,1) - shikaku3(2,1) ) / l3 );
    gamma4 = acos( abs( SV_ts.R0(2,1) - shikaku4(2,1) ) / l4 );

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

% ターゲットの姿勢が0のとき
if q0 == 0

a_13 = ( shikaku1(2,1) - shikaku3(2,1) ) / ( shikaku1(1,1) - shikaku3(1,1) );
a_24 = ( shikaku2(2,1) - shikaku4(2,1) ) / ( shikaku2(1,1) - shikaku4(1,1) );
b_13 = shikaku1(2,1) - a_13*shikaku1(1,1);
b_24 = shikaku2(2,1) - a_24*shikaku2(1,1);

% 左手1の位置
 if     shikaku1_2(1,1) <= POS_e1(1,1)  &&  shikaku1(2,1) <= POS_e1(2,1)  &&  POS_e1(1,1) <= shikaku1(1,1)  &&  POS_e1(2,1) <= shikaku1_4(2,1)  &&  sqrt( (POS_e1(1,1)-shikaku1(1,1))^2 + (POS_e1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_left1(1,1) = 111;
 elseif shikaku2_1(1,1) <= POS_e1(1,1)  &&  shikaku2_3(2,1) <= POS_e1(2,1)  &&  POS_e1(1,1) <= shikaku2(1,1)  &&  POS_e1(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_e1(1,1)-shikaku2(1,1))^2 + (POS_e1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_left1(2,2) = 111;
 elseif shikaku3(1,1) <= POS_e1(1,1)  &&  shikaku3_2(2,1) <= POS_e1(2,1)  &&  POS_e1(1,1) <= shikaku3_4(1,1)  &&  POS_e1(2,1) <= shikaku3(2,1)  &&  sqrt( (POS_e1(1,1)-shikaku3(1,1))^2 + (POS_e1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_left1(3,3) = 111;
 elseif shikaku4(1,1) <= POS_e1(1,1)  &&  shikaku4(2,1) <= POS_e1(2,1)  &&  POS_e1(1,1) <= shikaku4_3(1,1)  &&  POS_e1(2,1) <= shikaku4_1(2,1)  &&  sqrt( (POS_e1(1,1)-shikaku4(1,1))^2 + (POS_e1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_left1(4,4) = 111;
 elseif shikaku1_2(1,1) <= POS_e1(1,1)  &&  POS_e1(2,1) < shikaku1(2,1)  &&  shikaku2(2,1) < POS_e1(2,1)  &&  POS_e1(2,1) <= (a_13*POS_e1(1,1)+b_13)  &&  (a_24*POS_e1(1,1)+b_24) < POS_e1(2,1)
     contact_flag_left1(1,2) = 111;
 elseif shikaku2_3(2,1) <= POS_e1(2,1)  &&  shikaku2(1,1) < POS_e1(1,1)  &&  POS_e1(1,1) < shikaku3(1,1)  &&  POS_e1(2,1) < (a_13*POS_e1(1,1)+b_13)  &&  POS_e1(2,1) <= (a_24*POS_e1(1,1)+b_24)
     contact_flag_left1(2,3) = 111;
 elseif POS_e1(1,1) <= shikaku3_4(1,1)  &&  shikaku3(2,1) < POS_e1(2,1)  &&  POS_e1(2,1) < shikaku4(2,1)  &&  (a_13*POS_e1(1,1)+b_13) <= POS_e1(2,1)  &&  POS_e1(2,1) < (a_24*POS_e1(1,1)+b_24)
     contact_flag_left1(3,4) = 111;
 elseif POS_e1(2,1) <= shikaku4_1(2,1)  &&  shikaku1(1,1) < POS_e1(1,1)  &&  POS_e1(1,1) < shikaku4(1,1)  &&  (a_13*POS_e1(1,1)+b_13) < POS_e1(2,1)  &&  (a_24*POS_e1(1,1)+b_24) <= POS_e1(2,1)
     contact_flag_left1(4,1) = 111;
 else
     contact_flag_left1(5,5) = 111;
 end
% 左手2の位置
 if     shikaku1_2(1,1) <= POS_e2(1,1)  &&  shikaku1(2,1) <= POS_e2(2,1)  &&  POS_e2(1,1) <= shikaku1(1,1)  &&  POS_e2(2,1) <= shikaku1_4(2,1)  &&  sqrt( (POS_e2(1,1)-shikaku1(1,1))^2 + (POS_e2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_left2(1,1) = 111;
 elseif shikaku2_1(1,1) <= POS_e2(1,1)  &&  shikaku2_3(2,1) <= POS_e2(2,1)  &&  POS_e2(1,1) <= shikaku2(1,1)  &&  POS_e2(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_e2(1,1)-shikaku2(1,1))^2 + (POS_e2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_left2(2,2) = 111;
 elseif shikaku3(1,1) <= POS_e2(1,1)  &&  shikaku3_2(2,1) <= POS_e2(2,1)  &&  POS_e2(1,1) <= shikaku3_4(1,1)  &&  POS_e2(2,1) <= shikaku3(2,1)  &&  sqrt( (POS_e2(1,1)-shikaku3(1,1))^2 + (POS_e2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_left2(3,3) = 111;
 elseif shikaku4(1,1) <= POS_e2(1,1)  &&  shikaku4(2,1) <= POS_e2(2,1)  &&  POS_e2(1,1) <= shikaku4_3(1,1)  &&  POS_e2(2,1) <= shikaku4_1(2,1)  &&  sqrt( (POS_e2(1,1)-shikaku4(1,1))^2 + (POS_e2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_left2(4,4) = 111;
 elseif shikaku1_2(1,1) <= POS_e2(1,1)  &&  POS_e2(2,1) < shikaku1(2,1)  &&  shikaku2(2,1) < POS_e2(2,1)  &&  POS_e2(2,1) <= (a_13*POS_e2(1,1)+b_13)  &&  (a_24*POS_e2(1,1)+b_24) < POS_e2(2,1)
     contact_flag_left2(1,2) = 111;
 elseif shikaku2_3(2,1) <= POS_e2(2,1)  &&  shikaku2(1,1) < POS_e2(1,1)  &&  POS_e2(1,1) < shikaku3(1,1)  &&  POS_e2(2,1) < (a_13*POS_e2(1,1)+b_13)  &&  POS_e2(2,1) <= (a_24*POS_e2(1,1)+b_24)
     contact_flag_left2(2,3) = 111;
 elseif POS_e2(1,1) <= shikaku3_4(1,1)  &&  shikaku3(2,1) < POS_e2(2,1)  &&  POS_e2(2,1) < shikaku4(2,1)  &&  (a_13*POS_e2(1,1)+b_13) <= POS_e2(2,1)  &&  POS_e2(2,1) < (a_24*POS_e2(1,1)+b_24)
     contact_flag_left2(3,4) = 111;
 elseif POS_e2(2,1) <= shikaku4_1(2,1)  &&  shikaku1(1,1) < POS_e2(1,1)  &&  POS_e2(1,1) < shikaku4(1,1)  &&  (a_13*POS_e2(1,1)+b_13) < POS_e2(2,1)  &&  (a_24*POS_e2(1,1)+b_24) <= POS_e2(2,1)
     contact_flag_left2(4,1) = 111;
 else
     contact_flag_left2(5,5) = 111;
 end

% 右手3の位置
 if     shikaku1_2(1,1) <= POS_e3(1,1)  &&  shikaku1(2,1) <= POS_e3(2,1)  &&  POS_e3(1,1) <= shikaku1(1,1)  &&  POS_e3(2,1) <= shikaku1_4(2,1)  &&  sqrt( (POS_e3(1,1)-shikaku1(1,1))^2 + (POS_e3(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_right3(1,1) = 111;
 elseif shikaku2_1(1,1) <= POS_e3(1,1)  &&  shikaku2_3(2,1) <= POS_e3(2,1)  &&  POS_e3(1,1) <= shikaku2(1,1)  &&  POS_e3(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_e3(1,1)-shikaku2(1,1))^2 + (POS_e3(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_right3(2,2) = 111;
 elseif shikaku3(1,1) <= POS_e3(1,1)  &&  shikaku3_2(2,1) <= POS_e3(2,1)  &&  POS_e3(1,1) <= shikaku3_4(1,1)  &&  POS_e3(2,1) <= shikaku3(2,1)  &&  sqrt( (POS_e3(1,1)-shikaku3(1,1))^2 + (POS_e3(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_right3(3,3) = 111;
 elseif shikaku4(1,1) <= POS_e3(1,1)  &&  shikaku4(2,1) <= POS_e3(2,1)  &&  POS_e3(1,1) <= shikaku4_3(1,1)  &&  POS_e3(2,1) <= shikaku4_1(2,1)  &&  sqrt( (POS_e3(1,1)-shikaku4(1,1))^2 + (POS_e3(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_right3(4,4) = 111;
 elseif shikaku1_2(1,1) <= POS_e3(1,1)  &&  POS_e3(2,1) < shikaku1(2,1)  &&  shikaku2(2,1) < POS_e3(2,1)  &&  POS_e3(2,1) <= (a_13*POS_e3(1,1)+b_13)  &&  (a_24*POS_e3(1,1)+b_24) < POS_e3(2,1)
     contact_flag_right3(1,2) = 111;
 elseif shikaku2_3(2,1) <= POS_e3(2,1)  &&  shikaku2(1,1) < POS_e3(1,1)  &&  POS_e3(1,1) < shikaku3(1,1)  &&  POS_e3(2,1) < (a_13*POS_e3(1,1)+b_13)  &&  POS_e3(2,1) <= (a_24*POS_e3(1,1)+b_24)
     contact_flag_right3(2,3) = 111;
 elseif POS_e3(1,1) <= shikaku3_4(1,1)  &&  shikaku3(2,1) < POS_e3(2,1)  &&  POS_e3(2,1) < shikaku4(2,1)  &&  (a_13*POS_e3(1,1)+b_13) <= POS_e3(2,1)  &&  POS_e3(2,1) < (a_24*POS_e3(1,1)+b_24)
     contact_flag_right3(3,4) = 111;
 elseif POS_e3(2,1) <= shikaku4_1(2,1)  &&  POS_e3(1,1) < shikaku4(1,1)  &&  shikaku1(1,1) < POS_e3(1,1)  &&  (a_13*POS_e3(1,1)+b_13) < POS_e3(2,1)  &&  (a_24*POS_e3(1,1)+b_24) <= POS_e3(2,1)
     contact_flag_right3(4,1) = 111;
 else
     contact_flag_right3(5,5) = 111;
 end
% 右手4の位置
 if     shikaku1_2(1,1) <= POS_e4(1,1)  &&  shikaku1(2,1) <= POS_e4(2,1)  &&  POS_e4(1,1) <= shikaku1(1,1)  &&  POS_e4(2,1) <= shikaku1_4(2,1)  &&  sqrt( (POS_e4(1,1)-shikaku1(1,1))^2 + (POS_e4(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_right4(1,1) = 111;
 elseif shikaku2_1(1,1) <= POS_e4(1,1)  &&  shikaku2_3(2,1) <= POS_e4(2,1)  &&  POS_e4(1,1) <= shikaku2(1,1)  &&  POS_e4(2,1) <= shikaku2(2,1)  &&  sqrt( (POS_e4(1,1)-shikaku2(1,1))^2 + (POS_e4(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_right4(2,2) = 111;
 elseif shikaku3(1,1) <= POS_e4(1,1)  &&  shikaku3_2(2,1) <= POS_e4(2,1)  &&  POS_e4(1,1) <= shikaku3_4(1,1)  &&  POS_e4(2,1) <= shikaku3(2,1)  &&  sqrt( (POS_e4(1,1)-shikaku3(1,1))^2 + (POS_e4(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_right4(3,3) = 111;
 elseif shikaku4(1,1) <= POS_e4(1,1)  &&  shikaku4(2,1) <= POS_e4(2,1)  &&  POS_e4(1,1) <= shikaku4_3(1,1)  &&  POS_e4(2,1) <= shikaku4_1(2,1)  &&  sqrt( (POS_e4(1,1)-shikaku4(1,1))^2 + (POS_e4(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_right4(4,4) = 111;
 elseif shikaku1_2(1,1) <= POS_e4(1,1)  &&  POS_e4(2,1) < shikaku1(2,1)  &&  shikaku2(2,1) < POS_e4(2,1)  &&  POS_e4(2,1) <= (a_13*POS_e4(1,1)+b_13)  &&  (a_24*POS_e4(1,1)+b_24) < POS_e4(2,1)
     contact_flag_right4(1,2) = 111;
 elseif shikaku2_3(2,1) <= POS_e4(2,1)  &&  shikaku2(1,1) < POS_e4(1,1)  &&  POS_e4(1,1) < shikaku3(1,1)  &&  POS_e4(2,1) < (a_13*POS_e4(1,1)+b_13)  &&  POS_e4(2,1) <= (a_24*POS_e4(1,1)+b_24)
     contact_flag_right4(2,3) = 111;
 elseif POS_e4(1,1) <= shikaku3_4(1,1)  &&  shikaku3(2,1) < POS_e4(2,1)  &&  POS_e4(2,1) < shikaku4(2,1)  &&  (a_13*POS_e4(1,1)+b_13) <= POS_e4(2,1)  &&  POS_e4(2,1) < (a_24*POS_e4(1,1)+b_24)
     contact_flag_right4(3,4) = 111;
 elseif POS_e4(2,1) <= shikaku4_1(2,1)  &&  POS_e4(1,1) < shikaku4(1,1)  &&  shikaku1(1,1) < POS_e4(1,1)  &&  (a_13*POS_e4(1,1)+b_13) < POS_e4(2,1)  &&  (a_24*POS_e4(1,1)+b_24) <= POS_e4(2,1)
     contact_flag_right4(4,1) = 111;
 else
     contact_flag_right4(5,5) = 111;
 end


% ターゲットの姿勢がπ/4のとき
elseif q0 == pi/4

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

% 左手1の位置
 if     (a_12_2*POS_e1(1,1)+b_12_2) <= POS_e1(2,1)  &&  (a_41_1*POS_e1(1,1)+b_41_1) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_12_1*POS_e1(1,1)+b_12_1)  &&  POS_e1(2,1) <= (a_41_2*POS_e1(1,1)+b_41_2)  &&  sqrt( (POS_e1(1,1)-shikaku1(1,1))^2 + (POS_e1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_left1(1,1) = 111;
 elseif (a_12_2*POS_e1(1,1)+b_12_2) <= POS_e1(2,1)  &&  (a_23_2*POS_e1(1,1)+b_23_2) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_12_1*POS_e1(1,1)+b_12_1)  &&  POS_e1(2,1) <= (a_23_1*POS_e1(1,1)+b_23_1)  &&  sqrt( (POS_e1(1,1)-shikaku2(1,1))^2 + (POS_e1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_left1(2,2) = 111;
 elseif (a_34_1*POS_e1(1,1)+b_34_1) <= POS_e1(2,1)  &&  (a_23_2*POS_e1(1,1)+b_23_2) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_34_2*POS_e1(1,1)+b_34_2)  &&  POS_e1(2,1) <= (a_23_1*POS_e1(1,1)+b_23_1)  &&  sqrt( (POS_e1(1,1)-shikaku3(1,1))^2 + (POS_e1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_left1(3,3) = 111; 
 elseif (a_34_1*POS_e1(1,1)+b_34_1) <= POS_e1(2,1)  &&  (a_41_1*POS_e1(1,1)+b_41_1) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_34_2*POS_e1(1,1)+b_34_2)  &&  POS_e1(2,1) <= (a_41_2*POS_e1(1,1)+b_41_2)  &&  sqrt( (POS_e1(1,1)-shikaku4(1,1))^2 + (POS_e1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_left1(4,4) = 111;
 elseif (a_12_2*POS_e1(1,1)+b_12_2) <= POS_e1(2,1)  &&  (a_23_1*POS_e1(1,1)+b_23_1) < POS_e1(2,1)  &&  POS_e1(2,1) < (a_41_1*POS_e1(1,1)+b_41_1)  &&  POS_e1(2,1) <= ts_geo_center(2,1)  &&  POS_e1(1,1) < ts_geo_center(1,1)
     contact_flag_left1(1,2) = 111;
 elseif (a_23_2*POS_e1(1,1)+b_23_2) <= POS_e1(2,1)  &&  POS_e1(2,1) < (a_34_1*POS_e1(1,1)+b_34_1)  &&  (a_12_1*POS_e1(1,1)+b_12_1) < POS_e1(2,1)  &&  ts_geo_center(1,1) <= POS_e1(1,1)  &&  POS_e1(2,1) < ts_geo_center(2,1)
     contact_flag_left1(2,3) = 111;
 elseif POS_e1(2,1) <= (a_34_2*POS_e1(1,1)+b_34_2)  &&  (a_23_1*POS_e1(1,1)+b_23_1) < POS_e1(2,1)  &&  POS_e1(2,1) < (a_41_1*POS_e1(1,1)+b_41_1)  &&  ts_geo_center(2,1) <= POS_e1(2,1)  &&  ts_geo_center(1,1) < POS_e1(1,1)
     contact_flag_left1(3,4) = 111;
 elseif POS_e1(2,1) <= (a_41_2*POS_e1(1,1)+b_41_2)  &&  POS_e1(2,1) < (a_34_1*POS_e1(1,1)+b_34_1)  &&  (a_12_1*POS_e1(1,1)+b_12_1) < POS_e1(2,1)  &&  POS_e1(1,1) <= ts_geo_center(1,1)  &&  ts_geo_center(2,1) < POS_e1(2,1)
     contact_flag_left1(4,1) = 111;
 else
     contact_flag_left1(5,5) = 111;
 end
% 左手2の位置
 if     (a_12_2*POS_e2(1,1)+b_12_2) <= POS_e2(2,1)  &&  (a_41_1*POS_e2(1,1)+b_41_1) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_12_1*POS_e2(1,1)+b_12_1)  &&  POS_e2(2,1) <= (a_41_2*POS_e2(1,1)+b_41_2)  &&  sqrt( (POS_e2(1,1)-shikaku1(1,1))^2 + (POS_e2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_left2(1,1) = 111;
 elseif (a_12_2*POS_e2(1,1)+b_12_2) <= POS_e2(2,1)  &&  (a_23_2*POS_e2(1,1)+b_23_2) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_12_1*POS_e2(1,1)+b_12_1)  &&  POS_e2(2,1) <= (a_23_1*POS_e2(1,1)+b_23_1)  &&  sqrt( (POS_e2(1,1)-shikaku2(1,1))^2 + (POS_e2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_left2(2,2) = 111;
 elseif (a_34_1*POS_e2(1,1)+b_34_1) <= POS_e2(2,1)  &&  (a_23_2*POS_e2(1,1)+b_23_2) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_34_2*POS_e2(1,1)+b_34_2)  &&  POS_e2(2,1) <= (a_23_1*POS_e2(1,1)+b_23_1)  &&  sqrt( (POS_e2(1,1)-shikaku3(1,1))^2 + (POS_e2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_left2(3,3) = 111; 
 elseif (a_34_1*POS_e2(1,1)+b_34_1) <= POS_e2(2,1)  &&  (a_41_1*POS_e2(1,1)+b_41_1) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_34_2*POS_e2(1,1)+b_34_2)  &&  POS_e2(2,1) <= (a_41_2*POS_e2(1,1)+b_41_2)  &&  sqrt( (POS_e2(1,1)-shikaku4(1,1))^2 + (POS_e2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_left2(4,4) = 111;
 elseif (a_12_2*POS_e2(1,1)+b_12_2) <= POS_e2(2,1)  &&  (a_23_1*POS_e2(1,1)+b_23_1) < POS_e2(2,1)  &&  POS_e2(2,1) < (a_41_1*POS_e2(1,1)+b_41_1)  &&  POS_e2(2,1) <= ts_geo_center(2,1)  &&  POS_e2(1,1) < ts_geo_center(1,1)
     contact_flag_left2(1,2) = 111;
 elseif (a_23_2*POS_e2(1,1)+b_23_2) <= POS_e2(2,1)  &&  POS_e2(2,1) < (a_34_1*POS_e2(1,1)+b_34_1)  &&  (a_12_1*POS_e2(1,1)+b_12_1) < POS_e2(2,1)  &&  ts_geo_center(1,1) <= POS_e2(1,1)  &&  POS_e2(2,1) < ts_geo_center(2,1)
     contact_flag_left2(2,3) = 111;
 elseif POS_e2(2,1) <= (a_34_2*POS_e2(1,1)+b_34_2)  &&  (a_23_1*POS_e2(1,1)+b_23_1) < POS_e2(2,1)  &&  POS_e2(2,1) < (a_41_1*POS_e2(1,1)+b_41_1)  &&  ts_geo_center(2,1) <= POS_e2(2,1)  &&  ts_geo_center(1,1) < POS_e2(1,1)
     contact_flag_left2(3,4) = 111;
 elseif POS_e2(2,1) <= (a_41_2*POS_e2(1,1)+b_41_2)  &&  POS_e2(2,1) < (a_34_1*POS_e2(1,1)+b_34_1)  &&  (a_12_1*POS_e2(1,1)+b_12_1) < POS_e2(2,1)  &&  POS_e2(1,1) <= ts_geo_center(1,1)  &&  ts_geo_center(2,1) < POS_e2(2,1)
     contact_flag_left2(4,1) = 111;
 else
     contact_flag_left2(5,5) = 111;
 end

% 右手3の位置
 if     (a_12_2*POS_e3(1,1)+b_12_2) <= POS_e3(2,1)  &&  (a_41_1*POS_e3(1,1)+b_41_1) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_12_1*POS_e3(1,1)+b_12_1)  &&  POS_e3(2,1) <= (a_41_2*POS_e3(1,1)+b_41_2)  &&  sqrt( (POS_e3(1,1)-shikaku1(1,1))^2 + (POS_e3(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_right3(1,1) = 111;
 elseif (a_12_2*POS_e3(1,1)+b_12_2) <= POS_e3(2,1)  &&  (a_23_2*POS_e3(1,1)+b_23_2) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_12_1*POS_e3(1,1)+b_12_1)  &&  POS_e3(2,1) <= (a_23_1*POS_e3(1,1)+b_23_1)  &&  sqrt( (POS_e3(1,1)-shikaku2(1,1))^2 + (POS_e3(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_right3(2,2) = 111;
 elseif (a_34_1*POS_e3(1,1)+b_34_1) <= POS_e3(2,1)  &&  (a_23_2*POS_e3(1,1)+b_23_2) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_34_2*POS_e3(1,1)+b_34_2)  &&  POS_e3(2,1) <= (a_23_1*POS_e3(1,1)+b_23_1)  &&  sqrt( (POS_e3(1,1)-shikaku3(1,1))^2 + (POS_e3(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_right3(3,3) = 111; 
 elseif (a_34_1*POS_e3(1,1)+b_34_1) <= POS_e3(2,1)  &&  (a_41_1*POS_e3(1,1)+b_41_1) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_34_2*POS_e3(1,1)+b_34_2)  &&  POS_e3(2,1) <= (a_41_2*POS_e3(1,1)+b_41_2)  &&  sqrt( (POS_e3(1,1)-shikaku4(1,1))^2 + (POS_e3(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_right3(4,4) = 111;
 elseif (a_12_2*POS_e3(1,1)+b_12_2) <= POS_e3(2,1)  &&  (a_23_1*POS_e3(1,1)+b_23_1) < POS_e3(2,1)  &&  POS_e3(2,1) < (a_41_1*POS_e3(1,1)+b_41_1)  &&  POS_e3(2,1) <= ts_geo_center(2,1)  &&  POS_e3(1,1) < ts_geo_center(1,1)
     contact_flag_right3(1,2) = 111;
 elseif (a_23_2*POS_e3(1,1)+b_23_2) <= POS_e3(2,1)  &&  POS_e3(2,1) < (a_34_1*POS_e3(1,1)+b_34_1)  &&  (a_12_1*POS_e3(1,1)+b_12_1) < POS_e3(2,1)  &&  ts_geo_center(1,1) <= POS_e3(1,1)  &&  POS_e3(2,1) < ts_geo_center(2,1)
     contact_flag_right3(2,3) = 111;
 elseif POS_e3(2,1) <= (a_34_2*POS_e3(1,1)+b_34_2)  &&  (a_23_1*POS_e3(1,1)+b_23_1) < POS_e3(2,1)  &&  POS_e3(2,1) < (a_41_1*POS_e3(1,1)+b_41_1)  &&  ts_geo_center(2,1) <= POS_e3(2,1)  &&  ts_geo_center(1,1) < POS_e3(1,1)
     contact_flag_right3(3,4) = 111;
 elseif POS_e3(2,1) <= (a_41_2*POS_e3(1,1)+b_41_2)  &&  POS_e3(2,1) < (a_34_1*POS_e3(1,1)+b_34_1)  &&  (a_12_1*POS_e3(1,1)+b_12_1) < POS_e3(2,1)  &&  POS_e3(1,1) <= ts_geo_center(1,1)  &&  ts_geo_center(2,1) < POS_e3(2,1)
     contact_flag_right3(4,1) = 111;
 else
     contact_flag_right3(5,5) = 111;
 end
% 右手4の位置
 if     (a_12_2*POS_e4(1,1)+b_12_2) <= POS_e4(2,1)  &&  (a_41_1*POS_e4(1,1)+b_41_1) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_12_1*POS_e4(1,1)+b_12_1)  &&  POS_e4(2,1) <= (a_41_2*POS_e4(1,1)+b_41_2)  &&  sqrt( (POS_e4(1,1)-shikaku1(1,1))^2 + (POS_e4(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_right4(1,1) = 111;
 elseif (a_12_2*POS_e4(1,1)+b_12_2) <= POS_e4(2,1)  &&  (a_23_2*POS_e4(1,1)+b_23_2) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_12_1*POS_e4(1,1)+b_12_1)  &&  POS_e4(2,1) <= (a_23_1*POS_e4(1,1)+b_23_1)  &&  sqrt( (POS_e4(1,1)-shikaku2(1,1))^2 + (POS_e4(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_right4(2,2) = 111;
 elseif (a_34_1*POS_e4(1,1)+b_34_1) <= POS_e4(2,1)  &&  (a_23_2*POS_e4(1,1)+b_23_2) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_34_2*POS_e4(1,1)+b_34_2)  &&  POS_e4(2,1) <= (a_23_1*POS_e4(1,1)+b_23_1)  &&  sqrt( (POS_e4(1,1)-shikaku3(1,1))^2 + (POS_e4(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_right4(3,3) = 111; 
 elseif (a_34_1*POS_e4(1,1)+b_34_1) <= POS_e4(2,1)  &&  (a_41_1*POS_e4(1,1)+b_41_1) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_34_2*POS_e4(1,1)+b_34_2)  &&  POS_e4(2,1) <= (a_41_2*POS_e4(1,1)+b_41_2)  &&  sqrt( (POS_e4(1,1)-shikaku4(1,1))^2 + (POS_e4(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_right4(4,4) = 111;
 elseif (a_12_2*POS_e4(1,1)+b_12_2) <= POS_e4(2,1)  &&  (a_23_1*POS_e4(1,1)+b_23_1) < POS_e4(2,1)  &&  POS_e4(2,1) < (a_41_1*POS_e4(1,1)+b_41_1)  &&  POS_e4(2,1) <= ts_geo_center(2,1)  &&  POS_e4(1,1) < ts_geo_center(1,1)
     contact_flag_right4(1,2) = 111;
 elseif (a_23_2*POS_e4(1,1)+b_23_2) <= POS_e4(2,1)  &&  POS_e4(2,1) < (a_34_1*POS_e4(1,1)+b_34_1)  &&  (a_12_1*POS_e4(1,1)+b_12_1) < POS_e4(2,1)  &&  ts_geo_center(1,1) <= POS_e4(1,1)  &&  POS_e4(2,1) < ts_geo_center(2,1)
     contact_flag_right4(2,3) = 111;
 elseif POS_e4(2,1) <= (a_34_2*POS_e4(1,1)+b_34_2)  &&  (a_23_1*POS_e4(1,1)+b_23_1) < POS_e4(2,1)  &&  POS_e4(2,1) < (a_41_1*POS_e4(1,1)+b_41_1)  &&  ts_geo_center(2,1) <= POS_e4(2,1)  &&  ts_geo_center(1,1) < POS_e4(1,1)
     contact_flag_right4(3,4) = 111;
 elseif POS_e4(2,1) <= (a_41_2*POS_e4(1,1)+b_41_2)  &&  POS_e4(2,1) < (a_34_1*POS_e4(1,1)+b_34_1)  &&  (a_12_1*POS_e4(1,1)+b_12_1) < POS_e4(2,1)  &&  POS_e4(1,1) <= ts_geo_center(1,1)  &&  ts_geo_center(2,1) < POS_e4(2,1)
     contact_flag_right4(4,1) = 111;
 else
     contact_flag_right4(5,5) = 111;
 end


% ターゲットの姿勢(q0)が0～π/4のとき
elseif 0 < q0  &&  q0 < pi/4

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

% 左手1の位置
 if     (a_12_2*POS_e1(1,1)+b_12_2) <= POS_e1(2,1)  &&  (a_41_1*POS_e1(1,1)+b_41_1) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_12_1*POS_e1(1,1)+b_12_1)  &&  POS_e1(2,1) <= (a_41_2*POS_e1(1,1)+b_41_2)  &&  sqrt( (POS_e1(1,1)-shikaku1(1,1))^2 + (POS_e1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_left1(1,1) = 111;
 elseif (a_12_2*POS_e1(1,1)+b_12_2) <= POS_e1(2,1)  &&  (a_23_2*POS_e1(1,1)+b_23_2) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_12_1*POS_e1(1,1)+b_12_1)  &&  POS_e1(2,1) <= (a_23_1*POS_e1(1,1)+b_23_1)  &&  sqrt( (POS_e1(1,1)-shikaku2(1,1))^2 + (POS_e1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_left1(2,2) = 111;
 elseif (a_34_1*POS_e1(1,1)+b_34_1) <= POS_e1(2,1)  &&  (a_23_2*POS_e1(1,1)+b_23_2) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_34_2*POS_e1(1,1)+b_34_2)  &&  POS_e1(2,1) <= (a_23_1*POS_e1(1,1)+b_23_1)  &&  sqrt( (POS_e1(1,1)-shikaku3(1,1))^2 + (POS_e1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_left1(3,3) = 111; 
 elseif (a_34_1*POS_e1(1,1)+b_34_1) <= POS_e1(2,1)  &&  (a_41_1*POS_e1(1,1)+b_41_1) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_34_2*POS_e1(1,1)+b_34_2)  &&  POS_e1(2,1) <= (a_41_2*POS_e1(1,1)+b_41_2)  &&  sqrt( (POS_e1(1,1)-shikaku4(1,1))^2 + (POS_e1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_left1(4,4) = 111;
 elseif (a_12_2*POS_e1(1,1)+b_12_2) <= POS_e1(2,1)  &&  (a_23_1*POS_e1(1,1)+b_23_1) < POS_e1(2,1)  &&  POS_e1(2,1) < (a_41_1*POS_e1(1,1)+b_41_1)  &&  POS_e1(2,1) <= (a_13*POS_e1(1,1)+b_13)  &&  (a_24*POS_e1(1,1)+b_24) < POS_e1(2,1)
     contact_flag_left1(1,2) = 111;
 elseif (a_23_2*POS_e1(1,1)+b_23_2) <= POS_e1(2,1)  &&  POS_e1(2,1) < (a_34_1*POS_e1(1,1)+b_34_1)  &&  (a_12_1*POS_e1(1,1)+b_12_1) < POS_e1(2,1)  &&  POS_e1(2,1) <= (a_24*POS_e1(1,1)+b_24)  &&  POS_e1(2,1) < (a_13*POS_e1(1,1)+b_13)
     contact_flag_left1(2,3) = 111;
 elseif POS_e1(2,1) <= (a_34_2*POS_e1(1,1)+b_34_2)  &&  POS_e1(2,1) < (a_41_1*POS_e1(1,1)+b_41_1)  &&  (a_23_1*POS_e1(1,1)+b_23_1) < POS_e1(2,1)  &&  (a_13*POS_e1(1,1)+b_13) <= POS_e1(2,1)  &&  POS_e1(2,1) < (a_24*POS_e1(1,1)+b_24)
     contact_flag_left1(3,4) = 111;
 elseif POS_e1(2,1) <= (a_41_2*POS_e1(1,1)+b_41_2)  &&  (a_12_1*POS_e1(1,1)+b_12_1) < POS_e1(2,1)  &&  POS_e1(2,1) < (a_34_1*POS_e1(1,1)+b_34_1)  &&  (a_24*POS_e1(1,1)+b_24) <= POS_e1(2,1)  &&  (a_13*POS_e1(1,1)+b_13) < POS_e1(2,1)
     contact_flag_left1(4,1) = 111;
 else
     contact_flag_left1(5,5) = 111;
 end
% 左手2の位置
 if     (a_12_2*POS_e2(1,1)+b_12_2) <= POS_e2(2,1)  &&  (a_41_1*POS_e2(1,1)+b_41_1) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_12_1*POS_e2(1,1)+b_12_1)  &&  POS_e2(2,1) <= (a_41_2*POS_e2(1,1)+b_41_2)  &&  sqrt( (POS_e2(1,1)-shikaku1(1,1))^2 + (POS_e2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_left2(1,1) = 111;
 elseif (a_12_2*POS_e2(1,1)+b_12_2) <= POS_e2(2,1)  &&  (a_23_2*POS_e2(1,1)+b_23_2) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_12_1*POS_e2(1,1)+b_12_1)  &&  POS_e2(2,1) <= (a_23_1*POS_e2(1,1)+b_23_1)  &&  sqrt( (POS_e2(1,1)-shikaku2(1,1))^2 + (POS_e2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_left2(2,2) = 111;
 elseif (a_34_1*POS_e2(1,1)+b_34_1) <= POS_e2(2,1)  &&  (a_23_2*POS_e2(1,1)+b_23_2) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_34_2*POS_e2(1,1)+b_34_2)  &&  POS_e2(2,1) <= (a_23_1*POS_e2(1,1)+b_23_1)  &&  sqrt( (POS_e2(1,1)-shikaku3(1,1))^2 + (POS_e2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_left2(3,3) = 111; 
 elseif (a_34_1*POS_e2(1,1)+b_34_1) <= POS_e2(2,1)  &&  (a_41_1*POS_e2(1,1)+b_41_1) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_34_2*POS_e2(1,1)+b_34_2)  &&  POS_e2(2,1) <= (a_41_2*POS_e2(1,1)+b_41_2)  &&  sqrt( (POS_e2(1,1)-shikaku4(1,1))^2 + (POS_e2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_left2(4,4) = 111;
 elseif (a_12_2*POS_e2(1,1)+b_12_2) <= POS_e2(2,1)  &&  (a_23_1*POS_e2(1,1)+b_23_1) < POS_e2(2,1)  &&  POS_e2(2,1) < (a_41_1*POS_e2(1,1)+b_41_1)  &&  POS_e2(2,1) <= (a_13*POS_e2(1,1)+b_13)  &&  (a_24*POS_e2(1,1)+b_24) < POS_e2(2,1)
     contact_flag_left2(1,2) = 111;
 elseif (a_23_2*POS_e2(1,1)+b_23_2) <= POS_e2(2,1)  &&  POS_e2(2,1) < (a_34_1*POS_e2(1,1)+b_34_1)  &&  (a_12_1*POS_e2(1,1)+b_12_1) < POS_e2(2,1)  &&  POS_e2(2,1) <= (a_24*POS_e2(1,1)+b_24)  &&  POS_e2(2,1) < (a_13*POS_e2(1,1)+b_13)
     contact_flag_left2(2,3) = 111;
 elseif POS_e2(2,1) <= (a_34_2*POS_e2(1,1)+b_34_2)  &&  POS_e2(2,1) < (a_41_1*POS_e2(1,1)+b_41_1)  &&  (a_23_1*POS_e2(1,1)+b_23_1) < POS_e2(2,1)  &&  (a_13*POS_e2(1,1)+b_13) <= POS_e2(2,1)  &&  POS_e2(2,1) < (a_24*POS_e2(1,1)+b_24)
     contact_flag_left2(3,4) = 111;
 elseif POS_e2(2,1) <= (a_41_2*POS_e2(1,1)+b_41_2)  &&  (a_12_1*POS_e2(1,1)+b_12_1) < POS_e2(2,1)  &&  POS_e2(2,1) < (a_34_1*POS_e2(1,1)+b_34_1)  &&  (a_24*POS_e2(1,1)+b_24) <= POS_e2(2,1)  &&  (a_13*POS_e2(1,1)+b_13) < POS_e2(2,1)
     contact_flag_left2(4,1) = 111;
 else
     contact_flag_left2(5,5) = 111;
 end

% 右手3の位置
 if     (a_12_2*POS_e3(1,1)+b_12_2) <= POS_e3(2,1)  &&  (a_41_1*POS_e3(1,1)+b_41_1) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_12_1*POS_e3(1,1)+b_12_1)  &&  POS_e3(2,1) <= (a_41_2*POS_e3(1,1)+b_41_2)  &&  sqrt( (POS_e3(1,1)-shikaku1(1,1))^2 + (POS_e3(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_right3(1,1) = 111;
 elseif (a_12_2*POS_e3(1,1)+b_12_2) <= POS_e3(2,1)  &&  (a_23_2*POS_e3(1,1)+b_23_2) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_12_1*POS_e3(1,1)+b_12_1)  &&  POS_e3(2,1) <= (a_23_1*POS_e3(1,1)+b_23_1)  &&  sqrt( (POS_e3(1,1)-shikaku2(1,1))^2 + (POS_e3(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_right3(2,2) = 111;
 elseif (a_34_1*POS_e3(1,1)+b_34_1) <= POS_e3(2,1)  &&  (a_23_2*POS_e3(1,1)+b_23_2) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_34_2*POS_e3(1,1)+b_34_2)  &&  POS_e3(2,1) <= (a_23_1*POS_e3(1,1)+b_23_1)  &&  sqrt( (POS_e3(1,1)-shikaku3(1,1))^2 + (POS_e3(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_right3(3,3) = 111; 
 elseif (a_34_1*POS_e3(1,1)+b_34_1) <= POS_e3(2,1)  &&  (a_41_1*POS_e3(1,1)+b_41_1) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_34_2*POS_e3(1,1)+b_34_2)  &&  POS_e3(2,1) <= (a_41_2*POS_e3(1,1)+b_41_2)  &&  sqrt( (POS_e3(1,1)-shikaku4(1,1))^2 + (POS_e3(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_right3(4,4) = 111;
 elseif (a_12_2*POS_e3(1,1)+b_12_2) <= POS_e3(2,1)  &&  (a_23_1*POS_e3(1,1)+b_23_1) < POS_e3(2,1)  &&  POS_e3(2,1) < (a_41_1*POS_e3(1,1)+b_41_1)  &&  POS_e3(2,1) <= (a_13*POS_e3(1,1)+b_13)  &&  (a_24*POS_e3(1,1)+b_24) < POS_e3(2,1)
     contact_flag_right3(1,2) = 111;
 elseif (a_23_2*POS_e3(1,1)+b_23_2) <= POS_e3(2,1)  &&  POS_e3(2,1) < (a_34_1*POS_e3(1,1)+b_34_1)  &&  (a_12_1*POS_e3(1,1)+b_12_1) < POS_e3(2,1)  &&  POS_e3(2,1) <= (a_24*POS_e3(1,1)+b_24)  &&  POS_e3(2,1) < (a_13*POS_e3(1,1)+b_13)
     contact_flag_right3(2,3) = 111;
 elseif POS_e3(2,1) <= (a_34_2*POS_e3(1,1)+b_34_2)  &&  POS_e3(2,1) < (a_41_1*POS_e3(1,1)+b_41_1)  &&  (a_23_1*POS_e3(1,1)+b_23_1) < POS_e3(2,1)  &&  (a_13*POS_e3(1,1)+b_13) <= POS_e3(2,1)  &&  POS_e3(2,1) < (a_24*POS_e3(1,1)+b_24)
     contact_flag_right3(3,4) = 111;
 elseif POS_e3(2,1) <= (a_41_2*POS_e3(1,1)+b_41_2)  &&  (a_12_1*POS_e3(1,1)+b_12_1) < POS_e3(2,1)  &&  POS_e3(2,1) < (a_34_1*POS_e3(1,1)+b_34_1)  &&  (a_24*POS_e3(1,1)+b_24) <= POS_e3(2,1)  &&  (a_13*POS_e3(1,1)+b_13) < POS_e3(2,1)
     contact_flag_right3(4,1) = 111;
 else
     contact_flag_right3(5,5) = 111;
 end
% 右手4の位置
 if     (a_12_2*POS_e4(1,1)+b_12_2) <= POS_e4(2,1)  &&  (a_41_1*POS_e4(1,1)+b_41_1) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_12_1*POS_e4(1,1)+b_12_1)  &&  POS_e4(2,1) <= (a_41_2*POS_e4(1,1)+b_41_2)  &&  sqrt( (POS_e4(1,1)-shikaku1(1,1))^2 + (POS_e4(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_right4(1,1) = 111;
 elseif (a_12_2*POS_e4(1,1)+b_12_2) <= POS_e4(2,1)  &&  (a_23_2*POS_e4(1,1)+b_23_2) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_12_1*POS_e4(1,1)+b_12_1)  &&  POS_e4(2,1) <= (a_23_1*POS_e4(1,1)+b_23_1)  &&  sqrt( (POS_e4(1,1)-shikaku2(1,1))^2 + (POS_e4(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_right4(2,2) = 111;
 elseif (a_34_1*POS_e4(1,1)+b_34_1) <= POS_e4(2,1)  &&  (a_23_2*POS_e4(1,1)+b_23_2) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_34_2*POS_e4(1,1)+b_34_2)  &&  POS_e4(2,1) <= (a_23_1*POS_e4(1,1)+b_23_1)  &&  sqrt( (POS_e4(1,1)-shikaku3(1,1))^2 + (POS_e4(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_right4(3,3) = 111; 
 elseif (a_34_1*POS_e4(1,1)+b_34_1) <= POS_e4(2,1)  &&  (a_41_1*POS_e4(1,1)+b_41_1) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_34_2*POS_e4(1,1)+b_34_2)  &&  POS_e4(2,1) <= (a_41_2*POS_e4(1,1)+b_41_2)  &&  sqrt( (POS_e4(1,1)-shikaku4(1,1))^2 + (POS_e4(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_right4(4,4) = 111;
 elseif (a_12_2*POS_e4(1,1)+b_12_2) <= POS_e4(2,1)  &&  (a_23_1*POS_e4(1,1)+b_23_1) < POS_e4(2,1)  &&  POS_e4(2,1) < (a_41_1*POS_e4(1,1)+b_41_1)  &&  POS_e4(2,1) <= (a_13*POS_e4(1,1)+b_13)  &&  (a_24*POS_e4(1,1)+b_24) < POS_e4(2,1)
     contact_flag_right4(1,2) = 111;
 elseif (a_23_2*POS_e4(1,1)+b_23_2) <= POS_e4(2,1)  &&  POS_e4(2,1) < (a_34_1*POS_e4(1,1)+b_34_1)  &&  (a_12_1*POS_e4(1,1)+b_12_1) < POS_e4(2,1)  &&  POS_e4(2,1) <= (a_24*POS_e4(1,1)+b_24)  &&  POS_e4(2,1) < (a_13*POS_e4(1,1)+b_13)
     contact_flag_right4(2,3) = 111;
 elseif POS_e4(2,1) <= (a_34_2*POS_e4(1,1)+b_34_2)  &&  POS_e4(2,1) < (a_41_1*POS_e4(1,1)+b_41_1)  &&  (a_23_1*POS_e4(1,1)+b_23_1) < POS_e4(2,1)  &&  (a_13*POS_e4(1,1)+b_13) <= POS_e4(2,1)  &&  POS_e4(2,1) < (a_24*POS_e4(1,1)+b_24)
     contact_flag_right4(3,4) = 111;
 elseif POS_e4(2,1) <= (a_41_2*POS_e4(1,1)+b_41_2)  &&  (a_12_1*POS_e4(1,1)+b_12_1) < POS_e4(2,1)  &&  POS_e4(2,1) < (a_34_1*POS_e4(1,1)+b_34_1)  &&  (a_24*POS_e4(1,1)+b_24) <= POS_e4(2,1)  &&  (a_13*POS_e4(1,1)+b_13) < POS_e4(2,1)
     contact_flag_right4(4,1) = 111;
 else
     contact_flag_right4(5,5) = 111;
 end


% ターゲットの姿勢(q0)がπ/4<～<π/2のとき   % elseif pi/4 < q0 < pi/2
else

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

% 左手1の位置
 if     (a_12_2*POS_e1(1,1)+b_12_2) <= POS_e1(2,1)  &&  (a_41_1*POS_e1(1,1)+b_41_1) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_12_1*POS_e1(1,1)+b_12_1)  &&  POS_e1(2,1) <= (a_41_2*POS_e1(1,1)+b_41_2)  &&  sqrt( (POS_e1(1,1)-shikaku1(1,1))^2 + (POS_e1(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_left1(1,1) = 111;
 elseif (a_12_2*POS_e1(1,1)+b_12_2) <= POS_e1(2,1)  &&  (a_23_2*POS_e1(1,1)+b_23_2) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_12_1*POS_e1(1,1)+b_12_1)  &&  POS_e1(2,1) <= (a_23_1*POS_e1(1,1)+b_23_1)  &&  sqrt( (POS_e1(1,1)-shikaku2(1,1))^2 + (POS_e1(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_left1(2,2) = 111;
 elseif (a_34_1*POS_e1(1,1)+b_34_1) <= POS_e1(2,1)  &&  (a_23_2*POS_e1(1,1)+b_23_2) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_34_2*POS_e1(1,1)+b_34_2)  &&  POS_e1(2,1) <= (a_23_1*POS_e1(1,1)+b_23_1)  &&  sqrt( (POS_e1(1,1)-shikaku3(1,1))^2 + (POS_e1(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_left1(3,3) = 111; 
 elseif (a_34_1*POS_e1(1,1)+b_34_1) <= POS_e1(2,1)  &&  (a_41_1*POS_e1(1,1)+b_41_1) <= POS_e1(2,1)  &&  POS_e1(2,1) <= (a_34_2*POS_e1(1,1)+b_34_2)  &&  POS_e1(2,1) <= (a_41_2*POS_e1(1,1)+b_41_2)  &&  sqrt( (POS_e1(1,1)-shikaku4(1,1))^2 + (POS_e1(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_left1(4,4) = 111;
 elseif (a_12_2*POS_e1(1,1)+b_12_2) <= POS_e1(2,1)  &&  (a_23_1*POS_e1(1,1)+b_23_1) < POS_e1(2,1)  &&  POS_e1(2,1) < (a_41_1*POS_e1(1,1)+b_41_1)  &&  POS_e1(2,1) <= (a_13*POS_e1(1,1)+b_13)  &&  POS_e1(2,1) < (a_24*POS_e1(1,1)+b_24)
     contact_flag_left1(1,2) = 111;
 elseif (a_23_2*POS_e1(1,1)+b_23_2) <= POS_e1(2,1)  &&  POS_e1(2,1) < (a_34_1*POS_e1(1,1)+b_34_1)  &&  (a_12_1*POS_e1(1,1)+b_12_1) < POS_e1(2,1)  &&  (a_24*POS_e1(1,1)+b_24) <= POS_e1(2,1)  &&  POS_e1(2,1) < (a_13*POS_e1(1,1)+b_13)
     contact_flag_left1(2,3) = 111;
 elseif POS_e1(2,1) <= (a_34_2*POS_e1(1,1)+b_34_2)  &&  POS_e1(2,1) < (a_41_1*POS_e1(1,1)+b_41_1)  &&  (a_23_1*POS_e1(1,1)+b_23_1) < POS_e1(2,1)  &&  (a_13*POS_e1(1,1)+b_13) <= POS_e1(2,1)  &&  (a_24*POS_e1(1,1)+b_24) < POS_e1(2,1)
     contact_flag_left1(3,4) = 111;
 elseif POS_e1(2,1) <= (a_41_2*POS_e1(1,1)+b_41_2)  &&  (a_12_1*POS_e1(1,1)+b_12_1) < POS_e1(2,1)  &&  POS_e1(2,1) < (a_34_1*POS_e1(1,1)+b_34_1)  &&  POS_e1(2,1) <= (a_24*POS_e1(1,1)+b_24)  &&  (a_13*POS_e1(1,1)+b_13) < POS_e1(2,1)
     contact_flag_left1(4,1) = 111;
 else
     contact_flag_left1(5,5) = 111;
 end
% 左手2の位置
 if     (a_12_2*POS_e2(1,1)+b_12_2) <= POS_e2(2,1)  &&  (a_41_1*POS_e2(1,1)+b_41_1) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_12_1*POS_e2(1,1)+b_12_1)  &&  POS_e2(2,1) <= (a_41_2*POS_e2(1,1)+b_41_2)  &&  sqrt( (POS_e2(1,1)-shikaku1(1,1))^2 + (POS_e2(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_left2(1,1) = 111;
 elseif (a_12_2*POS_e2(1,1)+b_12_2) <= POS_e2(2,1)  &&  (a_23_2*POS_e2(1,1)+b_23_2) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_12_1*POS_e2(1,1)+b_12_1)  &&  POS_e2(2,1) <= (a_23_1*POS_e2(1,1)+b_23_1)  &&  sqrt( (POS_e2(1,1)-shikaku2(1,1))^2 + (POS_e2(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_left2(2,2) = 111;
 elseif (a_34_1*POS_e2(1,1)+b_34_1) <= POS_e2(2,1)  &&  (a_23_2*POS_e2(1,1)+b_23_2) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_34_2*POS_e2(1,1)+b_34_2)  &&  POS_e2(2,1) <= (a_23_1*POS_e2(1,1)+b_23_1)  &&  sqrt( (POS_e2(1,1)-shikaku3(1,1))^2 + (POS_e2(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_left2(3,3) = 111; 
 elseif (a_34_1*POS_e2(1,1)+b_34_1) <= POS_e2(2,1)  &&  (a_41_1*POS_e2(1,1)+b_41_1) <= POS_e2(2,1)  &&  POS_e2(2,1) <= (a_34_2*POS_e2(1,1)+b_34_2)  &&  POS_e2(2,1) <= (a_41_2*POS_e2(1,1)+b_41_2)  &&  sqrt( (POS_e2(1,1)-shikaku4(1,1))^2 + (POS_e2(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_left2(4,4) = 111;
 elseif (a_12_2*POS_e2(1,1)+b_12_2) <= POS_e2(2,1)  &&  (a_23_1*POS_e2(1,1)+b_23_1) < POS_e2(2,1)  &&  POS_e2(2,1) < (a_41_1*POS_e2(1,1)+b_41_1)  &&  POS_e2(2,1) <= (a_13*POS_e2(1,1)+b_13)  &&  POS_e2(2,1) < (a_24*POS_e2(1,1)+b_24)
     contact_flag_left2(1,2) = 111;
 elseif (a_23_2*POS_e2(1,1)+b_23_2) <= POS_e2(2,1)  &&  POS_e2(2,1) < (a_34_1*POS_e2(1,1)+b_34_1)  &&  (a_12_1*POS_e2(1,1)+b_12_1) < POS_e2(2,1)  &&  (a_24*POS_e2(1,1)+b_24) <= POS_e2(2,1)  &&  POS_e2(2,1) < (a_13*POS_e2(1,1)+b_13)
     contact_flag_left2(2,3) = 111;
 elseif POS_e2(2,1) <= (a_34_2*POS_e2(1,1)+b_34_2)  &&  POS_e2(2,1) < (a_41_1*POS_e2(1,1)+b_41_1)  &&  (a_23_1*POS_e2(1,1)+b_23_1) < POS_e2(2,1)  &&  (a_13*POS_e2(1,1)+b_13) <= POS_e2(2,1)  &&  (a_24*POS_e2(1,1)+b_24) < POS_e2(2,1)
     contact_flag_left2(3,4) = 111;
 elseif POS_e2(2,1) <= (a_41_2*POS_e2(1,1)+b_41_2)  &&  (a_12_1*POS_e2(1,1)+b_12_1) < POS_e2(2,1)  &&  POS_e2(2,1) < (a_34_1*POS_e2(1,1)+b_34_1)  &&  POS_e2(2,1) <= (a_24*POS_e2(1,1)+b_24)  &&  (a_13*POS_e2(1,1)+b_13) < POS_e2(2,1)
     contact_flag_left2(4,1) = 111;
 else
     contact_flag_left2(5,5) = 111;
 end

% 右手3の位置
 if     (a_12_2*POS_e3(1,1)+b_12_2) <= POS_e3(2,1)  &&  (a_41_1*POS_e3(1,1)+b_41_1) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_12_1*POS_e3(1,1)+b_12_1)  &&  POS_e3(2,1) <= (a_41_2*POS_e3(1,1)+b_41_2)  &&  sqrt( (POS_e3(1,1)-shikaku1(1,1))^2 + (POS_e3(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_right3(1,1) = 111;
 elseif (a_12_2*POS_e3(1,1)+b_12_2) <= POS_e3(2,1)  &&  (a_23_2*POS_e3(1,1)+b_23_2) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_12_1*POS_e3(1,1)+b_12_1)  &&  POS_e3(2,1) <= (a_23_1*POS_e3(1,1)+b_23_1)  &&  sqrt( (POS_e3(1,1)-shikaku2(1,1))^2 + (POS_e3(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_right3(2,2) = 111;
 elseif (a_34_1*POS_e3(1,1)+b_34_1) <= POS_e3(2,1)  &&  (a_23_2*POS_e3(1,1)+b_23_2) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_34_2*POS_e3(1,1)+b_34_2)  &&  POS_e3(2,1) <= (a_23_1*POS_e3(1,1)+b_23_1)  &&  sqrt( (POS_e3(1,1)-shikaku3(1,1))^2 + (POS_e3(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_right3(3,3) = 111; 
 elseif (a_34_1*POS_e3(1,1)+b_34_1) <= POS_e3(2,1)  &&  (a_41_1*POS_e3(1,1)+b_41_1) <= POS_e3(2,1)  &&  POS_e3(2,1) <= (a_34_2*POS_e3(1,1)+b_34_2)  &&  POS_e3(2,1) <= (a_41_2*POS_e3(1,1)+b_41_2)  &&  sqrt( (POS_e3(1,1)-shikaku4(1,1))^2 + (POS_e3(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_right3(4,4) = 111;
 elseif (a_12_2*POS_e3(1,1)+b_12_2) <= POS_e3(2,1)  &&  (a_23_1*POS_e3(1,1)+b_23_1) < POS_e3(2,1)  &&  POS_e3(2,1) < (a_41_1*POS_e3(1,1)+b_41_1)  &&  POS_e3(2,1) <= (a_13*POS_e3(1,1)+b_13)  &&  POS_e3(2,1) < (a_24*POS_e3(1,1)+b_24)
     contact_flag_right3(1,2) = 111;
 elseif (a_23_2*POS_e3(1,1)+b_23_2) <= POS_e3(2,1)  &&  POS_e3(2,1) < (a_34_1*POS_e3(1,1)+b_34_1)  &&  (a_12_1*POS_e3(1,1)+b_12_1) < POS_e3(2,1)  &&  (a_24*POS_e3(1,1)+b_24) <= POS_e3(2,1)  &&  POS_e3(2,1) < (a_13*POS_e3(1,1)+b_13)
     contact_flag_right3(2,3) = 111;
 elseif POS_e3(2,1) <= (a_34_2*POS_e3(1,1)+b_34_2)  &&  POS_e3(2,1) < (a_41_1*POS_e3(1,1)+b_41_1)  &&  (a_23_1*POS_e3(1,1)+b_23_1) < POS_e3(2,1)  &&  (a_13*POS_e3(1,1)+b_13) <= POS_e3(2,1)  &&  (a_24*POS_e3(1,1)+b_24) < POS_e3(2,1)
     contact_flag_right3(3,4) = 111;
 elseif POS_e3(2,1) <= (a_41_2*POS_e3(1,1)+b_41_2)  &&  (a_12_1*POS_e3(1,1)+b_12_1) < POS_e3(2,1)  &&  POS_e3(2,1) < (a_34_1*POS_e3(1,1)+b_34_1)  &&  POS_e3(2,1) <= (a_24*POS_e3(1,1)+b_24)  &&  (a_13*POS_e3(1,1)+b_13) < POS_e3(2,1)
     contact_flag_right3(4,1) = 111;
 else
     contact_flag_right3(5,5) = 111;
 end
% 右手4の位置
 if     (a_12_2*POS_e4(1,1)+b_12_2) <= POS_e4(2,1)  &&  (a_41_1*POS_e4(1,1)+b_41_1) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_12_1*POS_e4(1,1)+b_12_1)  &&  POS_e4(2,1) <= (a_41_2*POS_e4(1,1)+b_41_2)  &&  sqrt( (POS_e4(1,1)-shikaku1(1,1))^2 + (POS_e4(2,1)-shikaku1(2,1))^2 ) <= r_tip
     contact_flag_right4(1,1) = 111;
 elseif (a_12_2*POS_e4(1,1)+b_12_2) <= POS_e4(2,1)  &&  (a_23_2*POS_e4(1,1)+b_23_2) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_12_1*POS_e4(1,1)+b_12_1)  &&  POS_e4(2,1) <= (a_23_1*POS_e4(1,1)+b_23_1)  &&  sqrt( (POS_e4(1,1)-shikaku2(1,1))^2 + (POS_e4(2,1)-shikaku2(2,1))^2 ) <= r_tip
     contact_flag_right4(2,2) = 111;
 elseif (a_34_1*POS_e4(1,1)+b_34_1) <= POS_e4(2,1)  &&  (a_23_2*POS_e4(1,1)+b_23_2) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_34_2*POS_e4(1,1)+b_34_2)  &&  POS_e4(2,1) <= (a_23_1*POS_e4(1,1)+b_23_1)  &&  sqrt( (POS_e4(1,1)-shikaku3(1,1))^2 + (POS_e4(2,1)-shikaku3(2,1))^2 ) <= r_tip
     contact_flag_right4(3,3) = 111; 
 elseif (a_34_1*POS_e4(1,1)+b_34_1) <= POS_e4(2,1)  &&  (a_41_1*POS_e4(1,1)+b_41_1) <= POS_e4(2,1)  &&  POS_e4(2,1) <= (a_34_2*POS_e4(1,1)+b_34_2)  &&  POS_e4(2,1) <= (a_41_2*POS_e4(1,1)+b_41_2)  &&  sqrt( (POS_e4(1,1)-shikaku4(1,1))^2 + (POS_e4(2,1)-shikaku4(2,1))^2 ) <= r_tip
     contact_flag_right4(4,4) = 111;
 elseif (a_12_2*POS_e4(1,1)+b_12_2) <= POS_e4(2,1)  &&  (a_23_1*POS_e4(1,1)+b_23_1) < POS_e4(2,1)  &&  POS_e4(2,1) < (a_41_1*POS_e4(1,1)+b_41_1)  &&  POS_e4(2,1) <= (a_13*POS_e4(1,1)+b_13)  &&  POS_e4(2,1) < (a_24*POS_e4(1,1)+b_24)
     contact_flag_right4(1,2) = 111;
 elseif (a_23_2*POS_e4(1,1)+b_23_2) <= POS_e4(2,1)  &&  POS_e4(2,1) < (a_34_1*POS_e4(1,1)+b_34_1)  &&  (a_12_1*POS_e4(1,1)+b_12_1) < POS_e4(2,1)  &&  (a_24*POS_e4(1,1)+b_24) <= POS_e4(2,1)  &&  POS_e4(2,1) < (a_13*POS_e4(1,1)+b_13)
     contact_flag_right4(2,3) = 111;
 elseif POS_e4(2,1) <= (a_34_2*POS_e4(1,1)+b_34_2)  &&  POS_e4(2,1) < (a_41_1*POS_e4(1,1)+b_41_1)  &&  (a_23_1*POS_e4(1,1)+b_23_1) < POS_e4(2,1)  &&  (a_13*POS_e4(1,1)+b_13) <= POS_e4(2,1)  &&  (a_24*POS_e4(1,1)+b_24) < POS_e4(2,1)
     contact_flag_right4(3,4) = 111;
 elseif POS_e4(2,1) <= (a_41_2*POS_e4(1,1)+b_41_2)  &&  (a_12_1*POS_e4(1,1)+b_12_1) < POS_e4(2,1)  &&  POS_e4(2,1) < (a_34_1*POS_e4(1,1)+b_34_1)  &&  POS_e4(2,1) <= (a_24*POS_e4(1,1)+b_24)  &&  (a_13*POS_e4(1,1)+b_13) < POS_e4(2,1)
     contact_flag_right4(4,1) = 111;
 else
     contact_flag_right4(5,5) = 111;
 end


end


end