function [ contactflag, Contact_point, D_close, normal_vector, relative, contactsurf_jNo, curPosAP33] ... 
= Box_SixTips_CONTACT_FLAG_v05( SV_ts, d_time, r_tip, halftarget, halftargetz, POS_ee, Surfabc, distance_tmp, tsj, TB0)

%初期化
% POS_ee = zeros(3,1); %手先球位置　POS_ee1〜POS_ee6が入る

% distance = zeros(1,6);
% distance_min = zeros(1,1);
% distance_vel = zeros(1,6);
% Contact_point_A = zeros(3,6);
% Contact_point_B = zeros(3,6);
% relative_A = zeros(3,6);
% relative_B = zeros(3,6);
% Contact_point = zeros(3,6);
% normal_vector = ones(3,6);
% relative = zeros(3,6);
% ContactP = zeros(3,1);
% dj = zeros(1,1);
% sameside_jflag = zeros(1,6);
% D_close = 100;

%接触判定
%手先１つを固定　
for j = 1:6
    
        dj(1,j) = - Surfabc(1,j) * tsj(1,j) - Surfabc(2,j) * tsj(2,j) - Surfabc(3,j) * tsj(3,j);
        distance(1,j) =  abs( Surfabc(1,j) * POS_ee(1,1) + Surfabc(2,j) * POS_ee(2,1) + Surfabc(3,j) * POS_ee(3,1) + dj(1,j) ) / sqrt( (Surfabc(1,j))^2 + (Surfabc(2,j))^2 + (Surfabc(3,j))^2 );

%         distance(i,j) =  abs(Surfabc(1,j) * POS_ee(1,i) + Surfabc(2,j) * POS_ee(2,i) + Surfabc(3,j) * POS_ee(3,i) - 1) / sqrt((Surfabc(1,j))^2 + (Surfabc(2,j))^2 + (Surfabc(3,j))^2);       
        if distance(1,j) <= r_tip 
            %一つのエンドエフェクタ球、一つの平面を決めたときの、接触位置の原点座標（Contact_point）とターゲット中心からみた位置座標（relative　＝ターゲットから接触点までの距離）の計算
            Contact_point_A(1:3,j) = POS_ee(1:3,1) + distance(1,j) / norm(Surfabc(1:3,j)) * Surfabc(1:3,j);
            Contact_point_B(1:3,j) = POS_ee(1:3,1) - distance(1,j) / norm(Surfabc(1:3,j)) * Surfabc(1:3,j);
            relative_A(1:3,j) = SV_ts.A0 \ (Contact_point_A(1:3,j) - SV_ts.R0); 
            relative_B(1:3,j) = SV_ts.A0 \ (Contact_point_B(1:3,j) - SV_ts.R0);
            %%%接触確認
            %　A&&B　論理AND演算（Aが偽ならBの評価はされない、AかつB）　　 A||B　論理OR演算（Aが真ならBの評価はされない）
            %if文内容  ⇒　四角ターゲットの領域内に接触点の相対座標があるかどうか(xyzそれぞれ)　を不等式で表している            
            if   -halftarget <= relative_A(1,j) && relative_A(1,j) <= halftarget       &&    -halftarget <= relative_A(2,j) && relative_A(2,j) <= halftarget    &&    -halftargetz <= relative_A(3,j) && relative_A(3,j) <= halftargetz %定義域確認，法線ベクトル，接触点定義
                Contact_point(1:3,j) = Contact_point_A(1:3,j);
                normal_vector(1:3,j) = Surfabc(1:3,j);%法線ベクトルは手先から平面へ
                relative(1:3,j) = relative_A(1:3,j);
                contactflag_j(j) = 1;

            elseif -halftarget <= relative_B(1,j) && relative_B(1,j) <= halftarget     &&    -halftarget <= relative_B(2,j) && relative_B(2,j) <= halftarget    &&    -halftargetz <= relative_B(3,j) && relative_B(3,j) <= halftargetz
                Contact_point(1:3,j) = Contact_point_B(1:3,j);      %逆側の接触点
                normal_vector(1:3,j) = Surfabc(1:3,j);               %逆方向のベクトル（手先球中心が平面下にある）
                relative(1:3,j) = relative_B(1:3,j);
                contactflag_j(j) = 1;  %←←←ここほとんど入ってない?

%〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜
%%%角との接触
%不完全
%3面の間（角）、2面の間（辺）との接触。めり込み量の評価。反発力、モデル。
%加えて、球、円柱等の接触の判定（めり込み量の評価）も未完成なので掘り下げるかどうか

%〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜〜                 
            else%%%接触場所が平面の定義域外のelse
                Contact_point(1:3,j) = zeros(3,1);
                normal_vector(1:3,j) = Surfabc(1:3,j);
                relative(1:3,j) = zeros(3,1);
                contactflag_j(j) = 0.5;
            end
        else%%%distance>r_tip
            Contact_point(1:3,j) = zeros(3,1);
            normal_vector(1:3,j) = Surfabc(1:3,j);
            relative(1:3,j) = zeros(3,1);
            contactflag_j(j) = 0;
        end
end
        

        if contactflag_j(1) == 1
            contactflag = 1;
            contactsurf_jNo = 1;
            normal_vector(1:3,1) = - normal_vector(1:3,1);
            Contact_point(1:3,1) = Contact_point(1:3,1);
            relative(1:3,1) = relative(1:3,1);
            D_close = distance(1,1);
            
        elseif contactflag_j(2) == 1
            contactflag = 1;
            contactsurf_jNo = 2;
            normal_vector(1:3,1) = normal_vector(1:3,2);
            Contact_point(1:3,1) = Contact_point(1:3,2);
            relative(1:3,1) = relative(1:3,2);
            D_close = distance(1,2);
            
        elseif contactflag_j(3) == 1
            contactflag = 1;
            contactsurf_jNo = 3;
            normal_vector(1:3,1) = - normal_vector(1:3,3);
            Contact_point(1:3,1) = Contact_point(1:3,3);
            relative(1:3,1) = relative(1:3,3);
            D_close = distance(1,3);
            
        elseif contactflag_j(4) == 1
            contactflag = 1;
            contactsurf_jNo = 4;
            normal_vector(1:3,1) =  normal_vector(1:3,4);
            Contact_point(1:3,1) = Contact_point(1:3,4);
            relative(1:3,1) = relative(1:3,4);
            D_close = distance(1,4);
            
        elseif contactflag_j(5) == 1
            contactflag = 1;
            contactsurf_jNo = 5;
            normal_vector(1:3,1) = - normal_vector(1:3,5);
            Contact_point(1:3,1) = Contact_point(1:3,5);
            relative(1:3,1) = relative(1:3,5);
            D_close = distance(1,5);
            
        elseif contactflag_j(6) == 1
            contactflag = 1;
            contactsurf_jNo = 6;
            normal_vector(1:3,1) =  normal_vector(1:3,6);
            Contact_point(1:3,1) = Contact_point(1:3,6);
            relative(1:3,1) = relative(1:3,6);
            D_close = distance(1,6);
            
        %例外    
        elseif  norm(contactflag_j) > 1
            contactflag = 0.3;
            contactsurf_jNo = 9;
            normal_vector(1:3,1) = [0.001 0.0001 0.0001]';
            Contact_point(1:3,1) = zeros(3,1);
            relative(1:3,1) = zeros(3,1);
            D_close = 9;
        else
            contactflag = 0;
            contactsurf_jNo = 0;
            normal_vector(1:3,1) = [0.001 0.0001 0.0001]';
            Contact_point(1:3,1) = zeros(3,1);
            relative(1:3,1) = zeros(3,1);
            D_close = 9;
        end

%     curPosAP3_L = curPosAP3_L - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
%     TB0_L = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る　SV.A0
%     curPosAP3_LL = TB0_L * curPosAP3_L;   % 接触位置をターゲット重心座標に合わせ回転
%     curPosAP33_L = tilde( curPosAP3_LL );   % 交代行列を作る(転置がその-1倍となる行列)
    curPosAP3 = Contact_point - SV_ts.R0;   % APを使って慣性座標中心をターゲット重心座標中心に替える
%     TB0 = rpy2dc( SV_ts.Q0 );   % ターゲット回転角の方向余弦行列を作る　SV.A0
    curPosAP3_2 = TB0 * curPosAP3;   % 接触位置をターゲット重心座標に合わせ回転
    curPosAP33 = tilde( curPosAP3_2 );   % 交代行列を作る(転置がその-1倍となる行列)

    
% %次の関数で delta（めり込み量）評価に使う D_close の計算    
% distance_min = min(distance(1,1:6));
% D_close = distance_min;
% distance_tmp = distance(1,1:6);

end