function [ contactflag, Contact_point, D_close, normal_vector, relative, contactsurf_jNo, curPosAP33] ... 
= Box_SixTips_CONTACT_FLAG_v05( SV_ts, d_time, r_tip, halftarget, halftargetz, POS_ee, Surfabc, distance_tmp, tsj, TB0)

%‰Šú‰»
% POS_ee = zeros(3,1); %èæ‹…ˆÊ’u@POS_ee1`POS_ee6‚ª“ü‚é

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

%ÚG”»’è
%èæ‚P‚Â‚ğŒÅ’è@
for j = 1:6
    
        dj(1,j) = - Surfabc(1,j) * tsj(1,j) - Surfabc(2,j) * tsj(2,j) - Surfabc(3,j) * tsj(3,j);
        distance(1,j) =  abs( Surfabc(1,j) * POS_ee(1,1) + Surfabc(2,j) * POS_ee(2,1) + Surfabc(3,j) * POS_ee(3,1) + dj(1,j) ) / sqrt( (Surfabc(1,j))^2 + (Surfabc(2,j))^2 + (Surfabc(3,j))^2 );

%         distance(i,j) =  abs(Surfabc(1,j) * POS_ee(1,i) + Surfabc(2,j) * POS_ee(2,i) + Surfabc(3,j) * POS_ee(3,i) - 1) / sqrt((Surfabc(1,j))^2 + (Surfabc(2,j))^2 + (Surfabc(3,j))^2);       
        if distance(1,j) <= r_tip 
            %ˆê‚Â‚ÌƒGƒ“ƒhƒGƒtƒFƒNƒ^‹…Aˆê‚Â‚Ì•½–Ê‚ğŒˆ‚ß‚½‚Æ‚«‚ÌAÚGˆÊ’u‚ÌŒ´“_À•WiContact_pointj‚Æƒ^[ƒQƒbƒg’†S‚©‚ç‚İ‚½ˆÊ’uÀ•Wirelative@ƒ^[ƒQƒbƒg‚©‚çÚG“_‚Ü‚Å‚Ì‹——£j‚ÌŒvZ
            Contact_point_A(1:3,j) = POS_ee(1:3,1) + distance(1,j) / norm(Surfabc(1:3,j)) * Surfabc(1:3,j);
            Contact_point_B(1:3,j) = POS_ee(1:3,1) - distance(1,j) / norm(Surfabc(1:3,j)) * Surfabc(1:3,j);
            relative_A(1:3,j) = SV_ts.A0 \ (Contact_point_A(1:3,j) - SV_ts.R0); 
            relative_B(1:3,j) = SV_ts.A0 \ (Contact_point_B(1:3,j) - SV_ts.R0);
            %%%ÚGŠm”F
            %@A&&B@˜_—AND‰‰ZiA‚ª‹U‚È‚çB‚Ì•]‰¿‚Í‚³‚ê‚È‚¢AA‚©‚ÂBj@@ A||B@˜_—OR‰‰ZiA‚ª^‚È‚çB‚Ì•]‰¿‚Í‚³‚ê‚È‚¢j
            %if•¶“à—e  Ë@lŠpƒ^[ƒQƒbƒg‚Ì—Ìˆæ“à‚ÉÚG“_‚Ì‘Š‘ÎÀ•W‚ª‚ ‚é‚©‚Ç‚¤‚©(xyz‚»‚ê‚¼‚ê)@‚ğ•s“™®‚Å•\‚µ‚Ä‚¢‚é            
            if   -halftarget <= relative_A(1,j) && relative_A(1,j) <= halftarget       &&    -halftarget <= relative_A(2,j) && relative_A(2,j) <= halftarget    &&    -halftargetz <= relative_A(3,j) && relative_A(3,j) <= halftargetz %’è‹`ˆæŠm”FC–@üƒxƒNƒgƒ‹CÚG“_’è‹`
                Contact_point(1:3,j) = Contact_point_A(1:3,j);
                normal_vector(1:3,j) = Surfabc(1:3,j);%–@üƒxƒNƒgƒ‹‚Íèæ‚©‚ç•½–Ê‚Ö
                relative(1:3,j) = relative_A(1:3,j);
                contactflag_j(j) = 1;

            elseif -halftarget <= relative_B(1,j) && relative_B(1,j) <= halftarget     &&    -halftarget <= relative_B(2,j) && relative_B(2,j) <= halftarget    &&    -halftargetz <= relative_B(3,j) && relative_B(3,j) <= halftargetz
                Contact_point(1:3,j) = Contact_point_B(1:3,j);      %‹t‘¤‚ÌÚG“_
                normal_vector(1:3,j) = Surfabc(1:3,j);               %‹t•ûŒü‚ÌƒxƒNƒgƒ‹ièæ‹…’†S‚ª•½–Ê‰º‚É‚ ‚éj
                relative(1:3,j) = relative_B(1:3,j);
                contactflag_j(j) = 1;  %©©©‚±‚±‚Ù‚Æ‚ñ‚Ç“ü‚Á‚Ä‚È‚¢?

%````````````````````````````````````````````````````````````````
%%%Šp‚Æ‚ÌÚG
%•sŠ®‘S
%3–Ê‚ÌŠÔiŠpjA2–Ê‚ÌŠÔi•Ój‚Æ‚ÌÚGB‚ß‚è‚İ—Ê‚Ì•]‰¿B”½”­—ÍAƒ‚ƒfƒ‹B
%‰Á‚¦‚ÄA‹…A‰~’Œ“™‚ÌÚG‚Ì”»’èi‚ß‚è‚İ—Ê‚Ì•]‰¿j‚à–¢Š®¬‚È‚Ì‚ÅŒ@‚è‰º‚°‚é‚©‚Ç‚¤‚©

%````````````````````````````````````````````````````````````````                 
            else%%%ÚGêŠ‚ª•½–Ê‚Ì’è‹`ˆæŠO‚Ìelse
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
            
        %—áŠO    
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

%     curPosAP3_L = curPosAP3_L - SV_ts.R0;   % AP‚ğg‚Á‚ÄŠµ«À•W’†S‚ğƒ^[ƒQƒbƒgdSÀ•W’†S‚É‘Ö‚¦‚é
%     TB0_L = rpy2dc( SV_ts.Q0 );   % ƒ^[ƒQƒbƒg‰ñ“]Šp‚Ì•ûŒü—]Œ·s—ñ‚ğì‚é@SV.A0
%     curPosAP3_LL = TB0_L * curPosAP3_L;   % ÚGˆÊ’u‚ğƒ^[ƒQƒbƒgdSÀ•W‚É‡‚í‚¹‰ñ“]
%     curPosAP33_L = tilde( curPosAP3_LL );   % Œğ‘ãs—ñ‚ğì‚é(“]’u‚ª‚»‚Ì-1”{‚Æ‚È‚és—ñ)
    curPosAP3 = Contact_point - SV_ts.R0;   % AP‚ğg‚Á‚ÄŠµ«À•W’†S‚ğƒ^[ƒQƒbƒgdSÀ•W’†S‚É‘Ö‚¦‚é
%     TB0 = rpy2dc( SV_ts.Q0 );   % ƒ^[ƒQƒbƒg‰ñ“]Šp‚Ì•ûŒü—]Œ·s—ñ‚ğì‚é@SV.A0
    curPosAP3_2 = TB0 * curPosAP3;   % ÚGˆÊ’u‚ğƒ^[ƒQƒbƒgdSÀ•W‚É‡‚í‚¹‰ñ“]
    curPosAP33 = tilde( curPosAP3_2 );   % Œğ‘ãs—ñ‚ğì‚é(“]’u‚ª‚»‚Ì-1”{‚Æ‚È‚és—ñ)

    
% %Ÿ‚ÌŠÖ”‚Å deltai‚ß‚è‚İ—Êj•]‰¿‚Ég‚¤ D_close ‚ÌŒvZ    
% distance_min = min(distance(1,1:6));
% D_close = distance_min;
% distance_tmp = distance(1,1:6);

end