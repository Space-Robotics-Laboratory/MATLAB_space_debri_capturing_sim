function [ l_L1, l_L2, l_R1, l_R2, l_surf_L1, l_surf_L2, l_surf_R1, l_surf_R2, contact_flag_L1, contact_flag_L2, contact_flag_R1, contact_flag_R2 ] ...
= Maru_FourTips_CONTACT_FLAG( tm_geo, POS_eL1, POS_eL2, POS_eR1, POS_eR2, l_min, r_target )


%%% ��担���S�ƃ^�[�Q�b�g���S�E�\�ʂ̋���
l_L1 = sqrt( ( tm_geo(1,1) - POS_eL1(1,1) )^2 + ( tm_geo(2,1) - POS_eL1(2,1) )^2 );   % �^�[�Q�b�g���S�����担���S�܂ł̋���
l_surf_L1 = abs( l_L1 - r_target );   % �^�[�Q�b�g�\�ʂ����担���S�܂ł̋���
l_L2 = sqrt( ( tm_geo(1,1) - POS_eL2(1,1) )^2 + ( tm_geo(2,1) - POS_eL2(2,1) )^2 );
l_surf_L2 = abs( l_L2 - r_target );
l_R1 = sqrt( ( tm_geo(1,1) - POS_eR1(1,1) )^2 + ( tm_geo(2,1) - POS_eR1(2,1) )^2 );
l_surf_R1 = abs( l_R1 - r_target );
l_R2 = sqrt( ( tm_geo(1,1) - POS_eR2(1,1) )^2 + ( tm_geo(2,1) - POS_eR2(2,1) )^2 );
l_surf_R2 = abs( l_R2 - r_target );


%%% �ڐG����
if l_L1 <= l_min
   contact_flag_L1 = 1;
else
   contact_flag_L1 = 0;
end
if l_L2 <= l_min
   contact_flag_L2 = 1;
else
   contact_flag_L2 = 0;
end
if l_R1 <= l_min
   contact_flag_R1 = 1;
else
   contact_flag_R1 = 0;
end
if l_R2 <= l_min
   contact_flag_R2 = 1;
else
   contact_flag_R2 = 0;
end


end