function [ contactflag, Contact_point, D_close, normal_vector, relative, contactsurf_jNo, curPosAP33] ... 
= Box_SixTips_CONTACT_FLAG_v05( SV_ts, d_time, r_tip, halftarget, halftargetz, POS_ee, Surfabc, distance_tmp, tsj, TB0)

%������
% POS_ee = zeros(3,1); %��担�ʒu�@POS_ee1�`POS_ee6������

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

%�ڐG����
%���P���Œ�@
for j = 1:6
    
        dj(1,j) = - Surfabc(1,j) * tsj(1,j) - Surfabc(2,j) * tsj(2,j) - Surfabc(3,j) * tsj(3,j);
        distance(1,j) =  abs( Surfabc(1,j) * POS_ee(1,1) + Surfabc(2,j) * POS_ee(2,1) + Surfabc(3,j) * POS_ee(3,1) + dj(1,j) ) / sqrt( (Surfabc(1,j))^2 + (Surfabc(2,j))^2 + (Surfabc(3,j))^2 );

%         distance(i,j) =  abs(Surfabc(1,j) * POS_ee(1,i) + Surfabc(2,j) * POS_ee(2,i) + Surfabc(3,j) * POS_ee(3,i) - 1) / sqrt((Surfabc(1,j))^2 + (Surfabc(2,j))^2 + (Surfabc(3,j))^2);       
        if distance(1,j) <= r_tip 
            %��̃G���h�G�t�F�N�^���A��̕��ʂ����߂��Ƃ��́A�ڐG�ʒu�̌��_���W�iContact_point�j�ƃ^�[�Q�b�g���S����݂��ʒu���W�irelative�@���^�[�Q�b�g����ڐG�_�܂ł̋����j�̌v�Z
            Contact_point_A(1:3,j) = POS_ee(1:3,1) + distance(1,j) / norm(Surfabc(1:3,j)) * Surfabc(1:3,j);
            Contact_point_B(1:3,j) = POS_ee(1:3,1) - distance(1,j) / norm(Surfabc(1:3,j)) * Surfabc(1:3,j);
            relative_A(1:3,j) = SV_ts.A0 \ (Contact_point_A(1:3,j) - SV_ts.R0); 
            relative_B(1:3,j) = SV_ts.A0 \ (Contact_point_B(1:3,j) - SV_ts.R0);
            %%%�ڐG�m�F
            %�@A&&B�@�_��AND���Z�iA���U�Ȃ�B�̕]���͂���Ȃ��AA����B�j�@�@ A||B�@�_��OR���Z�iA���^�Ȃ�B�̕]���͂���Ȃ��j
            %if�����e  �ˁ@�l�p�^�[�Q�b�g�̗̈���ɐڐG�_�̑��΍��W�����邩�ǂ���(xyz���ꂼ��)�@��s�����ŕ\���Ă���            
            if   -halftarget <= relative_A(1,j) && relative_A(1,j) <= halftarget       &&    -halftarget <= relative_A(2,j) && relative_A(2,j) <= halftarget    &&    -halftargetz <= relative_A(3,j) && relative_A(3,j) <= halftargetz %��`��m�F�C�@���x�N�g���C�ڐG�_��`
                Contact_point(1:3,j) = Contact_point_A(1:3,j);
                normal_vector(1:3,j) = Surfabc(1:3,j);%�@���x�N�g���͎�悩�畽�ʂ�
                relative(1:3,j) = relative_A(1:3,j);
                contactflag_j(j) = 1;

            elseif -halftarget <= relative_B(1,j) && relative_B(1,j) <= halftarget     &&    -halftarget <= relative_B(2,j) && relative_B(2,j) <= halftarget    &&    -halftargetz <= relative_B(3,j) && relative_B(3,j) <= halftargetz
                Contact_point(1:3,j) = Contact_point_B(1:3,j);      %�t���̐ڐG�_
                normal_vector(1:3,j) = Surfabc(1:3,j);               %�t�����̃x�N�g���i��担���S�����ʉ��ɂ���j
                relative(1:3,j) = relative_B(1:3,j);
                contactflag_j(j) = 1;  %�����������قƂ�Ǔ����ĂȂ�?

%�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`
%%%�p�Ƃ̐ڐG
%�s���S
%3�ʂ̊ԁi�p�j�A2�ʂ̊ԁi�Ӂj�Ƃ̐ڐG�B�߂荞�ݗʂ̕]���B�����́A���f���B
%�����āA���A�~�����̐ڐG�̔���i�߂荞�ݗʂ̕]���j���������Ȃ̂Ō@�艺���邩�ǂ���

%�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`�`                 
            else%%%�ڐG�ꏊ�����ʂ̒�`��O��else
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
            
        %��O    
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

%     curPosAP3_L = curPosAP3_L - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
%     TB0_L = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������@SV.A0
%     curPosAP3_LL = TB0_L * curPosAP3_L;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
%     curPosAP33_L = tilde( curPosAP3_LL );   % ���s������(�]�u������-1�{�ƂȂ�s��)
    curPosAP3 = Contact_point - SV_ts.R0;   % AP���g���Ċ������W���S���^�[�Q�b�g�d�S���W���S�ɑւ���
%     TB0 = rpy2dc( SV_ts.Q0 );   % �^�[�Q�b�g��]�p�̕����]���s������@SV.A0
    curPosAP3_2 = TB0 * curPosAP3;   % �ڐG�ʒu���^�[�Q�b�g�d�S���W�ɍ��킹��]
    curPosAP33 = tilde( curPosAP3_2 );   % ���s������(�]�u������-1�{�ƂȂ�s��)

    
% %���̊֐��� delta�i�߂荞�ݗʁj�]���Ɏg�� D_close �̌v�Z    
% distance_min = min(distance(1,1:6));
% D_close = distance_min;
% distance_tmp = distance(1,1:6);

end