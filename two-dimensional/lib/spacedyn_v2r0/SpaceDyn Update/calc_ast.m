function[H_ast, C_ast] = calc_ast(LP,SV)
% �t���[�t���C���O���{�b�g�̉^������������
% �O��(�x�[�X�Ɛ�[��)��0�Ɖ��肵��
%
% |Hb   Hbm| |xb_dd| + |Cb| = |Fb| + |Jb'|Fh
% |Hbm' Hm | |��_dd|   |Cm|   |��|   |Jm'|
%
% ��=H_ast*��dd + c_ast
%        
% H_ast = Hm - Hbm'inv(Hb)Hbm
% C_ast = Cm - Hbm'inv(Hb)Cb

n = LP.num_q;
CC = calc_cc(LP,SV);
HH = calc_hh(LP,SV);

Hb  = HH(1:6,1:6);
Hbm = HH(1:6,7:6+n);
Hm  = HH(7:6+n,7:6+n);

Cb = CC(1:6);
Cm = CC(7:6+n);

H_ast = Hm - Hbm'*inv(Hb)*Hbm;
C_ast = Cm - Hbm'*inv(Hb)*Cb;

end


