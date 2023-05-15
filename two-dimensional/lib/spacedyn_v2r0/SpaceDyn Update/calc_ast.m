function[H_ast, C_ast] = calc_ast(LP,SV)
% フリーフライングロボットの運動方程式から
% 外力(ベースと先端に)は0と仮定して
%
% |Hb   Hbm| |xb_dd| + |Cb| = |Fb| + |Jb'|Fh
% |Hbm' Hm | |φ_dd|   |Cm|   |τ|   |Jm'|
%
% τ=H_ast*φdd + c_ast
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


