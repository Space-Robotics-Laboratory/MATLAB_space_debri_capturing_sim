function[H_asuta, C_asuta] = calc_asuta(LP, SV)

HH = calc_hh(LP, SV);
CC = calc_cc(LP, SV);

Hb = HH(1:6, 1:6);
Hbm = HH(1:6, 7:6+LP.num_q);
Hm = HH(7:6+LP.num_q, 7:6+LP.num_q);

Cb = CC(1:6, 1);
Cm = CC(7:6+LP.num_q, 1);

H_asuta = Hm - Hbm'*pinv(Hb)*Hbm;
C_asuta = Cm - Hbm'*pinv(Hb)*Cb;

