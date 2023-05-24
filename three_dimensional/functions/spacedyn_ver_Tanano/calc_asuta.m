function [H_asuta, C_asuta]=calc_asuta(LP,SV)

HH=calc_hh(LP,SV);
Hb=HH(1:6,1:6);
Hbm=HH(1:6,7:6+LP.num_q);
Hm=HH(7:6+LP.num_q,7:6+LP.num_q);
H_asuta=Hm-Hbm'*pinv(Hb)*Hbm;
CC=calc_cc(LP,SV);
cb=CC(1:6);
cm=CC(7:6+LP.num_q);
C_asuta=cm-Hbm'*pinv(Hb)*cb;