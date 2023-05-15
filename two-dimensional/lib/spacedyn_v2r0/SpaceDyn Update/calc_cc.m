%%@Cb‚ÆCm‚ğŒvZ‚·‚éŠÖ”

function CC = calc_cc (LP,SV)
n = LP.num_q;
SV_tmp = SV;
SV_tmp.vd0 = zeros(3,1);
SV_tmp.wd0 = zeros(3,1);
SV_tmp.qdd = zeros(n,1);
SV_tmp.Fe = zeros(3,n); 
SV_tmp.Te = zeros(3,n);
CC = r_ne(LP,SV_tmp);
end