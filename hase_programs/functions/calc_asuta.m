function[H_asuta, C_asuta] = calc_asuta( LP, SV )

    HH = calc_hh(LP, SV);

    H_b = HH(1:6, 1:6);
    H_b_min = pinv(H_b); 
    H_m = HH(7:end, 7:end);
    H_bm = HH(1:6, 7:end);
    H_bm_ten = H_bm';

    H_asuta =H_m - H_bm_ten * H_b_min * H_bm;

    
    CC = calc_cc( LP, SV );
    
    C_b = CC(1:6, 1:end);
    C_m = CC(7:end, 1:end);
    
    C_asuta = H_bm_ten * H_b_min * C_b + C_m;

end