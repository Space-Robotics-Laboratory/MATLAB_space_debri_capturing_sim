% ==== f_dyn_abQ_rk.m ====
%
% calculate the forward dynamics based on articulated body algorithms
% Fixed Step Runge-Kutta integration
%
% 2006. 3.30. S.ABIKO

function SV = f_dyn_abQ_rk(LP, AB, SV)

global Gravity d_time

SVt = SV;

% 1st Step
SV.dQtn = w2dQtn( SV.w0, SV.Qtn );
SVt = f_dyn_ab2(LP, AB, SV);

k1_R0  = d_time * SV.v0;
k1_Qtn = d_time * SV.dQtn;
k1_v0  = d_time * SVt.vd0;
k1_w0  = d_time * SVt.wd0;
if LP.num_q ~= 0 
    k1_q  = d_time * SV.qd;
    k1_qd = d_time * SVt.qdd;
end

SVt.R0  = SV.R0 + k1_R0/2;
SVt.Qtn = SV.Qtn + k1_Qtn/2;
SVt.v0  = SV.v0 + k1_v0/2;
SVt.w0  = SV.w0 + k1_w0/2;
if LP.num_q ~= 0 
    SVt.q  = SV.q +  k1_q/2;
    SVt.qd = SV.qd + k1_qd/2;
end
SVt.Qtn = SVt.Qtn/norm(SVt.Qtn);

vd0_tmp = SVt.vd0;
wd0_tmp = SVt.wd0;
if LP.num_q ~= 0 
    qdd_tmp = SVt.qdd;
end

% 2nd Step
SVt.dQtn = w2dQtn( SVt.w0, SVt.Qtn );
SVt = f_dyn_ab2(LP, AB, SVt);

k2_R0  = d_time * SVt.v0;
k2_Qtn = d_time * SVt.dQtn;
k2_v0  = d_time * SVt.vd0;
k2_w0  = d_time * SVt.wd0;
if LP.num_q ~= 0 
    k2_q  = d_time * SVt.qd;
    k2_qd = d_time * SVt.qdd;
end

SVt.R0  = SV.R0 + k2_R0/2;
SVt.Qtn = SV.Qtn + k2_Qtn/2;
SVt.v0  = SV.v0 + k2_v0/2;
SVt.w0  = SV.w0 + k2_w0/2;
if LP.num_q ~= 0 
    SVt.q  = SV.q  + k2_q/2;
    SVt.qd = SV.qd + k2_qd/2;
end
SVt.Qtn = SVt.Qtn/norm(SVt.Qtn);

vd0_tmp = SVt.vd0 + vd0_tmp;
wd0_tmp = SVt.wd0 + wd0_tmp;
if LP.num_q ~= 0 
    qdd_tmp = SVt.qdd + qdd_tmp;
end

% 3rd Step
SVt.dQtn = w2dQtn( SVt.w0, SVt.Qtn );
SVt = f_dyn_ab2(LP, AB, SVt);

k3_R0  = d_time * SVt.v0;
k3_Qtn = d_time * SVt.dQtn;
k3_v0  = d_time * SVt.vd0;
k3_w0  = d_time * SVt.wd0;
if LP.num_q ~= 0 
    k3_q  = d_time * SVt.qd;
    k3_qd = d_time * SVt.qdd;
end

SVt.R0  = SV.R0 + k3_R0;
SVt.Qtn = SV.Qtn + k3_Qtn;
SVt.v0  = SV.v0 + k3_v0;
SVt.w0  = SV.w0 + k3_w0;
if LP.num_q ~= 0 
    SVt.q  = SV.q  + k3_q;
    SVt.qd = SV.qd + k3_qd;
end
SVt.Qtn = SVt.Qtn/norm(SVt.Qtn);

vd0_tmp = SVt.vd0 + vd0_tmp;
wd0_tmp = SVt.wd0 + wd0_tmp;
if LP.num_q ~= 0 
    qdd_tmp = SVt.qdd + qdd_tmp;
end

% 4th Step
SVt.dQtn = w2dQtn( SVt.w0, SVt.Qtn );
SVt = f_dyn_ab2(LP, AB, SVt);

k4_R0  = d_time * SVt.v0;
k4_Qtn = d_time * SVt.dQtn;
k4_v0  = d_time * SVt.vd0;
k4_w0  = d_time * SVt.wd0;
if LP.num_q ~= 0 
    k4_q  = d_time * SVt.qd;
    k4_qd = d_time * SVt.qdd;
end

vd0_tmp = SVt.vd0 + vd0_tmp;
wd0_tmp = SVt.wd0 + wd0_tmp;
if LP.num_q ~= 0 
    qdd_tmp = SVt.qdd + qdd_tmp;
end
SVt.Qtn = SVt.Qtn/norm(SVt.Qtn);

% Solution
SV.R0  = SV.R0  + ( k1_R0 + 2*k2_R0 + 2*k3_R0 + k4_R0 )/6;
SV.Qtn = SV.Qtn + ( k1_Qtn+ 2*k2_Qtn + 2*k3_Qtn + k4_Qtn )/6;
SV.v0  = SV.v0  + ( k1_v0 + 2*k2_v0 + 2*k3_v0 + k4_v0 )/6;
SV.w0  = SV.w0  + ( k1_w0 + 2*k2_w0 + 2*k3_w0 + k4_w0 )/6;
if LP.num_q ~= 0 
    SV.q  = SV.q  + ( k1_q  + 2*k2_q  + 2*k3_q  + k4_q  )/6;
    SV.qd = SV.qd + ( k1_qd + 2*k2_qd + 2*k3_qd + k4_qd )/6;
end
SV.Qtn = SV.Qtn/norm(SV.Qtn);
SV.A0 = qtn2dc( SV.Qtn )';

% === EOF ===
