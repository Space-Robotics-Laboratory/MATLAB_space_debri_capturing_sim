function Rg=calc_Rg(LP,SV)
Rg=zeros(3,1);
for x=1:3
    masx=LP.m0*SV.R0(x,1);
    for i=1:LP.num_q
        masx=masx+LP.m(1,i)*SV.RR(x,i);
    end
    Rg(x,1)=masx/LP.mass;
    masx=0;
end