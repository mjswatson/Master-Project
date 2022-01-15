function [Fp,Mp]=engine(input,d1,d2,Throttle, pressure, x_cg,m)
if m<=0
    T=0;
else
T=(input.Tvac-(input.An*pressure))*Throttle*input.No_of_engines;
end
Mp_1=[0 T*sin(d2) (-T*sin(d1)*cos(d2))];
Mp=Mp_1*(input.dist_gim-x_cg);
Fp=[cos(d1)*T*cos(d2) T*sin(d1)*cos(d2) T*sin(d1)];
end