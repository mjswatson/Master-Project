function [CD,Cl,Cm]=subsonic(alpha,M,Cf,input,x_cg)
% drag coeff
Cdfb=Cf*(1+(60/((input.lb/input.d)^3))+0.0025*(input.lb/input.d))*(input.Sa/input.Sb);
Cdb=(0.029*((input.db/input.d)^3))/sqrt(Cdfb);
CD0=Cdfb+Cdb;
x=input.lb/input.d;
nu=1e-5*x^3-8e-4*x^2+0.0244*x+0.515;
Cdc=input.Cdc(sin(alpha)*M);
CDalpha=2*alpha^2*input.Sb/(input.Vt^(2/3))+(nu*Cdc*input.Sp/(input.Vt^(2/3))*alpha^3);
CD=CD0+CDalpha;
% lift coeff
alphav=0.0216*x^4-0.741*x^3+9.5929*x^2-56.267*x+137.32;
if alpha>alphav
    u=0.5*(1-cos(2*(alpha-alphav)));
Cl=(input.Kp*sin(alpha)*cos(alpha).^2)+(input.Kv*u*cos(alpha-alphav));
else
    Cl=input.Kp*sin(alpha)*cos(alpha).^2;
end
Cm=((input.Vt-input.Sb*(input.lb-x_cg))/(input.Sb*input.d))*sin(2*alpha)*cos(alpha/2)+nu*Cdc*(input.Sp/input.Sb)*((x_cg-input.xc)/input.d)*sin(alpha)^2;
end