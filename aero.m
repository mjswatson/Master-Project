function [Fa,Ma]=aero(V,h,x_cg,input)
speed=abs(sqrt(V(1)^2+V(2)^2+V(3)^2));
if h>100000
    Fa=[0 0 0];
    Ma=[0 0 0];
else
    
if speed==0
    Fa=[0 0 0];
    Ma=[0 0 0];
else
    
a=input.a(h);
M=speed/a;
%find angle of attack

if V(1)==0
    alpha=0;
else
    alpha=atan2(V(3),V(1));
end
if speed==0
    beta=0;
else
    beta=asin(V(2)/speed);
end

%find dash angles
if sin(alpha)==0
    omega_dash=0;
else
omega_dash=atan2(tan(beta),sin(alpha));
end

alpha_dash=acos(cos(alpha)*cos(beta));

%find air conditions
Temp=input.temp(h)-273.15;
dens=input.dens(h);
if dens<0
    dens=0;
end
visc=(((Temp+273.15)/273.11)^1.5)*input.mu0*((273.11+110.56)/(Temp+273.15+110.56));
R=286;
gamma=a^2/(Temp+273.15)*R;
% gamma=1.4;
%find coeffs
Cf=friction_coeff(visc,speed,dens,input);
if M<=0.9
    [CD,Cl,Cm]=subsonic(alpha,M,Cf,input,x_cg);
elseif M<=4
    [CD,Cl,Cm]=supersonic(M,alpha,x_cg,Cf,input);
else
    [CD,Cl,Cm]=Hypersonic(alpha,M,gamma,x_cg,Cf,input,alpha_dash);
end
%convert CD and Cl to Ca CY and CN
Ca_dash=-Cl*sin(alpha_dash)+CD*cos(alpha_dash);
CN_dash=Cl*cos(alpha_dash)+CD*sin(alpha_dash);
Ca=Ca_dash;
CN=CN_dash*cos(omega_dash);
CY=CN_dash*sin(omega_dash);

%logic circuit 
    if CN==0
        Cm=0;
        x_cp=0;
    else
        x_cp=-(((Cm*input.lb)/CN)-x_cg);
    end
if x_cp<-input.lb
    x_cp=-input.lb;
end

moment_arm=x_cp-x_cg;

% create forces from coeff 
coeff=[Ca CY CN];
Fa=coeff.*(0.5*dens*(speed^2)).*input.Sb;
Ma=[0 Fa(3)*moment_arm -Fa(2)*moment_arm];
Fa=[-Fa(1) Fa(2) -Fa(3)];


end

end

end
