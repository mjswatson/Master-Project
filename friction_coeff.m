function Cf=friction_coeff(visc,speed,dens,input);

if speed==0
    Cf=0;
else
Re=(dens*speed*input.L_s)/visc;
Cfl=0.664/sqrt(Re);
if Re<=3e6    
    Cf=Cfl;
else
    A=input.Re_trans*(input.Cftbar-input.Cflbar);
 Cft=(0.4555/(log10(Re)^2.58))-(A/Re); 
    Cf=Cft;
end
end
end