function [I,iI]=inertia(m,input)
h=input.lb-input.Ln;
d=input.d;
mass=m;
Izz=(1/12)*mass*(3*(d/2)^2+h^2);
Iyy=Izz;
Ixx=0.5*mass*(d/2)^2;
I =[Ixx 0 0;
    0 Iyy 0;
    0 0 Izz];
iI=inv(I);
end