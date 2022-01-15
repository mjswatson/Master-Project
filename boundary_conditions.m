function F=boundary_conditions(x)
%Initial Position on globe
pos_xi=7.33; %east in degrees 
pos_zi=57.33; %north
%Final goal position
pos_xend=58.33;
pos_yend=100000;
pos_zend=8.33;
orbit_incline=1.5; %in radians
path_angle=0.5;     %in radians
rfstar=[pos_xend;pos_yend;pos_zend];
initial_pos=[pos_xi;0;pos_zi];
%create matrices
VbT=zeros(1,3);
rbT=zeros(1,3);
pv=zeros(1,3);
pr=zeros(1,3);
%insert values
x(1)=VbT(1);
x(2)=VbT(2);
x(3)=VbT(3);
x(4)=rbT(1);
x(5)=rbT(2);
x(6)=rbT(3);
x(7)=pv(1);
x(8)=pv(2);
x(9)=pv(3);
x(10)=pr(1);
x(11)=pr(2);
x(12)=pr(3);
%creating needed variables
Vb=VbT(:);
rb=rbT(:);
pv=pv(:);
pr=pr(:);
r=norm(rb);
V=norm(Vb);
Hf=rb.*Vb;
HfT=reshape(Hf,[1,3]);
N=[90 0 135];  %vector parrallel to polar axis of earth
N1=N(:);
%equations
F(1,:)=1/2*rbT*rb-1/2*rfstar.^2;
F(2,:)=N*(rb.*Vb)-abs(rb.*Vb)*cos(orbit_incline);
F(3,:)=rbT*Vb-r*V*sin(path_angle);
F(4,:)=(VbT*pr)*r^2-(rbT*pv)*V^2+(rbT*Vb)*(V^2-rbT*pr);
F(5,:)=VbT*pv-V^2;
F(6,:)=(HfT*pr)*(HfT*(rb.*N1)+(HfT*pv)*(HfT*(Vb.*N1)));
end
