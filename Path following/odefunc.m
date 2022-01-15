function dxdt=odefunc(t,x,u0,t_step,input,T,N,aero_control,interp,final_state,dimensions)
S=[x(1) x(2) x(3)];
V=[x(4) x(5) x(6)];
w=[x(7) x(8) x(9)];
mf=x(10);
q=[x(11) x(12) x(13) x(14)];
h=S(3);
% if h<0
%     dxdt=zeros(1,14);
%     return
% end
if mf<0
    mf=0;
end
%% controls
if dimensions==2
Throttle=interp.throttle(t);
d1=interp.d2(t);
d2=0;
else
    Throttle=interp.throttle(t);
d1=interp.d1(t);
d2=interp.d2(t);
end
%% other calcs
%mass calcs
m=mf+input.mass_pay;
x_cg=((mf*input.xf)+(input.xp*input.mass_pay))/m;
%other systems
pressure=interp1(input.h_stand,input.press_stand,h,'linear','extrap');
[Fp,Mp]=engine(input,d1,d2,Throttle, pressure, x_cg,m);
if mf==0
    Fp=0;
    Mp=0;
end
[I,iI]=inertia(m,input);
g=input.g0*(input.Ro/(h+input.Ro))^2;
if isnan(q)
    TBI=[0 0 0;
        0 0 0;
        0 0 0];
    iTBI=TBI;
else
[iTBI,TBI]=TBI_calc(q);
end
if aero_control==1
[Fa,Ma]=aero(V,h,x_cg,input);
else
    Fa=[0 0 0];
    Ma=[0 0 0];
end
%% Velocity inputs
if h>0
grav=TBI*[0;0;g];
grav=reshape(grav,[1,3]);
else
    grav=[0 0 0];
end
Forces_acel=(Fp+Fa)/m;

%% create w cross
wz=w(3);
wy=w(2);
wx=w(1);
wcross=[0 -wx -wy -wz;
    wx 0 wz -wy;
    wy -wz 0 wx;
    -wz -wy -wx 0];

wcrss=[0 -wz wy;
    wz 0 -wx;
    -wy wx 0];

%% differentials
dvdt=grav+Forces_acel-(V*wcrss);

dwdt=((Mp+Ma)-cross(w,(w*I)))*iI;
mag_dwdt=sqrt(dwdt(1)^2+dwdt(2)^2+dwdt(3)^2);
% if mag_dwdt<1e-5
%     dwdt=[0 0 0];
% end
[roll,pitch,yaw]=quat2angle(q,'ZYX');
pitch=rad2deg(pitch);
yaw=rad2deg(yaw);
dqdt=0.5*wcross*q(:);
dqdt=reshape(dqdt,[1,4]);
dmdt=(input.No_of_engines*(-input.Tvac/(input.Isp*input.g0)))*Throttle;
dsdt=V*iTBI;
if mf==0
    dmdt=0;
end
% 
% display=[t];
% disp(display)
if dimensions==2
    dsdt(2)=0;
    dwdt(2)=0;
    dvdt(2)=0;
end
dxdt=[dsdt dvdt dwdt dmdt dqdt];
dxdt=dxdt(:);
end