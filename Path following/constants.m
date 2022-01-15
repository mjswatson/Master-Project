function out=constants
clear
%dimensions
%initial conditions
pitch=90;
yaw=0;
roll=0;
w0=[0 0 0];
V0=[0 0 0];
pos0=[0 0 0];
%current values for falcon 9 1.1
lb= 70; %height of rocket (m)
Ln=5.38; %see nose height (m)
d=3.7; %largest diameter (m)
db=3.7; %diameter at base of nose (m)

%mass and positions of mass
%all distances measured from nose
mt=505846; %total mass without sat
mass_sat= 2000;
mf=381412; %mass first stage fuel
xf=37.6; %position of centre mass of first stage fuel - assumed in middle
xp=8.55;     %position of centre of mass rest of rocket - assumed in middle
mass_pay=mt-(mf+mass_sat);   %mass of rest of rocket

%engines description
No_of_engines=9; %number of engines
Tvac=690E3;   %Engine Thrust in vaccuum N
Isp=311; %Engine Specific Impulse
d_nozzle=0.9; %engine nozzle diameter m
engine_h=2.18; %m
%equation to describe nose r in terms of x
c=0;
syms f(x,d_s,Ln_s) phi(x,Ln_s)
phi(x,Ln_s)=acos(1-(2*x/Ln_s));
f(x,d_s,Ln_s)=((d_s/2)/sqrt(pi()))*sqrt(phi-(0.5*sin(2*phi))+(c*sin(phi)^3)); %haack ogive model 13
sec_size=0.2; %depth of section - will affect accuracy (m)

%end inputs 
%calculating other parameters from inputs

%making quaternion
pitch=deg2rad(pitch);
yaw=deg2rad(yaw);
roll=deg2rad(roll);
quat=angle2quat(roll,pitch,yaw,'ZYX');
q0=[quat(1) quat(4) quat(3) quat(2)];
%engine stuff
An=(pi()*(d_nozzle/2)^2); %engine nozzle area

%gimbal distance
dist_gim=lb-engine_h;

%find parts of line for nose
x_val=[0.0001:sec_size:Ln];
output=hypersonic_solver(f,phi,Ln,x_val,db);
theta=output(1,:);
r=output(2,:);
clear Ln_s d_s x f phi %remove symbolic variables

%nose
output=nose_calcs(r,sec_size,x_val);
Vn=output(1); %Vol nose m^3
San=output(2); % Surface area nose m^2
Spn=output(3); %planform area nose m^2
xcn=output(4);
curve_l=output(5);
Abn=pi()*(db/2)^2; %m^2

%body 
Sab=(lb-Ln)*2*pi()*(d/2); % Surface area cylinder m^2
Sb=pi()*(d/2)^2; %area of body m2
Spb=d*(lb-Ln);
Vb=Abn*(lb-Ln);

%totals
Sa=San+Sab; %Surface area of whole body m^2
Sp=Spb+Spn; %plan form area
Vt=Vn+Vb;

%centroid
xcb=0.5*(lb-Ln);
xc=(Spb*xcb+xcn*Spn)/Sp;

%surface length
l_b=lb-Ln;
L_s=l_b+curve_l;

%constants
g0=9.81; %sea level grav
u0=6.67408E-11;  %grav constant 
Ro=6371E3; %radius of earth
p_atm=100000; % sea level pressure
a0=343; %sea level speed of sound
gamma=1.4;
mu0=1.716e-5; %viscosity at ground

%aero constants due to circular shape
Kv=3.2;
Kp=1.3;

%standard atm data
[h_stand,press_stand,temp_stand,dens_stand,a_stand]=stand_atm();

Re_trans=3e6; %transition reynolds number from laminar to turbulent
Cftbar=0.074*Re_trans^-0.2;
Cflbar=0.664/sqrt(Re_trans);
%create output structure
out=struct;
out=ws2struct;
end
