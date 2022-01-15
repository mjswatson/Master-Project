function out=hypersonic_solver(f,phi,Ln_val,x_val,d)
syms drdx x Ln_s r d_s %create variables
dr_dx=diff(f,x); %differntiate equation of line

i=1;
drdx_all=zeros(1,numel(x_val)); %preallocate
r_all=zeros(1,numel(x_val));

while i<numel(x_val)
    eqn=[dr_dx-drdx==0, Ln_s==Ln_val, x==x_val(i), f-r==0, d_s==d]; %create system of equations
    vars=[drdx,Ln_s,x,r,d_s]; %variables vector
    sol=solve(eqn,vars); %solve equations 
    %extract answers
    drdx_val=double(sol.drdx);
    r_val=double(sol.r);
    drdx_all(:,i)=drdx_val;
    r_all(:,i)=r_val;
    i=i+1;
end
%final conversions and output
    r_all(end)=d/2;   %set final value to end diameter
    theta=atan(drdx_all);
    theta=rad2deg(theta);
    out=[theta;r_all];
end   