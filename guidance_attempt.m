fun=@boundary_conditions;
Vb0=[1;2;3];
rb0=[0;0;0];
pv0=[0;0;0];
pr0=[0;0;0];
x0=[Vb0 rb0 pv0 pr0];
x=fsolve(fun,x0);
