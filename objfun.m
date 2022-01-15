function [L] = objfun( y, auxdata,input)
N       = auxdata.N;
Nstates = auxdata.Nstates;
Ncontrols=auxdata.Ncontrols;
t_step=auxdata.t_step;
Vmax=auxdata.Vmax;
xf=auxdata.xf;
T=auxdata.T;
M=auxdata.M;
dimensions=auxdata.dimensions;
aero_control=auxdata.aero_control;
norm1=auxdata.norm;
x0=auxdata.x0;
% extract conditions and controls from y
x=zeros(N,Nstates);
x(2:end,:) = reshape(y(1:Nstates*(N-1)), [], Nstates);
x=x.*norm1;
x(1,:)=x0;
u = reshape(y(Nstates*(N-1)+1: end), [], Ncontrols);
% %time step

% %run sim
if dimensions==3
options=odeset('MaxStep',1,'RelTol',1e-3,'AbsTol',1e-4,'Events',@stop_events); %add stop events if needed
else
    options=odeset('MaxStep',1,'RelTol',1e-3,'AbsTol',1e-4);
end

time_interval=0:T;
if dimensions==2
  u(:,1)=deg2rad(u(:,1));  
else
u(:,1:2)=deg2rad(u(:,1:2));
end
interp=controls(u,t_step,T,dimensions);
in=[x0(1:10),input.q0];
warning('off','all')
% warning('off','all') %stops integration error due to time step not small enough
[time,X]=ode45(@odefunc,time_interval,in,options,u,t_step,input,T,N,aero_control,interp,dimensions);

% %find final states
Vnew=X(end,4:6);
if or(isnan(Vnew),isinf(Vnew))
    Vnew(1)=-10;
end
% target=xf(end,1:3);
% pos=x_real(end,1:3);
% normalised=pos./target;
%objective function
L =-Vnew(1);
% this form could be changed to exact obj function of problem
end