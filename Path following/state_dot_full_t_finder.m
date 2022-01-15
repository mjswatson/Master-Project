function [states,time] = state_dot_full(x,u,t_step,input,t,N,M,aero_control,Nstates,final_state,T,run_no,dimensions)
%name initial conditions
pos0=[x(1) x(1,2) x(1,3)];
V0=[x(1,4) x(1,5) x(1,6)];
w0=[x(1,7) x(1,8) x(1,9)];
m0=x(1,10);
initial=[pos0 V0 w0 m0 input.q0];
%% run model using current workspace
step=1;
if run_no==2
   options=odeset('MaxStep',1,'RelTol',1e-3,'AbsTol',1e-4,'Events',@stop_events); %add stop events if needed 
else
options=odeset('MaxStep',1,'RelTol',1e-3,'AbsTol',1e-4); %add stop events if needed
end
time_interval=0:step:t;
%u(:,1:2)=deg2rad(u(:,1:2));
interp=controls(u,t_step,t,dimensions);
% warning('off','all') %stops integration error due to time step not small enough
[time,X]=ode45(@odefunc,time_interval,initial,options,u,t_step,input,t,N,aero_control,interp,final_state,dimensions);
% split data up
% states=X;
% 
% [rows,columns]=size(states);
% if rows<T
%     out=states;
% else
% %remove excess rows - rows where stops- ensures on time step rows present
states=X;
end
