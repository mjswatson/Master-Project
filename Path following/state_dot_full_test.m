function [states,time] = state_dot_full_test(x,u,t_step,input,t,N,M,aero_control,Nstates,final_state,T,dimensions,Ncontrols)
%name initial conditions
pos0=[x(1) x(1,2) x(1,3)];
V0=[x(1,4) x(1,5) x(1,6)];
w0=[x(1,7) x(1,8) x(1,9)];
m0=x(1,10);
q0=x(1,11:14);
initial=[pos0 V0 w0 m0 q0];
%% run model using current workspace
step=1; %get result every second
options=odeset('MaxStep',1,'RelTol',1e-3,'AbsTol',1e-4,'Events',@stop_events); %add stop events if needed
time_interval=0:step:t;
u(:,1:Ncontrols-1)=(u(:,1:Ncontrols-1))/10;
interp=controls(u,t_step,t,dimensions);
% warning('off','all') %stops integration error due to time step not small enough
[time,X]=ode45(@odefunc,time_interval,initial,options,u,t_step,input,t,N,aero_control,interp,final_state,dimensions);
ftime=time(end); %find final time
% split data up
% states=X;
% 
% [rows,columns]=size(states);
% if rows<T
%     out=states;
% else
% %remove excess rows - rows where stops- ensures on time step rows present
% if time(end)~=t
%     X(end,:)=[];
% else
%     out=X;
% end
if numel(time_interval)==2
    states=[X(1,:);X(end,:)];
else
states=X;
end

end
