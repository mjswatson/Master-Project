function [states,time_check] = state_dot(x,u,t_step,input,T,M,i,j,O,aero_control,interp,final_state)
%name initial conditions
pos0=[x(1) x(1,2) x(1,3)];
V0=[x(1,4) x(1,5) x(1,6)];
w0=[x(1,7) x(1,8) x(1,9)];
m0=x(1,10);
q0=x(1,11:14);
initial=[pos0 V0 w0 m0 q0];
%% run model using current workspace
options=odeset('MaxStep',1,'RelTol',1e-3,'AbsTol',1e-4,'Events',@stop_events); %can add stop events func if needed
start_time=(i-1)*(T/(M))+(j-1)*(T/(M)/(O)); %so correct t
end_time=(i-1)*(T/(M))+j*(T/(M)/(O));
step=(end_time-start_time)/10;
time_interval=[start_time:step:end_time];
% warning('off','all') %stops integration error due to time step not small enough
[time,X]=ode45(@odefunc,time_interval,initial,options,u,t_step,input,T,M,aero_control,interp,final_state);
% split data up 
if time(end)~=end_time
    time_check=1;
else
    time_check=0;
end
states=X;

end
