function [states] = state_dot(x,u,t_step,input,T,N,i,M,aero_control,interp,dimensions,start_time,end_time)
%name initial conditions
pos0=[x(1) x(1,2) x(1,3)];
V0=[x(1,4) x(1,5) x(1,6)];
w0=[x(1,7) x(1,8) x(1,9)];
m0=x(1,10);
angles=x(1,11:13);
q0=angle2quat(angles(2),angles(1),angles(3),'YZX');
initial=[pos0 V0 w0 m0 q0];
%% run model using current workspace
if dimensions==3
options=odeset('MaxStep',1,'RelTol',1e-3,'AbsTol',1e-4,'Events',@stop_events); %add stop events if needed
else
    options=odeset('MaxStep',1,'RelTol',1e-3,'AbsTol',1e-4);
end
time_interval=[start_time:end_time];
% warning('off','all') %stops integration error due to time step not small enough
[time,X]=ode45(@odefunc,time_interval,initial,options,u,t_step,input,T,N,aero_control,interp,dimensions);

% split data up 
Pos_new=X(:,1:3);
Vnew=X(:,4:6) ;
w_new=X(:,7:9);
m_new=X(:,10);
q_new=X(:,11:14);
[pitch,roll,yaw]=quat2angle(q_new,'YZX');
angle_new=[roll,pitch,yaw];
states=[Pos_new Vnew w_new m_new angle_new];
end
