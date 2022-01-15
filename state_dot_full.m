function [states] = state_dot_full(x,u,t_step,input,T,N,M,aero_control,Nstates,dimensions,Ncontrols)
%name initial conditions
pos0=[x(1) x(1,2) x(1,3)];
V0=[x(1,4) x(1,5) x(1,6)];
w0=[x(1,7) x(1,8) x(1,9)];
m0=x(1,10);
initial=[pos0 V0 w0 m0 input.q0];
%% run model using current workspace
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
warning('off','all')
% warning('off','all') %stops integration error due to time step not small enough
[time,X]=ode45(@odefunc,time_interval,initial,options,u,t_step,input,T,N,aero_control,interp,dimensions);

% if numel(X(:,1))<numel(time_interval)
% options=odeset('MaxStep',1,'RelTol',1e-3,'AbsTol',1e-4)
%     [time,X]=ode23s(@odefunc,time_interval,initial,options,u,t_step,input,T,N,aero_control,interp,dimensions);
% end
% split data up
Pos_new=X(:,1:3);
Vnew=X(:,4:6) ;
w_new=X(:,7:9);
m_new=X(:,10);
q_new=X(:,11:14);
[pitch,roll,yaw]=quat2angle(q_new,'YZX');
angle_new=[roll,pitch,yaw];
states=[Pos_new Vnew w_new m_new angle_new];

[rows,columns]=size(states);
% if rows<T
%     out=states;
% else
out=zeros(N+1,Nstates);
out(1,:)=initial(1:Nstates);
if floor(rows/M)<N
    out=out;
else
for i=1:N+1
out(i,:)=states((i-1)*floor(T/N)+1,:);
end
end
% end

states=out;
end
