function [position,isterminal,direction]=stop_events(t,x,u,t_step,input,T,N,aero_control,interp,dimensions)

w=x(7:9);
mag_w=norm(w);

% q=x(10:14);
% [TBI]=TBI_calc(q);
% V=x(4:6);
% V=reshape(V,[1,3]);
% iTBI=transpose(TBI);
% dsdt=V*iTBI;
V=abs(x(4:6));
V=reshape(V,[1,3]); %limits on sideways speed of rocket

% dsdt=V*iTBI;

position=[x(3)+100;mag_w-(pi()/2);500-V(2);500-V(3)]; %second part means it stop if it starts falling/ losing altitude at 50m/s
if or(not(isreal(position)),isnan(position))
position=[-1;1;-1;-1];
end
isterminal=[1;1;1;1];
direction=[-1;0;-1;-1];
end