function [position,isterminal,direction]=stop_events(t,x,u,t_step,input,T,N,aero_control,interp,final_state,dimensions)

q=x(11:14);
[iTBI,TBI]=TBI_calc(q);
V=x(4:6);
V=reshape(V,[1,3]);
dsdt=V*iTBI;
if final_state(1)<0
x_check=abs(final_state(1))+x(1);    
else
x_check=final_state(1)-x(1);
end
if final_state(2)<0
 z_check=abs(final_state(2))+x(2);    
else
z_check=final_state(2)-x(2);
end
y_check=final_state(3)-x(3);

% (2*pi()-mag_w)
position=[x(3);(dsdt(3)+10);x_check;z_check;y_check]; %second part means it stop if it starts falling/ losing altitude at 10m/s
isterminal=[1;1;1;1;1];
direction=[-1;-1;-1;-1;-1];
end