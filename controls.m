function interp=controls(u,t_step,T,dimensions)
time=[0:t_step:T];
if dimensions==2
throttle_interp=griddedInterpolant(time,u(:,2));
d2_interp=griddedInterpolant(time,u(:,1));
interp.throttle=throttle_interp;
interp.d2=d2_interp;
else
    d2_interp=griddedInterpolant(time,u(:,2));
    throttle_interp=griddedInterpolant(time,u(:,3));
    d1_interp=griddedInterpolant(time,u(:,1));
    interp.throttle=throttle_interp;
    interp.d1=d1_interp;
    interp.d2=d2_interp;
end
 
 
 
