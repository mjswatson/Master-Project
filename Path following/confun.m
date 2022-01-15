 function [c,ceq] = confun(y, auxdata,input,final_state,x_init)
% % Get some parameters
M           = auxdata.M;
T           = auxdata.T;
Nstates     = auxdata.Nstates;
Ncontrols   = auxdata.Ncontrols;
t_step      = auxdata.t_step;
Max_aero_angle=auxdata.max_aero_angle;
aero_control=auxdata.aero_control;
O           = auxdata.O;
t=auxdata.t;
% extract states and controls
x = reshape(y(1:Nstates*(M)), [], Nstates);
u = reshape(y(Nstates*(M)+1: end), [], Ncontrols);
u(:,1:2)=deg2rad(u(:,1:2));
% create interpolants for controls - speeds up
interp=controls(u,t_step,t);
% Do single shooting in each subinterval
states_at_Nodes = zeros(M, Nstates);
states_at_Nodes(1,:)=x_init;
time_check=0;
for i = 1:M-1
    x0 = x(i, :);
    states = zeros(O, Nstates);
    states(1,:) = x0; 
    for j=1:O
        [out,time_check]=state_dot(states(j,:),u,t_step,input,t,M,i,j,O,aero_control,interp,final_state);
    states(j+1,:)=out(end,:);
    if time_check==1
        break
    end
%     V=out(end,4:6);
%     speed=abs(sqrt(V(1)^2+V(2)^2+V(3)^2));
%     if V(1)==0
%     alpha=0;
% else
%     alpha=atan2(V(3),V(1));
%     alpha=rad2deg(alpha);
% end
% if speed==0
%     beta=0;
% else
%     beta=asin(V(2)/speed);
%     beta=rad2deg(beta);
% end
% 
%     logic1=norm(out(end,7:9))>2*pi();
%     logic2=or(beta>max_aero_angle,alpha>max_aero_angle); %stop if angle of attack too large
%     if or(logic1,logic2) %stop if rotation speed too large
%         break
%     end
%     
     end
%     
%     if norm(states(end,:))==0 %stops process if last loop broke due to rotation
%         break
%     end
    if time_check==1
        break
    end 
    states_at_Nodes(i+1,:)= states(end,:);
end
% continuity constraints
ceq_temp = x - states_at_Nodes(1:end,:);
% initial and final state postions
% equality constraints
ceq = ceq_temp;
% inequality constraints
% c = states_atNodes(:, 1)' - 1/9; % this may not important (for this problem)as we imposed
% lower bound for states at nodes
c = [];
 end
    
    