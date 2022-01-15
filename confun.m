 function [c,ceq] = confun(y, auxdata,input,time_gaps)
% % Get some parameters
N           = auxdata.N;
M           = auxdata.M;
T           = auxdata.T;
x_init      = auxdata.x0;
x_final     = auxdata.xf;
Nstates     = auxdata.Nstates;
Ncontrols   = auxdata.Ncontrols;
dimensions=auxdata.dimensions;
t_step=auxdata.t_step;
x0=auxdata.x0;
max_aero_angle=auxdata.max_aero_angle;
aero_control=auxdata.aero_control;
norm1=auxdata.norm;
% extract states and controls
x=zeros(N,Nstates);
x(2:end,:) = reshape(y(1:Nstates*(N-1)), [], Nstates);
x=x.*norm1;
x(1,:)=x0;
u = reshape(y(Nstates*(N-1)+1: end), [], Ncontrols);
u(:,1:dimensions-1)=deg2rad(u(:,1:dimensions-1));
% create interpolants for controls - speeds up
interp=controls(u,t_step,T,dimensions);
% Do single shooting in each subinterval
states_at_Nodes = zeros(N+1, Nstates);
states_at_Nodes(1,:)=x_init;
start_time=0;
for i = 1:N
    states = zeros(M, Nstates);
    states(1,:) = x(i,:);
    end_time=start_time+time_gaps(i);
    states=state_dot(x(i,:),u,t_step,input,T,N,i,M,aero_control,interp,dimensions,start_time,end_time);
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
%     
%     if norm(states(end,:))==0 %stops process if last loop broke due to rotation
%         break
%     end
    
    states_at_Nodes(i+1,:)= states(end,:);
   states_all(start_time+1:end_time+1,:)=states;
    start_time=end_time;
end

if isnan(states_at_Nodes(end,:))
    posf=[1 1 1];
else
    Pos_new=states_at_Nodes(:,1:3);
posf=Pos_new(end,:);
 end
pos_target=x_final(end,1:3);
angles_diff=x(1:end,11:13) - states_at_Nodes(1:end-1,11:13);
% continuity constraints
ceq_temp = x - states_at_Nodes(1:end-1,:);
%angles bit
diff=zeros(N,3);
for i=1:numel(angles_diff)
    if angles_diff(i)>pi()
        diff(i)=pi()-(angles_diff(i)-pi());
    else
        diff(i)=angles_diff(i);
    end
end
ceq_temp(:,11:13)=diff;
% initial and final state postions
final_pos=zeros(1,Nstates); %sets all other states position diff to 0 - wont be considered
final_pos(1:3)=[posf - pos_target]; 
if dimensions==2
    final_pos(2)=0;
end
%check quaternion is correct
ceq_temp = [ceq_temp; final_pos];
% equality constraints
ceq = ceq_temp;
% inequality constraints
% c = states_atNodes(:, 1)' - 1/9; % this may not important (for this problem)as we imposed
% lower bound for states at nodes
c = [];
 end
    
    