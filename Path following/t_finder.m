function [t,no_control_points]=t_finder(auxdata,x_init,input,final_state,dist,plot_corner)
%extract data from auxdata
Ncontrols=auxdata.Ncontrols;
M=auxdata.M;
Nstates=auxdata.Nstates;
t_step=auxdata.t_step;
N=auxdata.N;
dimensions=auxdata.dimensions;
aero_control=auxdata.aero_control;
T=auxdata.T;
xi=zeros(M,Nstates);
t=T/3;
run_no=1;
x_init(7:9)=0;
xi(1,:)=x_init;
test=1;

no_control_points=t/t_step;
u=zeros(no_control_points+1,Ncontrols);
u(:,1:Ncontrols-1)=0;
if plot_corner
u(:,Ncontrols)=0.7; %slows for corner
else
    u(:,Ncontrols)=1;
end
[xin,time]=state_dot_full_t_finder(xi,u,t_step,input,t,N,M,aero_control,Nstates,final_state,T,run_no,dimensions);
[rows,col]=size(xin);
%find initial gap
for i=1:rows
dist_travelled=norm(xin(i,1:3)-x_init(1:3));
check=dist_travelled-dist;
if check>0
    break
end
end
t=((i-1));
no_control_points=t/t_step;
u=zeros(no_control_points+1,Ncontrols);
u(:,1:Ncontrols-1)=0;
if plot_corner
u(:,Ncontrols)=0.7; %slows for corner
else
    u(:,Ncontrols)=1;
end
run_no=run_no+1;
%find exact stopping time
[xin,time]=state_dot_full_t_finder(xi,u,t_step,input,t,N,M,aero_control,Nstates,final_state,T,run_no,dimensions);
if time(end)>i-2
t=time(end);
no_control_points=t/t_step;
end
end
