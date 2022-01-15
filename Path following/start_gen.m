function [ui,error]=start_gen(no_control_points,auxdata,Max_gimball_angle,min_throttle,xi,input,final_state,line,j)
Ncontrols=auxdata.Ncontrols;
t_step=auxdata.t_step;
t=auxdata.t;
N=auxdata.N;
M=auxdata.M;
dimensions=auxdata.dimensions;
aero_control=auxdata.aero_control;
Nstates=auxdata.Nstates;
T=auxdata.T;
max_rot=auxdata.max_rotation;

ui=(rand(no_control_points+1,Ncontrols).*(2*Max_gimball_angle)-Max_gimball_angle)*10;
ui(:,Ncontrols)=rand(no_control_points+1,1).*(1-min_throttle)+min_throttle;
xin=state_dot_full(xi,ui,t_step,input,t,N,M,aero_control,Nstates,final_state,T,dimensions,Ncontrols);

error=error_calc(xin,final_state,line,j,max_rot); %distance from target as proportion of overall distance
end