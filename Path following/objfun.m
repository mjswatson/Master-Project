function [L] = objfun( y, auxdata,input,final_state,x_init,line,j)
N       = auxdata.N;
Nstates = auxdata.Nstates;
Ncontrols=auxdata.Ncontrols;
t_step=auxdata.t_step;
T=auxdata.T;
M=auxdata.M;
t=auxdata.t;
dimensions=auxdata.dimensions;
aero_control=auxdata.aero_control;
max_rot=auxdata.max_rotation;
% extract conditions and controls from y
u = reshape(y, [], Ncontrols);
x=x_init;
% %run sim
x_real=state_dot_full(x,u,t_step,input,t,N,M,aero_control,Nstates,final_state,T,dimensions,Ncontrols);
error=error_calc(x_real,final_state,line,j,max_rot);           
%objective function
L =error;
% this form could be changed to exact obj function of problem
end