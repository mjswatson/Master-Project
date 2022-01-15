clear
input=constants;
posf=[-200 1000 100000];
Max_gimball_angle=6; %deg
Max_aero_angle=45; %deg
aero_forces='on'; %on or off
min_throttle=0.5;
max_rotation_speed=4; %deg
Iter=2;       %number of points tested
max_iter=10;    %max number of iterations run by fmincon
con_tol=1e-3;        %fmincon contraint tolerance
step_tol=1e-3;      %fmincon smallest step size
Vmax=2500;       %estimated final speed
N=3;           % number of nodes => (N-1) subintervals
M=60;        %number of intervals
Nstates=14;            % number of states
Ncontrols=3;            % number of controls
pos0=input.pos0;
v0=input.V0;
w0=input.w0;
m0=input.mf;
q0=input.q0;
x0=[pos0, v0, w0, m0, q0];       % initial states
xf =posf;      % final states
initial_control=[0,0,1]; %initial control value d1,d2 throttle
T=180;  %final time s
t_step = T/(N)/(M); %time step
if strcmp(aero_forces,'on')
aero_control=1;
else
    aero_control=0;
end
max_rotation=deg2rad(max_rotation_speed);
%% Store parameters for the use of constraint and objective function
auxdata.Vmax=Vmax;
auxdata.M=M;
auxdata.N=N;                  
auxdata.T=T;          
auxdata.Nstates=Nstates;    
auxdata.Ncontrols=Ncontrols;  
auxdata.x0=x0;    
auxdata.xf=xf;   
auxdata.t_step=t_step;
auxdata.max_aero_angle=Max_aero_angle;
auxdata.aero_control=aero_control;
%% Set lower bound and upper bound for states and control
[yU,yL]=bounds(max_rotation,m0,Max_gimball_angle,min_throttle,N,M);
%% Initial guess
x=zeros(N+1,Nstates);
u=zeros(N*M+1,Ncontrols);
%set throttle to full for all
u(:,3)=1;
%put initial conditions in
x(1,:)=x0;
u(1,:)=initial_control;
%calc states of intial controls
x=state_dot_full(x,u,t_step,input,T,N,M,aero_control,Nstates);
x=x(1:end-1,:);
x=x(:);
u=u(:);
% nvars=N*Ncontrols;
y0=[x;u];
%% check functions
% warning('off','all')
confun(y0, auxdata,input)
objfun(y0, auxdata,input)
warning('off','all')
strtpt=zeros(Iter,(numel(u)+numel(x)));
strtpt(1,:)=transpose(y0); %set initial value given as 1 start point 
%% gen start points
parfor i=2:Iter
    j=1;
        while j==1
    ui=rand(N*M+1,Ncontrols).*(2*Max_gimball_angle)-Max_gimball_angle;
    ui(:,3)=rand(N*M+1,1).*(1-min_throttle)+min_throttle;
%     ui(1:5,3)=1; %first 5 throttle is full
%     ui(1:5,1:2)=0; %no gimball in first 5
    xi=zeros(N,Nstates);
    xi(1,:)=x0;
    xin=state_dot_full(xi,ui,t_step,input,T,N,M,aero_control,Nstates);
   [rows,columns]=size(xin);
    xi(2:end,:)=xin(2:N,:);
%ensures initial guess runs fully - decided to not use -
%more intensive to find runable solution this way than fmincon       
%     if rows==N+1 
%     j=0;
%     else
%     j=1;
%     end
j=0;
        end
xi=xi(:);
ui=ui(:);
yi=[xi;ui];
yi=transpose(yi);
strtpt(i,:)=yi;
end
%% find trajectories to point 
for i=1:Iter
    y0=strtpt(i,:);
    y0=transpose(y0);
     options=optimoptions(@fmincon,'UseParallel',true,'Display','Iter','MaxFunctionEvaluations',Inf,'MaxIterations',max_iter,'ConstraintTolerance',con_tol,'StepTolerance',step_tol);
 [y_opt,f,exitflag,output]=fmincon(@(y)objfun(y, auxdata,input),y0,[],[],[],[],yL,yU,@(y) confun(y, auxdata,input),options);

end
% options=optimoptions(@fmincon,'UseParallel',true,'MaxFunctionEvaluations',Inf,'MaxIterations',max_iter,'ConstraintTolerance',con_tol,'StepTolerance',step_tol);
% problem=createOptimProblem('fmincon','x0',y0,'objective',@(y)objfun(y, auxdata,input),'lb',yL,'ub',yU,'nonlcon',@(y) confun(y, auxdata,input),'options',options);
% startpnts=CustomStartPointSet(strtpt);
% ms=MultiStart('Display','iter','UseParallel',true);
% [y_opt,f,exitflag,output,y_opt_all]=run(ms,problem,startpnts);

save('y_opt_all');
%quit