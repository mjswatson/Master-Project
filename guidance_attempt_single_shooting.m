clear
overall=tic;
input=constants;
posf=[-80000 1 100000];
tol=0;
no_cores=20;
plot_iter=false;
Iter=10;       %number of points tested if random
load_old=true;
if load_old
     starts=load('result2_2D_plots.mat');
     Iter=numel(starts.constraint);
end
dimensions=2; %number of dimensions to run in 
initial_control=[0,1]; %initial control values d1,d2 throttle for 3d
Ncontrols=2;            % number of controls
Max_gimball_angle=6; %deg
max_time=3*3600;
Max_aero_angle=45; %deg
aero_forces='on'; %on or off
min_throttle=0.5;
save_name='result2_2D_plots2.mat';
max_rotation_speed=4; %deg
max_iter=30;    %max number of iterations run by fmincon
con_tol=10;        %fmincon contraint tolerance
step_tol=1;      %fmincon smallest step size
Vmax=2500;       %estimated final speed
t_step=1;   %time interval for controls
Nstates=14;            % number of states
pos0=input.pos0;
v0=input.V0;
w0=input.w0;
m0=input.mf;
q0=input.q0;
x0=[pos0, v0, w0, m0, q0];       % initial states
xf =posf;      % final states
T=240;  %final time s
N=(T/t_step)+1;
if strcmp(aero_forces,'on')
aero_control=1;
else
    aero_control=0;
end
max_rotation=deg2rad(max_rotation_speed);
%% Store parameters for the use of constraint and objective function
auxdata.Vmax=Vmax;
auxdata.N=N;                  
auxdata.T=T;          
auxdata.Nstates=Nstates;    
auxdata.Ncontrols=Ncontrols;  
auxdata.x0=x0;    
auxdata.xf=xf;   
auxdata.t_step=t_step;
auxdata.max_aero_angle=Max_aero_angle;
auxdata.aero_control=aero_control;
auxdata.dimensions=dimensions;
auxdata.tol=tol;
% Set lower bound and upper bound for states and control
% piecewise fuction for control u, there are N values of control
if dimensions==3
uL1 = -Max_gimball_angle*ones(N, 1);                 
uU1 = Max_gimball_angle*ones(N, 1); 
uL2=-Max_gimball_angle*ones(N, 1);  
uU2= Max_gimball_angle*ones(N, 1);
uL3=ones(N, 1)*min_throttle;
uL3(1)=1; % ensure full throttle at start to lift off 
uU3=ones(N, 1);
yL = [uL1; uL2; uL3];
yU = [uU1; uU2; uU3];
else
uL1 = -Max_gimball_angle*ones(N, 1);                 
uU1 = Max_gimball_angle*ones(N, 1); 
uL2=ones(N, 1)*min_throttle;
uL2(1)=1; % ensure full throttle at start to lift off 
uU2=ones(N, 1);
yL = [uL1; uL2];
yU = [uU1; uU2];  
end
%% Initial guess
u=zeros(N,Ncontrols);
%set throttle to full for all
u(:,Ncontrols)=1;
%put initial conditions in
u(1,:)=initial_control;
u0=u(:);
y0=u0;
strtpt(1,:)=transpose(y0);
%% check functions
warning('off','all')
confun(y0, auxdata,input)
objfun(y0, auxdata,input)
%  time_limit=time_limit_hrs*60*60;
%% gen random starts
pool=gcp('nocreate');
if isempty(pool)
parpool(no_cores)
end
if not(load_old)
parfor i=2:Iter
    ui=rand(N,Ncontrols).*(2*Max_gimball_angle)-Max_gimball_angle;
    ui(:,Ncontrols)=rand(N,1).*(1-min_throttle)+min_throttle;
    ui=ui(:);
yi=[ui];
yi=transpose(yi);
strtpt(i,:)=yi;
end
else
    j=1;
    clear strtpt
    for i=1:Iter
    if starts.output_store{i}.stepsize>1e-9
        strtpt(j,:)=transpose(starts.y_opt_store(:,i));
        j=j+1;
    else
        Iter=Iter-1;
    end
    end
end
time=toc(overall);
time_left=max_time-time;
time_per_it=time_left/Iter;
%% Solve NLP problem:
% set up options for solver
    for i=1:Iter
if plot_iter
 options=optimoptions(@fmincon,'UseParallel',true,'Display','Iter','MaxFunctionEvaluations',Inf,'OutputFcn',@(y,optimValues,state)myplotfun(y,optimValues,state,auxdata,input,time_per_it),'FiniteDifferenceStepSize',1e-3);
else
  options=optimoptions(@fmincon,'UseParallel',true,'Display','Iter','MaxFunctionEvaluations',Inf,'OutputFcn',@(y,optimValues,state)stopper(y,optimValues,state,time_per_it),'FiniteDifferenceStepSize',1e-3);
end
pool=gcp('nocreate');
if pool.NumWorkers<no_cores
    delete(gcp('nocreate'));
    parpool(no_cores)
end
    start=strtpt(i,:); 
    y0=start;
 [y_opt,f,exitflag,output]=fmincon(@(y)objfun(y, auxdata,input),y0,[],[],[],[],yL,yU,@(y) confun(y, auxdata,input),options);
y_opt_store(:,i)=y_opt;
u = reshape(y_opt,[N Ncontrols]);
u_store(:,:,i)=u;
f_store(i)=f;
constraint(i)=output.constrviolation;
output_store{i}=output;

if plot_iter
hold on 
grid on
grid minor
hold off
name=strcat('single_shooting_method',num2str(i),'_',num2str(dimensions),'D','.fig');
savefig(name)
clf
end
    end
 %% Save controls
u_opt=reshape(y_opt,[N Ncontrols]);
[Pos_new,Vnew,w_new,m_new] = state_dot(x0,u_opt,t_step,input,T,N,aero_control,dimensions);
dist=Pos_new(end,:)-xf(1:3);
states=[Pos_new,Vnew,w_new,m_new];
save(save_name,'u','xf','f_store','constraint','y_opt','u_store','output_store','y_opt_store')
quit; %for running on mencomp