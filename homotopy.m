clear
overall=tic;
input=constants;
random_start_gen='no';  %yes or no to generate random start points
Iter=10;
num_cores=4;
run_fmincon=true; %run fmincon or patternsearch
path='no'; %yes or no to load followed path
if strcmp(random_start_gen,'no')
    if strcmp(path,'yes')
start_u=load('path2.mat');  %file directory if not using random start points
    else
     starts=load('2D_vac_complete.mat');
 %     Iter=numel(starts.constraint);
    end
end
save_name='random6_3D_improved4.mat'; %file results will be saved in
max_time=30; %max time in hours
posf=[-80000 1 100000];
plot_iter=false;
dimensions=2; %number of dimensions to run in 
initial_control=[0,1]; %initial control values d1,d2 throttle for 3d
Ncontrols=2;            % number of controls
Max_gimball_angle=6; %deg
Max_aero_angle=45; %deg
aero_forces='off'; %on or off
min_throttle=0.5;
max_rotation_speed=6; %deg
max_iter=10;    %max number of iterations run by fmincon
con_tol=10;        %fmincon contraint tolerance
step_tol=1;      %fmincon smallest step size
Vmax=2500;       %estimated final speed
N=4;           % number of nodes => (N-1) subintervals
t_step=1;   %time interval for controls
Nstates=13;            % number of states
pos0=input.pos0;
v0=input.V0;
w0=input.w0;
m0=input.mf;
angles=input.angles; %[roll,pitch,yaw];
x0=[pos0, v0, w0, m0, angles];       % initial states
xf =posf;      % final states
T=180;  %final time s
M=T/N/t_step;
pool=gcp('nocreate');
if isempty(pool)
parpool(num_cores)
end
if strcmp(aero_forces,'on')
aero_control=100;
else
    aero_control=0;
end
max_rotation=deg2rad(max_rotation_speed);
normaliser=[posf,2500,500,500,max_rotation,max_rotation,max_rotation,m0,pi(),1,pi()];
max_time=max_time*3600;
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
auxdata.dimensions=dimensions;
auxdata.norm=normaliser;
%find max final time 
if and(strcmp(random_start_gen,'no'),strcmp(path,'yes'))
start_controls=start_u.u_line;
start_controls(:,1:(dimensions-1))=rad2deg(start_controls(:,(1:dimensions-1)));
T_max=T;
for i=1:numel(start_controls(1,1,:))
    u=start_controls(:,:,1);
    [row,col]=size(u);
    T=(row/t_step)-1;
    if T>T_max
        T_max=T;
    end
end
T=T_max;
auxdata.T=T;
end

%split into segments
remain=rem(T,N);
if remain~=0
extra=(remain/2);
added_most=floor(extra); %how many parts need 2s adding 
time_gaps=[floor(T/N)+3];
for i=1:added_most %add legs with extra time
    time_gaps(1+i)=floor(T/N)+2;
end
for i=(3+added_most):(N)
    time_gaps(i-1)=floor(T/N);
end
else
    time_gaps=(T/N)*ones(1,N);
end
%% Set lower bound and upper bound for states and control

[yU,yL]=bounds(max_rotation,m0,Max_gimball_angle,min_throttle,N,dimensions,T,t_step);
% uL=[uL1; uL2; uL3];
% uU=[uU1; uU2; uU3];
%% Initial guess
if strcmp(random_start_gen,'yes')
x=zeros(N+1,Nstates);
u=zeros(N*M+1,Ncontrols);
%set throttle to full for all
u(:,end)=1;
%put initial conditions in
x(1,:)=x0;
u(1,:)=initial_control;
%calc states of intial controls
xi=state_dot_full(x,u,t_step,input,T,N,M,aero_control,Nstates,dimensions,Ncontrols);
x=xi(2:end-1,:);
x=x./normaliser;
x=x(:);
u=u(:);
% nvars=N*Ncontrols;
y0=[x;u];
elseif and(strcmp(path,'yes'),strcmp(random_start_gen,'yes'))
    u=start_controls(:,:,1);
    [row,col]=size(u);
    T=(row/t_step)-1;
    auxdata.T=T;
    [roll,pitch,yaw]=quat2angle(start_u.flight_path(:,11:14,1),'ZYX');
    flight_angle=[roll,pitch,yaw];
    flight_use=[start_u.flight_path(:,1:10,1),flight_angle];
    xin=flight_use(:,:,1);
%if doesnt divide exactly add extra time onto first segments - cover less distance here
remain=rem(T,N);
if remain~=0
extra=(remain/2);
added_most=floor(extra); %how many parts need 2s adding 
time_gaps=[floor(T/N)+3];
x(1,:)=xin(floor(T/N)+4,:); %add first longer leg
for i=1:added_most %add legs with extra time
    x(2+i,:)=xin((floor(T/N)+4)+(floor(T/N)+2)*i,:);
    time_gaps(1+i)=floor(T/N)+2;
end
for i=(3+added_most):(N)
    x(i,:)=xin(3+added_most+((i-1)*floor((T)/N)+1),:);
    time_gaps(i-1)=floor(T/N);
end
else
    for i=1:N
x(i,:)=xin((i)*floor(T/N)+1,:);
    end
end
x=x(1:end-1,:);
x=x./normaliser;
x=x(:);
u=u(:);
% nvars=N*Ncontrols;
y0=[x;u];
else
    y0=transpose(starts.y_opt_store(:,1));
    x= reshape(y0(1:Nstates*(N-1)), [], Nstates);
   u = reshape(y0(Nstates*(N-1)+1: end), [], Ncontrols);
end
%% check functions
% warning('off','all')
confun(y0, auxdata,input,time_gaps)
objfun(y0, auxdata,input)

strtpt=zeros(Iter,(numel(u)+numel(x)));
strtpt(1,:)=transpose(y0); %set initial value given as 1 start point 
%% gen start points
if strcmp(random_start_gen,'yes')
parfor i=1:Iter
    j=1;
        while j==1
    ui=rand(N*M+1,Ncontrols).*(2*Max_gimball_angle)-Max_gimball_angle;
    ui(:,Ncontrols)=rand(N*M+1,1).*(1-min_throttle)+min_throttle;
%     ui(1:5,3)=1; %first 5 throttle is full
%     ui(1:5,1:2)=0; %no gimball in first 5
    xi=zeros(N-1,Nstates);
    xi(1,:)=x0;
    xin=state_dot_full(xi,ui,t_step,input,T,N,M,aero_control,Nstates,dimensions,Ncontrols);
   [rows,columns]=size(xin);
    xi=xin(2:N,:);
%ensures initial guess runs fully - decided to not use -
%more intensive to find runable solution this way than fmincon       
%     if rows==N+1 
%     j=0;
%     else
%     j=1;
%     end
j=0;
        end
xi=xi./normaliser;
xi=xi(:);
ui=ui(:);
yi=[xi;ui];
yi=transpose(yi);
strtpt(i,:)=yi;
end
strtpt(1,:)=y0;
elseif and(strcmp(path,'yes'),strcmp(random_start_gen,'yes'))
    for i=2:numel(start_controls(1,1,:))
start_controls=start_u.u_line;
   start_controls(:,1:(dimensions-1))=rad2deg(start_controls(:,(1:dimensions-1)));
    u=start_controls(:,:,1);
    [row,col]=size(u);
    T=(row/t_step)-1;
    auxdata.T=T;
    xin=start_u.flight_path(:,:,i);
%if doesnt divide exactly add extra time onto first segments - cover less distance here
remain=rem(T,N);
extra=(remain/2);
added_most=floor(extra); %how many parts need 2s adding 
x(1,:)=xin(1,:);
x(2,:)=xin(floor(T/N)+3,:); %add first longer leg
for i=1:added_most %add legs with extra time
    x(2+i,:)=xin((floor(T/N)+3)+(floor(T/N)+2)*i,:);
end
for i=(3+added_most):(N+1)
    x(i,:)=xin((i-1)*floor((T)/N)+1,:);
end
x=x(1:end-1,:);
x=x./normaliser;
x=x(:);
u=u(:);
% nvars=N*Ncontrols;
yi=[x;u];
    yi=transpose(yi);
    strtpt(i,:)=yi;
    end
else
    clear strtpt
    i=1;
    j=1;
    for i=1:Iter
        if isfield(starts.output_store{i},'stepsize')==1
        if starts.output_store{i}.stepsize>1e-9
            strtpt(j,:)=transpose(starts.y_opt_store(:,i));
            j=j+1;
        else
            Iter=Iter-1;
        end
        elseif isfield(starts.output_store{i},'meshsize')==1
         if starts.output_store{i}.meshsize>1e-5
            strtpt(j,:)=transpose(starts.y_opt_store(:,i));
            j=j+1;
        else
            Iter=Iter-1;
         end            
        end
    end
end
%% Solve NLP problem -fmincon
%find time left for solver
time_gone=toc(overall);
time_left=max_time-time_gone;
time_per_it=time_left/Iter;
% set up options for solver

i=1;
    while aero_control<100
if plot_iter
 options=optimoptions(@fmincon,'UseParallel',true,'Display','Iter','MaxFunctionEvaluations',Inf,'MaxIterations',300,'OutputFcn',@(y,optimValues,state)myplotfun(y,optimValues,state,auxdata,input,time_gaps,time_per_it),'FiniteDifferenceStepSize',1e-3);
else
  options=optimoptions(@fmincon,'UseParallel',true,'Display','Iter','MaxFunctionEvaluations',Inf,'MaxIterations',300,'OutputFcn',@(y,optimValues,state)stopper(y,optimValues,state,time_per_it),'FiniteDifferenceStepSize',1e-3);
end
pool=gcp('nocreate');
if pool.NumWorkers<num_cores
    delete(gcp('nocreate'))
    parpool(num_cores)
end   
try
 [y_opt,f,exitflag,output]=fmincon(@(y)objfun(y, auxdata,input),y0,[],[],[],[],yL,yU,@(y) confun(y, auxdata,input,time_gaps),options);
y_opt_store(:,i)=y_opt;
 x=zeros(N,Nstates);
x(2:end,:) = reshape(y_opt(1:Nstates*(N-1)), [], Nstates);
x=x.*normaliser;
x(1,:)=x0;
x_opt_store(:,:,i)=x;
u = reshape(y_opt(Nstates*(N-1)+1: end), [], Ncontrols);
u_store(:,:,i)=u;
f_store(i)=f;
constraint(i)=output.constrviolation;
output_store{i}=output;
if plot_iter
hold on 
grid on
grid minor
hold off
name=strcat('multiple_shooting_method_',num2str(i),'_',num2str(dimensions),'D','.fig');
savefig(name)
clf
end
catch  
if plot_iter
    clf
end
end
time=toc(overall);
if time>40*3600
    break
end

if output.constrviolation<1
    aero_control=aero_control+5;
    y0=y_opt;
else
    aero_control=aero_control-1;
end
auxdata.aero_control=aero_control;
    end



best=min(f_store); 
[rowb,colb]=find(f_store==best);
y_opt=y_opt_store(:,rowb);
%% Extract states and controls
x=zeros(N,Nstates);
x(2:end,:) = reshape(y_opt(1:Nstates*(N-1)), [], Nstates);
x=x.*normaliser;
x(1,:)=x0;
u = reshape(y_opt(Nstates*(N-1)+1: end), [], Ncontrols);
test=state_dot_full_test(x,u,t_step,input,T,N,M,aero_control,Nstates,dimensions,Ncontrols);

save(save_name,'u','x','test','xf','f_store','constraint','y_opt','x_opt_store','u_store','output_store','y_opt_store')
quit; %for running on mencomp