clear
%% inputs
input=constants;
initialcontrol=[0 0 1];
no_lines=5;
a=1;   %max turning acceleration in g
t_step=1; %time interval for controls
tol=0.1; % %/100 of target has to be within
max_iter=50;    %max number of iterations run by fmincon
max_step_time=10; %inital if path cant be followed at this interval will be increased - give more freedom
increase_factor=1.2; %factor of time increase
max_search=100000; %number of random control variation that will be tried before abandoning the path - increases with search factor as well
init_tol=1.5;
con_tol=1e-3;        %fmincon contraint tolerance
step_tol=1e-3;      %fmincon smallest step size
objective_limit=1e-2;
N=10;       %number of turning points
M=10;       %nodes per line
O=2;        %number sub divisions between nodes
Nstates=14;
Ncontrols=3;
Max_gimball_angle=6; %deg
Max_aero_angle=45; %deg
aero_forces='on'; %on or off
min_throttle=0.5;
max_rotation_speed=4; %deg
T=180; %time for whole flight
t=ceil(T/(N/2)); %over estimate time for each point
no_control_points=ceil(t/t_step);
%%
posf=[-200 1000 100000];
pos0=input.pos0;
v0=input.V0;
w0=input.w0;
m0=input.mf;
q0=input.q0;
x0=[pos0, v0, w0, m0, q0];       % initial states
xf =posf;      % final states
if strcmp(aero_forces,'on')
aero_control=1;
else
    aero_control=0;
end
max_rotation=deg2rad(max_rotation_speed);
Max_gimball_angle=deg2rad(Max_gimball_angle);
%% Store parameters for the use of constraint and objective function
auxdata.N=N;            
auxdata.M=M;
auxdata.Nstates=Nstates;    
auxdata.Ncontrols=Ncontrols;  
auxdata.x0=x0;    
auxdata.xf=xf;   
auxdata.t=t;
auxdata.max_aero_angle=Max_aero_angle;
auxdata.aero_control=aero_control;
auxdata.t_step=t_step;
auxdata.O=O;
auxdata.T=T;
%% generate random paths between start and end point 
[lines,quats]=line_generation(auxdata,no_lines);
%% analyse lines
for i=1:no_lines
   line=lines(:,:,i);
   lineO=line;
   quat=quats(:,:,i);
        x=zeros(M,Nstates);
        x(1,:)=x0;
        t_start=1;
        u_all=zeros(T,Ncontrols);
        flight=zeros(T,Nstates);
        j=1;
        k=2;
        corner=zeros(10,3);
    while j<numel(line(:,1))+10*N
        failure=0;
        check=10000;
        %accounts for shorter first leg        
        if norm(flight(1,:))==0 %makes intial run initialise
        max_step_time_O=max_step_time;
        %set start as previous target sol
        x_init=x(j,:);
        %set way point as target
        final_state=[line(j+1,:),quat(j+1,:)];
        else
            x_init=flight(t_end,:);
            if norm(abs(flight(t_end,1:3)))*1.2<norm(abs(line(j,:))) %if run isnt as far as first point stops from moving on
                j=j-1;
                final_state=[line(j+1,:),quat(j+1,:)];
            else
            final_state=[line(j+1,:),quat(j+1,:)];
            end
           if norm(abs(flight(t_end,1:3)))*1.2>norm(abs(lineO(k,:))) %check if passed corner point if yes plot corner and reset before
                k=k+1;
            plot_corner=true;
           else
            plot_corner=false;
            end
            
         if plot_corner
        [corner,corner_quat]=circles_for_days(quat,j,x_init,a,line);
        %insert corner points into line while removing point outside corner
        before=line(1:j-1,:);
        after=line(j+1:end,:);
        line=[before;corner;after];
        %insert quats for corner into line
        before_quat=quat(1:j-1,:);
        after_quat=quat(j+1:end,:);
        quat=[before_quat;corner_quat;after_quat];
        %remove excess data back to before turn begins
        if abs(x_init(1:3))>=round(abs(corner(1,:)),4)
        for q=1:t_end
            point_checker=abs(corner(1,:))-abs(flight(q,1:3));
            if point_checker<0
                break
            end
        end
        u_all(q-1:end,:)=0; %removes all controls and flights past turn
        flight(q-1:end,:)=0;
        x_init=flight(q-2,:); %resets intial states to before turn
        end
        final_state=[line(j,:),quat(j,:)];
         end
        end
        dist=norm(final_state(1:3)-x_init(1:3)); 
        
        %% find time needed for leg and shrink leg to small enough time frame 
        check2=0;
        while check2~=1
        [t,no_control_points]=t_finder(auxdata,x_init,input,final_state,dist);
        %check time size of segment - reduce if too large
        if  or(t<=1,dist<50) %if less than 1s
            final_state=[line(j+2,:),quat(j+2,:)];
            j=j+1;
            dist=norm(final_state(1:3)-x_init(1:3));
        elseif t>max_step_time 
           x_new=linspace(x_init(1),final_state(1),3);
            z_new=linspace(x_init(2),final_state(2),3);
            y_new=linspace(x_init(3),final_state(3),3);
            final_state(1:3)=[x_new(2) z_new(2) y_new(2)]; %as linear interp grad will be same so q same
            dist=norm(final_state(1:3)-x_init(1:3));
        else
            check2=1;
            t=ceil(t);
            no_control_points=t;
        end 
        end       
        %correct for new t
        auxdata.t=t; 
         xi=zeros(M,Nstates);
         xi(1,:)=x_init;
         iter=1;
         
         %% find control scheme and check if meets min criteria 
        while check>tol
        %first guess is always full power straight line burn
        check1=0;
        while check1~=1
        if iter==1
            ui(no_control_points+1,1:2)=0;
            ui(:,3)=0.99;
            xin=state_dot_full(xi,ui,t_step,input,t,N,M,aero_control,Nstates,final_state,T);
            error=error_calc(xin,final_state,line,j);
        else
         %gen random start points in parallel   
         for idx=1:1000
         func(idx)=parfeval(@start_gen,2,no_control_points,auxdata,Max_gimball_angle,min_throttle,xi,input,final_state,line,j);
         end
            for idx=1:1000
                [completedIdx,value,error]=fetchNext(func);
            if and(init_tol>error,error>-init_tol)
                ui=value;
                break
            end
            end
            cancel(func)
        end     
            
        if and(init_tol>error,error>-init_tol)
                check1=1;
            else
                check1=0;
        end 
            iter=iter+1;
            if iter>(max_search/1000) %stop if too many iterations run and no solution found
                break
            end
        end
        %% optimise results within met criteria 
         if iter>(max_search/1000)
             break
         end
%         [row,col]=size(xin);
%         xi(2:row-1,:)=xin(2:end-1,:);
%         
%         xi=xi(:);
%         ui=ui(:);
        y0=[ui];
        y0=y0(:);
        
        %test
        objfun(y0,auxdata,input,final_state,x_init,line,j)
        check3=0;
                
        %optimise 
        [yU,yL]=bounds(max_rotation,m0,Max_gimball_angle,min_throttle,N,M,no_control_points);
        options=optimoptions(@fmincon,'UseParallel',true,'Display','Iter','MaxFunctionEvaluations',Inf,'ObjectiveLimit',objective_limit,'StepTolerance',1e-12);
        [y_opt,f,exitflag,output]=fmincon(@(y)objfun(y, auxdata,input,final_state,x_init,line,j),y0,[],[],[],[],yL,yU,[],options);
        % test solution and find real end values
        %x = reshape(y_opt(1:Nstates*(M)), [], Nstates);
        u = reshape(y_opt, [], Ncontrols);
        [test,time]=state_dot_full_test(x_init,u,t_step,input,t,N,M,aero_control,Nstates,final_state,T);
        %% check fmincon has hit target or on same line- if not tries again
        check=error_calc(test,final_state,line,j); %check how close to target
        end
        %% change time step if conditions not met and record failed attempt
        if iter>(max_search/1000)
            if max_step_time==1
                break
            end
            max_step_time=ceil(increase_factor*max_step_time);
            failure=failure+1;
            max_search=max_search*increase_factor;
           if max_step_time>T/2
               break
           end
        end
        %% store results and time gap covered if conditions met
        if check<tol
        if time(end)~=floor(time(end))
     t_end=t_start+floor(time(end));
     u(:,1:2)=u(:,1:2)/10;
     u_all(t_start:t_end,:)=u(1:end-1,:);
     flight(t_start:t_end,:)=test(1:end-1,:);
     t_start=t_end;
        else
     t_end=t_start+floor(time(end));
     u(:,1:2)=u(:,1:2)/10;
     u_all(t_start:t_end,:)=u(1:end,:);
     flight(t_start:t_end,:)=test(1:end,:);
     t_start=t_end;
        end
     %if has previously failed and so changed step resets step
     if failure>=1
     max_step_time=max_step_time/(increase_factor^failure);
     end
        end
        % removes variable size matrices so next run can create fresh
     clear u ui
     j=j+1;
     %% if fails too many times abandon the path 
     if failure>5
         break
     end
    end
    %% save path if sucessful 
    if iter<(max_search/1000)
    u_line(:,:,i)=u_all;
    end
end