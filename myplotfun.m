function stop=myplotfun(y,optimValues,state,auxdata,input,time_gaps,time_per_it)
iter_no=optimValues.iteration;
remain=rem(iter_no,5);
if and(strcmp(state,'iter'),remain==0)
N       = auxdata.N;
Nstates = auxdata.Nstates;
Ncontrols=auxdata.Ncontrols;
t_step=auxdata.t_step;
Vmax=auxdata.Vmax;
xf=auxdata.xf;
T=auxdata.T;
x0=auxdata.x0;
M=auxdata.M;
dimensions=auxdata.dimensions;
aero_control=auxdata.aero_control;
norm1=auxdata.norm;
% extract states and controls
x=zeros(N,Nstates);
x(2:end,:) = reshape(y(1:Nstates*(N-1)), [], Nstates);
x=x.*norm1;
x(1,:)=x0;
u = reshape(y(Nstates*(N-1)+1: end), [], Ncontrols);
u(:,1:dimensions-1)=deg2rad(u(:,1:dimensions-1));
% %time step
start_time=0;
interp=controls(u,t_step,T,dimensions);
% %run sim
for i = 1:N
    states = zeros(M, Nstates);
    states(1,:) = x(i,:);
    end_time=start_time+time_gaps(i);
    states=state_dot(x(i,:),u,t_step,input,T,N,i,M,aero_control,interp,dimensions,start_time,end_time);  
    states_all(i)={states};
    start_time=end_time;
end
if iter_no==0
    colour_use='k';
else
line_col=iter_no/5;
colour=['r','g','b','m','y','k','c',];
colour_use=colour(line_col);
end
hold on
if dimensions==2
    for i=1:N
        state=cell2mat(states_all(i));
        if i==1
    plot(state(:,1),state(:,3),colour_use)
        else
     plot(state(:,1),state(:,3),colour_use,'HandleVisibility','off')
        end
    end
    scatter(x(:,1),x(:,3),colour_use,'HandleVisibility','off')
else
        for i=1:N
        state=cell2mat(states_all(i));
        if i==1
        plot3(state(:,1),state(:,2),state(:,3),colour_use)
        else
        plot3(state(:,1),state(:,2),state(:,3),colour_use,'HandleVisibility','off')
        end
        end
    scatter3(x(:,1),x(:,2),x(:,3),colour_use,'HandleVisibility','off')
end
hold off
end

if strcmp(state,'init')
    tic
    stop=0;
else
    time=toc;
    if time>=time_per_it
        stop=1;
    else
        stop=0;
    end
end
end