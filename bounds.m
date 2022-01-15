function [yU,yL]=bounds(max_rotation,m0,Max_gimball_angle,min_throttle,N,dimensions,T,t_step)
x1L = -ones(N-1, 1);       % allow -ve position and speed           
x1U = ones(N-1, 1);         %+1 accounts for initial position value
x3L=zeros(N-1, 1); 
x3U=2*ones(N-1, 1);
x4L=zeros(N-1, 1); 
x4U=ones(N-1, 1);
x6L=-ones(N-1, 1);  
x6U=ones(N-1, 1);
x7L = zeros(N-1, 1);                  
x7U = zeros(N-1, 1);
x8L = -ones(N-1, 1);                  
x8U = ones(N-1, 1);
x10L = zeros(N-1, 1);          %has zeros so cant go -ve mass       
x10U = ones(N-1, 1);
x12L=ones(N-1,1)*-pi();
x12U=ones(N-1,1)*pi();
x11L=zeros(N-1,1);
x11U=zeros(N-1,1);
if dimensions==2
x13L=zeros(N-1,1);
x13U=zeros(N-1,1);
x5L=zeros(N-1, 1); 
x5U=zeros(N-1, 1);
x2L =zeros(N-1, 1);                 
x2U =zeros(N-1, 1);
x9L = -zeros(N-1, 1);                 
x9U = zeros(N-1, 1);
else
x13L=ones(N-1,1)*-pi();
x13U=ones(N-1,1)*pi();
x5L=zeros(N-1, 1);  
x5U=ones(N-1, 1);
x2L = -ones(N-1, 1);                 
x2U = ones(N-1, 1) ;
x9L = -ones(N-1, 1);                 
x9U = ones(N-1, 1);
end
% piecewise fuction for control u, there are N-1 values of control
ncontrols=(T/t_step)+1;
if dimensions==3
uL1 = -Max_gimball_angle*ones(ncontrols+1, 1);                 
uU1 = Max_gimball_angle*ones(ncontrols+1, 1); %times M means 1 control every sub interval
uL2=-Max_gimball_angle*ones(ncontrols+1, 1);  
uU2= Max_gimball_angle*ones(ncontrols+1, 1);
uL3=ones(ncontrols+1, 1)*min_throttle;
uL3(1)=1; % ensure full throttle at start to lift off 
uU3=ones(ncontrols+1, 1);
yL = [x1L; x2L; x3L; x4L; x5L; x6L; x7L; x8L; x9L; x10L; x11L; x12L; x13L; uL1; uL2; uL3];
yU = [x1U; x2U; x3U; x4U; x5U; x6U; x7U; x8U; x9U; x10U; x11U; x12U; x13U; uU1; uU2; uU3];
else
uL1 = -Max_gimball_angle*ones(ncontrols, 1);                 
uU1 = Max_gimball_angle*ones(ncontrols, 1); %times M means 1 control every sub interval
uL2=ones(ncontrols, 1)*min_throttle;
uL2(1)=1; % ensure full throttle at start to lift off 
uU2=ones(ncontrols, 1);
x13U(:)=0;
yL = [x1L; x2L; x3L; x4L; x5L; x6L; x7L; x8L; x9L; x10L; x11L; x12L; x13L; uL1; uL2];
yU = [x1U; x2U; x3U; x4U; x5U; x6U; x7U; x8U; x9U; x10U; x11U; x12U; x13U; uU1; uU2];  
end