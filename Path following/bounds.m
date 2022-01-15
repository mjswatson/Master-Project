function [yU,yL]=bounds(max_rotation,m0,Max_gimball_angle,min_throttle,N,M,no_control_points)
Max_gimball_angle=Max_gimball_angle*10;
uL1 = -Max_gimball_angle*ones(no_control_points+1, 1);                 
uU1 = Max_gimball_angle*ones(no_control_points+1, 1); %times M means 1 control every sub interval
uL2=-Max_gimball_angle*ones(no_control_points+1, 1);  
uU2= Max_gimball_angle*ones(no_control_points+1, 1);
uL3=ones(no_control_points+1,1)*min_throttle;
uL3(1)=1; % ensure full throttle at start to lift off 
uU3=ones(no_control_points+1,1);
yL = [uL1; uL2; uL3];
yU = [uU1; uU2; uU3];
end