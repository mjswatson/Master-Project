function [all,allquat]=line_generation(auxdata,no_lines)
no_points=auxdata.N;
start=auxdata.x0(1:3);
final=auxdata.xf(1:3);
%  start=[0 0 0];
%  final=[-200 1000 100000];
%  no_points=200;
% no_lines=1;
%all=zeros(no_points+2,3,no_lines);

for i=1:no_lines
%gen random points between start and end
point1=start+[0 0 100]; %ensures first part is just burn straight up for 100m 
x=(final(1)-point1(1)).*rand(no_points-1,1)+point1(1); 
z=(final(2)-point1(2)).*rand(no_points-1,1)+point1(2);
y=(final(3)-point1(3)).*rand(no_points-1,1)+point1(3);
%order points
% directions
if final(1)<0
    directionx='descend';
else
    directionx='ascend';
end
if final(2)<0
    directionz='descend';
else
    directionz='ascend';
end
if final(3)<0
    directiony='descend';
else
    directiony='ascend';
end
x=sort(x,directionx);
z=sort(z,directionz);
y=sort(y,directiony);
points=[x z y];
xy=[start;point1;points;final];



%function to divide curve into equal spaced parts - https://www.mathworks.com/matlabcentral/fileexchange/7233-curvspace
xy1=curvspace(xy,no_points+1); 
xy(3:end,:)=xy1(2:end,:);
y=xy(:,3);
x=xy(:,1);
z=xy(:,2);


%find info about points
diffs=zeros(no_points+2,2);
angles=zeros(no_points+2,3);
for j=2:no_points+1
    dy=y(j)-y(j-1);
    dx=x(j)-x(j-1);
    dz=z(j)-z(j-1);
dydx=dy/dx;
dzdx=dz/dx;
diffs(j,1)=dydx;
diffs(j,2)=dzdx;
pitch=atan2(dy,sqrt(dx^2+dz^2));
yaw=atan2(dx,dz);
angles(j,2)=pitch;
angles(j,3)=yaw;
end
% combine points and find curve
quat=angle2quat(angles(:,1),angles(:,2),angles(:,3),'ZYX');
quat(end,:)=quat(end-1,:); %ensures final point has achievable quaternion
% xy=transpose(xy);
% curve=cscvn(xy);
% 
% fnplt(curve)
% hold on
% scatter3(xy(1,:),xy(2,:),xy(3,:));
% hold on
alldiff(:,:,i)=diffs;
allquat(:,:,i)=quat;
all(:,:,i)=xy;
% all_curves(:,i)=curve;
end
end
% hold off