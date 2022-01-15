function [points]=circles_for_days(quat,j,x_init,a,line)
a=a*9.81;
r=norm(x_init(1,1:3))^2/a; %find turning circle from velocity -currently at start of leg - could use max v
C=2*pi()*r;
%find path vectors and angle between them
path_line1=line(j+1,:)-line(j,:);
path_line_d1=path_line1/norm(path_line1);
path_line2=line(j,:)-line(j-1,:);
path_line_d2=path_line2/norm(path_line2);
angle=pi()-acos(dot(path_line_d1,path_line_d2)/(norm(path_line_d1)*norm(path_line_d2)));

%find points of contact with sphere
s1=r/tan(angle/2);     %dist from turning point
point1=line(j,:)+s1*path_line_d1;    %forecast forward along 2nd line - find point s1 from turn
point2=line(j,:)-s1*path_line_d2;   
point3=point1+1e-6*C*path_line_d1;
points=[point2,point1];
%approx on sphere if v small dist from point1 relative to the cirumference
%find centre of circle
% options=optimoptions('fsolve','MaxFunctionEvaluations',50000,'MaxIterations',10000,'Display','none');
% [centre,fval,exit]=fsolve(@(x)sphere_equations(x,point1,point2,point3,r),point1,options);
% if exit<1 %ensures equations solved by retrying from random point between 2 points
%     while exit<1
%         start=rand(1,3).*(point1-point2)+point2;
%         [centre,fval,exit]=fsolve(@(x)sphere_equations(x,point1,point2,point3,r),start,options);
%     end
% end
% 
% norm1=point1-centre; %move coords of lines to around centre of sphere
% norm2=point2-centre;
% 
% point1_lat_long=[asin(norm1(3)/r),atan2(norm1(1)/r,norm1(2)/r)];   %convert coords to lat and long
% point2_lat_long=[asin(norm2(3)/r),atan2(norm2(1)/r,norm2(2)/r)];
% 
% A=[cos(point2_lat_long(1)),0,sin(point2_lat_long(1))];
% lambda_d=point1_lat_long(2)-point2_lat_long(2);
% B=[cos(point1_lat_long(1))*cos(lambda_d),cos(point1_lat_long(1))*sin(lambda_d),sin(point1_lat_long(1))];
% d=acos(dot(A,B)/(norm(A)*norm(B)));  %find distance between points along circle
% s=linspace(0,d,10);     %split distance into steps
% P=zeros(numel(s),3);
% for i=1:numel(s)        %find coords of steps
% P(i,:)=((sin((d-s(i)))/sin(d)).*A)+((sin(s(i))/sin(d)).*B); 
% if P(i,1)<0
% long(i,1)=atan(P(i,2)./P(i,1))+pi()+point2_lat_long(2);
% else
%     long(i,1)=atan(P(i,2)./P(i,1))+point2_lat_long(2);
% end
% end
% lat=asin(P(:,3));
% %convert long and lat to sphere co-ords
% y=sin(lat).*r;
% z=cos(lat).*cos(long).*r;
% x=cos(lat).*sin(long).*r;
% points_norm=[x,z,y];
% %convert back to inertial coords
% points=points_norm+centre;
% %add in line co-oords for finding quats
% points_all=zeros(12,3);
% points_all(1,:)=line(j,:)-s1*1.1*path_line_d2;
% points_all(2:end-1,:)=points;
% points_all(end,:)=line(j,:)+s1*1.1*path_line_d1; 
% points_all=round(points_all,5);
% 
% %find quaternion at points so can be aimed at
% y=points_all(:,3);
% x=points_all(:,1);
% z=points_all(:,2);
% diffs=zeros(10,2);
% angles=zeros(10,3);
% for k=2:11
%     dy=y(k)-y(k-1);
%     dx=x(k)-x(k-1);
%     dz=z(k)-z(k-1);
% dydx=dy/dx;
% dzdx=dz/dx;
% diffs(k,1)=dydx;
% diffs(k,2)=dzdx;
% pitch=atan2(dy,sqrt(dx^2+dz^2));
% yaw=atan2(dx,dz);
% angles(k,2)=pitch;
% angles(k,3)=yaw;
% end
% quat=angle2quat(angles(:,1),angles(:,2),angles(:,3),'ZYX');
% quat(1,:)=[];
% corner=round(points,5);
end

