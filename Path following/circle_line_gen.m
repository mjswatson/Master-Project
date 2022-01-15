start=[0 0 0];
final=[-200 1000 100000];
no_points=20;
dist=5e3;
xy=zeros(no_points,3);
xy(1,:)=start;
for i=2:no_points+1
    centre=xy(i-1,:);
    %y finder
    theta=(pi()).*rand(1); %only allowed 180 then cant go backwards
    x=dist*cos(theta);
    y=dist*sin(theta);
    %find z
    z=sqrt(dist^2-(x-centre(1))^2-(y-centre(3))^2)+centre(2);
    xy(i,:)=[x,z,y];
end