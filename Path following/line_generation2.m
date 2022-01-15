function co_ords=line_generation(auxdata,no_lines)
no_points=auxdata.N;
start=auxdata.x0(1:3);
final=auxdata.xf(1:3);
delta_d=30;
%all=zeros(no_points+2,3,no_lines);

for i=1:no_lines
%gen random points between start and end
x=[(final(1)/no_points):(final(1)/no_points):final(1)]; %makes standard distance along
x=x(:);
z=(final(2)-start(2)).*rand(no_points,1)+start(2);
y=(final(3)-start(3)).*rand(no_points,1)+start(3);
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
% combine points and find curve
points=[x z y];
xy=[start;points;final];


for j=2:no_points+2
    dist=norm(xy(j-1,:)-xy(j,:));
    no_divisions=ceil(dist/delta_d);
    x_points=linspace(xy(j-1,1),xy(j,1),no_divisions);
    y_points=linspace(xy(j-1,3),xy(j,3),no_divisions);
    z_points=linspace(xy(j-1,2),xy(j,2),no_divisions);
    xall{j-1}=x_points(:);
    zall{j-1}=z_points(:);
    yall{j-1}=y_points(:);
end
    xall=cell2mat(xall(:));
    zall=cell2mat(zall(:));
    yall=cell2mat(yall(:));
    xall_lines{i}=xall;
    yall_lines{i}=yall;
    zall_lines{i}=zall;
    clear xall zall yall
end
co_ords=[xall_lines(:), zall_lines(:), yall_lines(:)];
end