function F=sphere_equations(x,point1,point2,point3,r)
F(1)=(point1(1)-x(1))^2+(point1(3)-x(3))^2+(point1(2)-x(2))^2-r^2;
F(2)=(point2(1)-x(1))^2+(point2(3)-x(3))^2+(point2(2)-x(2))^2-r^2;
F(3)=(point3(1)-x(1))^2+(point3(3)-x(3))^2+(point3(2)-x(2))^2-r^2;
end