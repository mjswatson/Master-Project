function F=great_circle(x,norm1,norm2,r)
F(1)=norm1(1)*sin(x(2))+norm1(3)*cos(x(2))-norm1(2)/sqrt((r/x(1))^2-1);
F(2)=norm2(1)*sin(x(2))+norm2(3)*cos(x(2))-norm2(2)/sqrt((r/x(1))^2-1);
end