function y = integral2(r,R,K_theta)
r=r(:);
K_theta=K_theta(:);
int=(r./R).*K_theta;
area=trapz(int);
y = area;
end