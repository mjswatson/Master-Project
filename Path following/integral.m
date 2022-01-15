function y = integral(lx,lb,r,R,K_theta)
lx=lx(:);
r=r(:);
K_theta=K_theta(:);
int=(r./R).*K_theta.*(lx./lb);
area=trapz(int);
y = area;
end