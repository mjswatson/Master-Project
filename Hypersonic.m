function [CD,Cl,Cm]=Hypersonic(alpha,M,gamma,x_cg,Cf,input,alpha_dash)
%Cm
k_theta=zeros(1,numel(input.theta));
i=1;
while i<numel(input.theta)+1
    k_theta(i)=input.K_theta([rad2deg(alpha),input.theta(i)]);
    i=i+1;
end

lx=(x_cg-input.x_val)./input.lb;
R=input.d*0.5;
int=integral(lx,input.Ln,input.r,R,k_theta);
K=((gamma+3)/(gamma+1))*(1+((2/(gamma+3))*(1/M^2)));
Cm=int*(K/pi())*(input.Ln/R);
%CL and CD
int2=integral2(input.r,R,k_theta);
CN=(K/pi())*(input.Ln/R)*int2;
% drag stuff
Cdb=input.Cdb(M);
% x=((input.d/input.Ln)*M);
% Cdw_line=0.435*x^2+0.1158*x+0.021;
% Cdw=Cdw_line/(0.7*(M^2));
Cf_ratio=0.0001*M^5-0.0033*M^4+0.029*M^3-0.1041*M^2+0.0043*M+0.9977;
CDf=Cf*(input.Sa/input.Sb)*1.02*Cf_ratio;

% if alpha<=pi()/2
% CX_dash=-(CDf+Cdb+Cdw);
% else
% CX_dash=CDf+Cdb+Cdw;
% end
% CX=cos(alpha_dash).^2*CX_dash;

% if M<7
%     CD=CN*sin(alpha)-CX*cos(alpha);
% else
CDalpha=CN*alpha;
CDN2=4.65/(((input.Ln*2)/input.d)^2);
CD0=CDf+Cdb+CDN2;
CD=CD0+CDalpha;
CX=(CN*sin(alpha_dash)-CD)/cos(alpha_dash);
% end

Cl=CN*cos(alpha_dash)+CX*sin(alpha_dash);


end




