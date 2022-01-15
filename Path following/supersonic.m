function [CD,Cl,Cm]=supersonic(M,alpha,x_cg,Cf,input)
Cdc=Cdc_interp(M*sin(alpha));
Cl=2*alpha+(Cdc*(input.Sp/input.Sb)*alpha^2);
Cm=2*alpha*((input.Vb/(input.Sb*input.lb))-(1-(x_cg/input.lb)))+(Cdc*(input.Sp/input.Sb)*((input.xc-x_cg)/input.lb)*alpha^2);
%drag
CDb=Cdb_interp(M);
CDN2=4.65/(((2*input.Ln)/input.d)^2);
Cd0=(Cf*(input.Sa/input.Sb))+CDb+CDN2;
CDalpha=Cl*alpha;
CD=Cd0+CDalpha;

end