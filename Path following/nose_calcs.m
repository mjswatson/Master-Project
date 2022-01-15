function out=nose_calcs(r,sec_size,x_val)
i=1;
Vn=zeros(1,numel(r));
Sa=zeros(1,numel(r));
Sp=zeros(1,numel(r));
xbar=zeros(1,numel(r));
L=zeros(1,numel(r));
while i<numel(r)-1
Vn(i)=((1/3)*pi()*sec_size)*(r(i+1)^2+(r(i+1)*r(i))+r(i)^2);
Sa(i)=pi()*(r(i)+r(i+1))*sqrt((r(i+1)-r(i))^2+sec_size^2);
Sp(i)=(r(i)+r(i+1))/2*sec_size;
xbar(i)=Sp(i)*(x_val(i)+0.5*sec_size);
L(i)=sqrt(sec_size^2+(r(i+1)-r(i))^2);
i=i+1;
end
xbar=sum(xbar);
Vn=sum(Vn);
Sa=sum(Sa);
Sp=sum(Sp);
L=sum(L);
out=[Vn;Sa;Sp;xbar;L];
end
