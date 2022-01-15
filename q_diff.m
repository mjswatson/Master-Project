function dqdt=q_diff(t,q,wm)
%creates q matrix
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q=[q1;q2;q3;q4];
%differential
dqdt=0.5*wm*q;
%vectorise
dqdt=dqdt(:);
end