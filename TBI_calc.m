function [iTBI,TBI]=TBI_calc(q)
q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);
%form TBI
TBI1=q0^2+q1^2-q2^2-q3^2;
TBI2=2*(q1*q2+q0*q3);
TBI3=2*(q1*q3-q0*q2);
TBI4=2*(q1*q2-q0*q3);
TBI5=q0^2-q1^2+q2^2-q3^2;
TBI6=2*(q2*q3+q0*q1);
TBI7=2*(q1*q3+q0*q2);
TBI8=2*(q2*q3-q0*q1);
TBI9=q0^2-q1^2-q2^2+q3^2;

TBI=[TBI1 TBI2 TBI3;
    TBI4 TBI5 TBI6;
    TBI7 TBI8 TBI9];
iTBI=transpose(TBI);
end