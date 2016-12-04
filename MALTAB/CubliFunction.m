l=0.085;
lb=0.075;
mb=0.419;
mw=0.204;
Ib=3.34*10^-3;
Iw=0.57*10^-3;
Cb=1.02*10^-3;
Cw=0.05*10^-3;
Km=25.1*10^-3;
g=9.81;
T=0.02;
A=[0,1,0;(mb*lb+mw*l)*g/(Ib+mw*(l^2)),-Cb/(Ib+mw*(l^2)),Cw/(Ib+mw*(l^2));-(mb*lb+mw*l)*g/(Ib+mw*(l^2)),Cb/(Ib+mw*(l^2)),-Cw*(Ib+Iw+mw*(l^2))/(Ib+mw*(l^2))/Iw];
B=[0;-Km/(Ib+mw*(l^2));Km*(Ib+Iw+mw*(l^2))/(Ib+mw*(l^2))/Iw];
C=eye(3);
D=zeros(3,1);
Ad=expm(A*T)
syms x;
Bd=inv(A)*(Ad-eye(3))*B
Q=[1,0,0;0,1,0;0,0,1];
R=1;
Qd=[1,0,0;0,1,0;0,0,1];
Rd=1;
K=lqr(A,B,Q,R)
K1=K(1,1);
K2=K(1,2);
K3=K(1,3);
Kd=lqrd(A,B,Q,R,T)
Kd1=Kd(1,1);
Kd2=Kd(1,2);
Kd3=Kd(1,3);
Kdd=dlqr(Ad,Bd,Q,R)