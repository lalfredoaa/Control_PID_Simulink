%=========================================================
%DC motors
%=========================================================
Tp=1.2;
Wd=pi/Tp;

Mp=2.8-2.5;

%find d using maximun overshoot Md=exp(-d*pi/(sqrt(1-d^2))
Mp=(2.8-2.5)/2.5;

k=log(Mp);
d1=-k/sqrt(k^2+pi^2);
d2=k/sqrt(k^2+pi^2);
if d1 > 0
    d = d1;
else 
    d = d2;
end

%find Wn using Wd=Wn*sqrt(1-d^2)
Wn=Wd/sqrt(1-d^2);

Ks=2.5;

%Substitute in transfer function
G=tf([Ks*Wn^2],[1 2*d*Wn Wn^2]);

%with integrator
Gi=tf([Ks*Wn^2],[1 2*d*Wn Wn^2 0]);


%=========================================================
%Packaging station
%=========================================================

%controllable canonical form
A=[0 1 0;
   0 0 1;
   0 0 0];
B=[0 0 0.2]';
C=[1 0 0];
sys=ss(A,B,C,0);

%controlability
Contr=[B A*B A*A*B];
rank(Contr); %3 in this case so it is controllable 
%observability
Obs=obsv(sys.A, sys.C);
rank(Obs); %3 so in this case it is controllable

%We propose K
K=acker(sys.A,sys.B,[-1,-2,-3]);
%and L
L=acker(sys.A',sys.C',[-4 -8 -12])'; %faster poles

%to follow reference
A2=sys.A-sys.B*K;
sys2=ss(A2,sys.B,sys.C,0);
G2=tf(sys2); %from here we get the needed gain to compensate the 1 in ss
%our compensating gain is 6/0.2


%for vertical movement motor:
Pol=poly([-1 -2 -3]);


%=========================================================
%DC motors control
%=========================================================
%horizontal position
x=3;
if(x>0)
    xdisp=20*x-10;
elseif (x<0)
    xdisp=20*x+10;
else
    xdisp=0;
end

%delay to arrive
if(abs(x)==5)
    delayx=60; %100 for placement movement
elseif(abs(x)==4)
    delayx=50;
elseif(abs(x)==3)
    delayx=40;
elseif(abs(x)==2)
    delayx=30;
elseif(abs(x)==1)
    delayx=20;
else
    delayx=0;
end
    
%vertical position
y=-2;
if(y>0)
    ydisp=25*y-12.5;
elseif (y<0)
    ydisp=25*y+12.5;
else
    ydisp=0;
end

if(abs(y)==5)
    delayy=40; %100 for placement movement
elseif(abs(y)==4)
    delayy=30;
elseif(abs(y)==3)
    delayy=25;
elseif(abs(y)==2)
    delayy=20;
elseif(abs(y)==1)
    delayy=10;
else
    delayy=0;
end

if(delayx>delayy)
    delaypos=delayx;
else
    delaypos=delayy;
end

%delay 100 for the 25 placement movement

delaystation1=1.5;
delaystation2=50;
delaystation3=125; %can be 150 like professor's graph
delaystation4=300; %5x60s period



