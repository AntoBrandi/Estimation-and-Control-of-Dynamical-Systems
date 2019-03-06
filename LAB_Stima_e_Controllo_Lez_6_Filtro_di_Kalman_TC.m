%% STIMA e CONTROLLO - LAB.6 - FILTRO DI KALMAN TC

% Definizione variabili utili al sistema
Ra=0.03;
La=0.0006;
f=1;
J=100;
km=3.8;
kv=km;
Ga=1;

%% Definizione del sistema in anello aperto + rumore di processo

A=[0 1 0;0 -f/J km/J;0 -kv/La -Ra/La];
B=[0;0;Ga/La];
C=[1 0 0];
D=0;

Plant=ss(A,[B B],C,D,'inputname',{'u' 'w'},'outputname','y');

%% Filtro di Kalman

Q=1;
R=1;
[kalmf,L,p,M]=kalman(Plant,Q,R);

% Estraggo la prima componente dell'uscita del filtro
% ye
kalmf=kalmf(1,:);

%% Definizione del sistema con errore di processo ed errore di misura
% Definisco in pratica anche il sensore che misura l'uscita reale del
% sistema e la fornisce in ingresso al filtro di kalman. Tale sensore avrà
% inevitabilmente un suo errore di misura che dipenderà dalle proprietà
% stesse del sensore

a=A;
b=[B B 0*B];
c=[C;C];
d=[0 0 0;0 0 1];

P=ss(a,b,c,d,'inputname',{'u' 'w' 'v'},'outputname',{'y' 'yv'});

%% Realizzo il collegamento tra il sistema P ed il filtro di kalman

% Fornisco l'ingresso u ad entrambi i sistemi
sys=parallel(P,kalmf,1,1,[],[]);

% Retroaziono l'uscita yv in ingresso
SimModel=feedback(sys,1,4,2,1); % Retroaizone positiva

% Trascuro la componente yv dalla lista di I/O del sistema
SimModel=SimModel([1 3],[1 2 3]);

% Stampo a schermo la lista di I/O del sistema
SimModel.InputName
SimModel.OutputName

%% Simulo il comportamento del sistema

t=[0:0.01:100]';
u=sin(t/5);
n=length(t);
rng default
w=sqrt(Q)*randn(n,1);
v=sqrt(R)*randn(n,1);

% Matrici utili al simulink
W=[t w];
V=[t v];
U=[t u];

% Simulazione
[out,x]=lsim(SimModel,[w,v,u],t,'zoh');

% Estraggo le componenti utili
y=out(:,1);
ye=out(:,2);
yv=y+v;

%% Rappresento

subplot(211),plot(t,y,'-',t,ye,'--'),title('Uscite del Sistema'),xlabel('Tempo [s]'),ylabel('Output')
legend('Uscita Reale','Uscita Stimata'),grid;

subplot(212),plot(t,y-yv,'-',t,y-ye,'--'),title('Errori del Sistema'),xlabel('Tempo [s]'),ylabel('Errore')
legend('Errore di Misura','Errore di Stima'),grid;

%% Calcolo Covarianza errori

% Errore di Misura
ErrMeas=y-yv;
ErrMeasCov=sum(ErrMeas.*ErrMeas)/length(ErrMeas)

% Errore di Stima
ErrEst=y-ye;
ErrEstCov=sum(ErrEst.*ErrEst)/length(ErrEst)

