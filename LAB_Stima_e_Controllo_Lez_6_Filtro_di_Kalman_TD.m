%% STIMA e CONTROLLO - LAB.6 - FILTRO DI KALMAN TD

% Definizione del Sistema + rumore di processo
A=[1.1269 -0.4940 0.1129;1 0 0;0 1 0];
B=[-0.3832;0.5919;0.5191];
C=[1 0 0];
D=0;

Plant = ss(A,[B B],C,D,-1,'inputname',{'u' 'w'},'outputname','y');

%% Filtro di Kalman

Q=1;
R=1;
[kalmf,L,p,M]=kalman(Plant,Q,R);

% Estraggo la prima uscita del filtro di kalman
% ye
kalmf=kalmf(1,:);

%% Definisco il sistema completo con errore di misura ed errore di processo
% In pratica aggiungo al sistema Plant il sensore che ne misura le uscite
% introducendo un certo rumore di misura

a=A;
b=[B B 0*B];
c=[C;C];
d=[0 0 0;0 0 1];

P=ss(a,b,c,d,-1,'inputname',{'u' 'w' 'v'},'outputname',{'y' 'yv'});

%% Effettuo il collegamento in parallelo tra il filtro ed il sistema P

% Assegno l'ingresso u ad entrambi i sistemi
sys=parallel(P,kalmf,1,1,[],[]);

% Retroaziono l'uscita yv in ingresso
SimModel=feedback(sys,1,4,2,1); % Retroaizone positiva

% Trascuro la componente yv dalla lista di I/O
SimModel=SimModel([1 3],[1 2 3]);

% Stampo a schermo la lista di input ed output del sistema
SimModel.InputName
SimModel.OutputName

%% Simulo il comportamento del sistema con i rumori

t=[0:100]';
u=sin(t/5);
n=length(t);
rng default
w=sqrt(Q)*randn(n,1);
v=sqrt(R)*randn(n,1);

% Simulazione
[out,x]=lsim(SimModel,[w,v,u]);

% Estraggo le componenti utili
y=out(:,1);
ye=out(:,2);
yv=y+v;

%% Rappresentazione

subplot(211),plot(t,y,'-',t,ye,'--'),title('Uscite del Sistema'),xlabel('No. di campioni'),ylabel('Output')
legend('Uscita Reale','Uscita Stimata'),grid;

subplot(212),plot(t,y-yv,'-',t,y-ye,'--'),title('Errori del Sistema'),xlabel('No. di campioni'),ylabel('Errore')
legend('Errore di Misura','Errore di Stima'),grid;

%% Calcolo Covarianza degli errori

% Errore di misura
ErrMeas=y-yv;
ErrMeasCov=sum(ErrMeas.*ErrMeas)/length(ErrMeas)

% Errore di stima
ErrEst=y-ye;
ErrEstCov=sum(ErrEst.*ErrEst)/length(ErrEst)