%% STIMA e CONTROLLO - LAB.4 - PENDOLO INVERSO

% Definizione Variabili utili al sistema
M=0.3;
m=0.2;
g=9.81;
L=0.5;

%% Definizione del Sistema in Anello Aperto

A=[0 1 0 0;0 0 -(m*g)/M 0;0 0 0 1;0 0 ((M+m)*g)/(M*L) 0];
B=[0;1/M;0;-1/(M*L)];
C=[1 0 0 0;0 0 1 0];
D=0;
sys=ss(A,B,C,D);

% Autovalori del sistema in anello aperto
e=eig(A);

% CONTROLLABILITA
C0=ctrb(sys);
if rank(C0)==4
    fprintf('Sistema Completamente Controllabile \n')
else
    fprintf('Sistema NON Completamente Controllabile \n')
end

% OSSERVABILITA
O0=obsv(sys);
if rank(O0)==4
    fprintf('Sistema Completamente Osservabile \n')
else
    fprintf('Sistema NON Completamente Osservabile \n')
end

%% Specifiche sulla Retroazione

% PRIMA SPECIFICA (Allocazione Poli Immaginari)
p1=[-40 -20 -2+1i -2-1i];
% Retroazione
K1=place(A,B,p1);
% Matrice di stato sistema retroazionato
sys1=ss(A-B*K1,B,C,D);

% SECONDA SPECIFICA (Controllore Ottimo R=1)
R=1;
Q=C'*C;
[K2,S2,E2]=lqr(sys,Q,R);
% Matrice di stato sistema retroazionato
sys2=ss(A-B*K2,B,C,D);

% TERZA SPECIFICA (Controllore Ottimo R=1/1000)
R=1/1000;
[K3,S3,E3]=lqr(sys,Q,R);
% Matrice di stato sistema retroazionato
sys3=ss(A-B*K3,B,C,D);

%% Rappresento le uscite del sistema

% Condizione iniziale
x0=[0;0;0.45;0];

t=0:0.005:10;

[y1,t1]=initial(sys1,x0,t);
[y2,t2]=initial(sys2,x0,t);
[y3,t3]=initial(sys3,x0,t);

figure(1)
plot(t1,y1),title('Uscita sistema in anello chiuso (Allocazione di Poli Immeginari)'),xlabel('Tempo [s]'),ylabel('Uscita'),grid

figure(2)
plot(t2,y2),title('Uscita sisteam in anello chiuso ( Controllore Ottimo R=1)'),grid,xlabel('Tempo [s]'),ylabel('Uscita');

figure(3)
plot(t3,y3),title('Uscita sistema in anello chiuso (Controllore Ottimo R=1/1000)'),grid,xlabel('Tempo [s]'),ylabel('Uscita');
