%% STIMA e CONTROLLO - LAB.2 - SISTEMA IDRAULICO

% Definizione Variabili Utili al sistema
r=0.3;
R=6;
c=pi*(r^2);
S=pi*(R^2);
g=9.81;
H=4;
% Portate in ingresso ai serbatoi
q10=1;
q30=0.5;
% Condizioni Iniziali serbatoi ( Battenti iniziali)
h1=11;
h2=9;
h3=6;

a=((c^2)*g)/(S*q10);
b=((c^2)*g)/(S*(q10+q30));

%% Definizione del Sistema in anello aperto

A=[-a a 0;a -2*a a;0 a -(a+b)];
B=[1/S 0;0 0;0 1/S];
C=[0 0 1];
D=[0 0];

sys=ss(A,B,C,D);

% Autovalori del sistema in anello aperto
e=eig(A);

% CONTROLLABILITA
C0=ctrb(sys);
if rank(C0)==3
    fprintf('Sistema Completamente Controllabile \n')
else
    fprintf('Sistema NON Completamente Controllabile \n')
end

% OSSERVABILITA 
O0=obsv(sys);
if rank(O0)==3
    fprintf('Sistema Completamente Osservabile \n')
else
    fprintf('Sistema NON Completamente Osservabile \n')
end

%% Specifiche sulla Retroazione

% PRIMA SPECIFICA Poli Reali
p1=[-0.01 -0.05 -0.05];
K1=place(A,B,p1);
% Matrice di stato sistema retroazionato 1
A1=A-B*K1;

% SECONDA SPECIFICA Poli Immaginari
p2=[-0.005*(1+1i) -0.005*(1-1i) -0.05];
K2=place(A,B,p2);
% Matrice di stato sistema retroazionato 2
A2=A-B*K2;

%% Definizione dell'Osservatore

% Specifica sull'osservatore
p=[-0.005 -0.1 -0.2];
% Retroazione sul sistema duale
KD=place(A',C',p);
L=KD';
% Matrice di stato dell'osservatore
A0=A-L*C;

%% Definizione Sistema Retroazionato + Osservatore
% Sfrutto il principio di separazione

% PRIMA SPECIFICA (Poli Reali)
a=[A1 B*K1;zeros(3) A0];
b=[B;0*B];
c=[C 0*C];
d=D;
sys1=ss(a,b,c,d);

% SECONDA SPECIFICA (Poli Immaginari)
a=[A2 B*K2;zeros(3) A0];
b=[B;0*B];
c=[C 0*C];
d=D;
sys2=ss(a,b,c,d);

%% Risposta Libera del sistema nei due casi

% Condizione iniziale
x0=[h1;h2;h3;h1+1;h2-1;h3+2];

[y1,t1,x1]=initial(sys1,x0);
[y2,t2,x2]=initial(sys2,x0);

sim('LAB_Stima_e_Controllo_Lez_3_Sistema_Idraulico_Simulink');

figure(1)
plot(t1,y1),grid,title('Variazione Rispetto all equilibrio del livello dei serbatoi (Poli Reali)')
xlabel('Tempo [s]'),ylabel('Variazione');

figure(2)
plot(t1,x1(:,4),'-',t1,x1(:,5),'--',t1,x1(:,6),'-.'),grid,title('Errore sulla stima degli stati')
xlabel('Tempo [s]'),ylabel('Errore')
legend('Errore sulla prima VdS','Errore sulla seconda VdS','Errore sulla terza VdS');

figure(3)
plot(t2,y2),grid,title('Variazione Rispetto all equilibrio del livello dei serbatoi (Poli Immaginari)')
xlabel('Tempo [s]'),ylabel('Variazione');

figure(4)
plot(t2,x2(:,4),'-',t2,x2(:,5),'--',t2,x2(:,6),'-.'),grid,title('Errore sulla stima degli stati')
xlabel('Tempo [s]'),ylabel('Errore')
legend('Errore sulla prima VdS','Errore sulla seconda VdS','Errore sulla terza VdS');

figure(5)
plot(t,err),title('Errore sulla sima degli stati (Simulink)'),grid;
