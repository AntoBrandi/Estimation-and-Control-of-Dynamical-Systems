%% STIMA e CONTROLLO - LAB.1 - MOTORE CC

% Definizione delle variabili utili al sistema
Ra=0.03;
La=0.0006;
f=1;
J=100;
km=3.8;
kv=km;
% Valore Iniziale del Guadagno 
Ga=1;

%% Definizione del Sistema in Anello Aperto

A=[0 1 0;0 -f/J km/J;0 -kv/La -Ra/La];
B=[0;0;Ga/La];
C=[1 0 0];
D=0;

sys=ss(A,B,C,D);

% Polinomio caratteristico sistema in anello aperto
polinomio=poly(A);

% Autovalori sistema in anello aperto
e=eig(A);

% Zeri, Poli, Guadagno sistema in anello aperto
[Zeri,Poli,Guadagno]=zpkdata(sys,'v');

% Pulsazione Naturale e coefficiente di smorzamento sistema in anello
% aperto
[Wn,Smorzamento]=damp(sys);

% Funzione di trasferimento sistema in anello aperto
[NUM,DEN]=tfdata(sys,'v');

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

%% Rappresentazione risposta al Gradino e all'Impulso in anello aperto

% Risposta all'impulso
figure(1)
impulse(sys),grid,title('Risposta ad un Impulso, Sistema in Anello Aperto'),xlabel('Tempo [s]'),ylabel('Uscita');

% Risposta al gradino
figure(2)
step(sys),grid,title('Risposta ad un Gradino, Sistema in Anello Aperto'),xlabel('Tempo [s]'),ylabel('Uscita');

%% Specifiche sulla Retroazione

%% PRIMA SPECIFICA (Poli Immaginari)

% Radici
p1=[-60 3*sqrt(2)*(-1+1i) 3*sqrt(2)*(-1-1i)];
% Polinomio
P1=poly(p1);
% Errore di posizione nullo ep=0
Ga=P1(4)/NUM(4);
B=[0;0;Ga/La];
% Retroazione sul sistema con ep=0
K1=place(A,B,p1);
% Sistema Retroazionato con ep = 0 e prima specifica
sys1=ss(A-B*K1,B,C,D);

% Funzione di trasferimento in anello chiuso 
[NUM1,DEN1]=tfdata(sys1,'v');

% Zeri,Poli, Guadagno sistema in anello chiuso
[Zeri1,Poli1,Guadagno1]=zpkdata(sys1,'v');

% Pulsazione naturale e coefficiente di smorzamento sistema in anello
% chiuso
[Wn1,Smorzamento1]=damp(sys1);


%% SECONDA SPECIFICA (Poli Reali)

% Radici
p2=[-80 -30 -3];
% Polinomio
P2=poly(p2);
% Errore di posizione nullo ep=0
Ga=P2(4)/NUM(4);
B=[0;0;Ga/La];
% Retroazione sul sistema con ep=0
K2=place(A,B,p2);
% Sistema Retroazionato con ep = 0 e prima specifica
sys2=ss(A-B*K2,B,C,D);

% Funzione di trasferimento in anello chiuso 
[NUM2,DEN2]=tfdata(sys2,'v');

% Zeri,Poli, Guadagno sistema in anello chiuso
[Zeri2,Poli2,Guadagno2]=zpkdata(sys2,'v');

% Pulsazione naturale e coefficiente di smorzamento sistema in anello
% chiuso
[Wn2,Smorzamento2]=damp(sys2);

%% Risposta al gradino sistemi in anello chiuso

figure(3)
step(sys1),hold on
step(sys2),title('Risposta ad un Gradino, Sistema in Anello Chiuso'),grid,xlabel('Tempo [s]'),ylabel('Uscita')
legend('Prima Specifica (Poli Immaginari)','Seconda Specifica (Poli Reali)');

