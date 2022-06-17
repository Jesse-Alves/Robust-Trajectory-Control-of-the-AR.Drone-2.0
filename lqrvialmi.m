%===============================================================
%========================= LQR VIA LMI =========================
%===============================================================

function K = lqrvialmi(SYS,Q,R,x0,alfa,beta,theta)
%% Tratando argumentos passados
A = SYS.A;
B = SYS.B;
C = SYS.C;
D = SYS.D;

n = size(A,1);
m = size(B,2);


%% LMI
setlmis([])

%% Declaração de Variáveis
gama = lmivar(1,[1 1]);
[W1,n_W1] = lmivar(1,[n 1]);
[W2,n_W2] = lmivar(2,[m n]);

%% ===========> Declaração de LMIs
%% 1 LMI
lmiterm([-1 1 1 gama],1,1);
lmiterm([-1 2 1 0],x0);
lmiterm([-1 2 2 W1],1,1);
%% 2 LMI
lmiterm([2 1 1 W1],A,1,'s');
lmiterm([2 1 1 W2],B,1,'s');
lmiterm([2 2 1 W1],Q^0.5,1);
lmiterm([2 3 1 W2],R^0.5,1);
lmiterm([2 2 2 0],-1);
lmiterm([2 3 2 0],0);
lmiterm([2 3 3 0],-1);

%% 3 LMI
lmiterm([-3 1 1 gama],1,1);
%% 4 LMI
lmiterm([-4 1 1 W1],1,1);

%% ALOCAÇÃO DE POLOS
% Primeira Faixa
lmiterm([5 1 1 W1],A,1,'s');
lmiterm([5 1 1 W2],B,1,'s');
lmiterm([5 1 1 W1],2*alfa,1);

% Segunda Faixa
lmiterm([6 1 1 W1],-A,1,'s');
lmiterm([6 1 1 W2],-B,1,'s');
lmiterm([6 1 1 W1],-2*beta,1);

% Setor Cônico
lmiterm([7 1 1 W1],sind(theta)*A,1,'s');
lmiterm([7 1 1 W2],sind(theta)*B,1,'s');

lmiterm([7 1 2 W1],cosd(theta)*A,1);
lmiterm([7 1 2 W1],-cosd(theta),A');
lmiterm([7 1 2 W2],cosd(theta)*B,1);
lmiterm([7 1 2 -W2],-cosd(theta),B');

lmiterm([7 2 2 W1],sind(theta)*A,1,'s');
lmiterm([7 2 2 W2],sind(theta)*B,1,'s');
%% Solver
LQR_lmi = getlmis;
c = [1 zeros(1,n_W2-1)]';
options = [1e-8,300,1e9,20,1];

[c_opt,x_opt] = mincx(LQR_lmi,c,options);

%% Obtenção dos Resultados
gama_opt = dec2mat(LQR_lmi,x_opt,gama);
W1_opt = dec2mat(LQR_lmi,x_opt,W1);
W2_opt = dec2mat(LQR_lmi,x_opt,W2);

K = W2_opt*inv(W1_opt);















