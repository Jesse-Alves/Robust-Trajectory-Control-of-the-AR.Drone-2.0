%===============================================================
%=================== LQR ROBUSTO VIA LMI =======================
%===============================================================

function K = lqrrobusto(A,B,C,D,Q,R,x0,alfa,beta,theta)
%% Dimensionamento das Matrizes
[n,p,ra] = size(A);
[p,m,rb] = size(B);

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
lmiterm([-2 1 1 gama],1,1);
%% 3 LMI
lmiterm([-3 1 1 W1],1,1);

%% LMIs DO LQR
q = 4;
for i=1:ra
    for j=1:rb
        lmiterm([q 1 1 W1],A(:,:,i),1,'s');
        lmiterm([q 1 1 W2],B(:,:,j),1,'s');
        lmiterm([q 2 1 W1],Q^0.5,1);
        lmiterm([q 3 1 W2],R^0.5,1);
        lmiterm([q 2 2 0],-1);
        lmiterm([q 3 2 0],0);
        lmiterm([q 3 3 0],-1);

        q = q + 1;
    end
end

%% LMIs da ALOCAÇÃO DE POLOS
for i=1:ra
    for j=1:rb
    % Primeira Faixa
    lmiterm([q 1 1 W1],A(:,:,i),1,'s');
    lmiterm([q 1 1 W2],B(:,:,j),1,'s');
    lmiterm([q 1 1 W1],2*alfa,1);

    % Segunda Faixa
    q = q + 1;
    lmiterm([q 1 1 W1],-A(:,:,i),1,'s');
    lmiterm([q 1 1 W2],-B(:,:,j),1,'s');
    lmiterm([q 1 1 W1],-2*beta,1);

    % Setor Cônico
    q = q + 1;
    lmiterm([q 1 1 W1],sind(theta)*A(:,:,i),1,'s');
    lmiterm([q 1 1 W2],sind(theta)*B(:,:,j),1,'s');
    lmiterm([q 1 2 W1],cosd(theta)*A(:,:,i),1);
    lmiterm([q 1 2 W1],-cosd(theta),A(:,:,i)');
    lmiterm([q 1 2 W2],cosd(theta)*B(:,:,j),1);
    lmiterm([q 1 2 -W2],-cosd(theta),B(:,:,j)');
    lmiterm([q 2 2 W1],sind(theta)*A(:,:,i),1,'s');
    lmiterm([q 2 2 W2],sind(theta)*B(:,:,j),1,'s');

    q = q + 1;
    end
end
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

