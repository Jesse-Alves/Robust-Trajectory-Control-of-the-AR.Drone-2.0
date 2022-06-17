% =========================================================================
% ============ PROJETO DE CONTROLE COM SISTEMA DESACOPLADO ================
% =========================================================================

clear all;
clc;
close all;
%% Tipo de Trajetoria
% circulo = 1 /// oito inclinado = 0
tipo = 1;

if tipo == 1
    chave_circular = 1;
    chave_oito = 0;
else
    chave_circular = 0;
    chave_oito = 1;
end
%% ==========> Parametros do AR.Drone 2.0 <============
Cx = 0.3;
Cy = 0.1;
g = 9.8;
Kz = 1.3;
Kpsi = 1;
tz = 0.4;
tpsi = 0.3;
wfi = 4.47;
wtheta = 4.47;
zetafi = 0.5;
zetatheta = 0.5;
Kfi = 1.5; 
Ktheta = 1.5;
Cx = 0.3;
Cy = 0.1;

%% PARAMETROS INCERTOS
zmax1 = 0.2; zmax2 = 2;
psimax1 = 1; psimax2 = 5.15;
fimax1 = 0.2; fimax2 = 0.52
thetamax1 = 0.15; thetamax2 = 0.52;
%% ===============================================
%  ========== SISTEMA LINEARIZADO ================
%  ===============================================
s = tf([1 0],[1]);

%% VARIAVEL Z
Az = [0 1;
      0 -1/tz];
Bz1 = [0;
      (Kz*zmax1)/(tz)];
Bz2 = [0;
      (Kz*zmax2)/(tz)];
Cz = [1 0];
Dz = 0;

Az_a = [Az zeros([size(Az,1),size(Bz1,2)]);-Cz zeros(size(Cz,1))];

Bz_a1 = [Bz1;-Dz];
Bz_a2 = [Bz1;-Dz];
Bz_a=[];
Bz_a(:,:,1) = Bz_a1;
Bz_a(:,:,2) = Bz_a2;

Cz_a = [Cz zeros(size(Cz,1))];

%Gz = ss(Az_a,Bz_a,Cz_a,Dz);
%Gz2 = ss(Az,Bz,Cz,Dz);

%% VARIAVEL PSI
Apsi = [0 1;
        0 -1/tpsi];
Bpsi1 = [0;
       (Kpsi*psimax1)/(tpsi)];
Bpsi2 = [0;
       (Kpsi*psimax2)/(tpsi)];   
Cpsi = [1 0];
Dpsi = 0;
   
Apsi_a = [Apsi zeros([size(Apsi,1),size(Bpsi1,2)]);-Cpsi zeros(size(Cpsi,1))];

Bpsi_a1 = [Bpsi1;-Dpsi];
Bpsi_a2 = [Bpsi2;-Dpsi];
Bpsi_a=[];
Bpsi_a(:,:,1) = Bpsi_a1;
Bpsi_a(:,:,2) = Bpsi_a2;

Cpsi_a = [Cpsi zeros(size(Cpsi,1))];

%Gpsi = ss(Apsi_a,Bpsi_a,Cpsi_a,Dpsi);
%Gpsi2 = ss(Apsi,Bpsi,Cpsi,Dpsi);

%% VARIAVEL FI-Y 
Afi_y = [0 1 0 0;
       -wfi^2 -2*zetafi*wfi 0 0;
         0 0 0 1;
         -g 0 0 -Cy];
     
Bfi_y1 = [0;Kfi*fimax1*wfi^2;0;0];
Bfi_y2 = [0;Kfi*fimax2*wfi^2;0;0];
    
Cfi_y = [0 0 1 0];
Dfi_y = [0];
    
Afi_y_a = [Afi_y zeros([size(Afi_y,1),size(Bfi_y1,2)]);
          -Cfi_y zeros(size(Cfi_y,1))];

Bfi_y_a1 = [Bfi_y1;-Dfi_y];
Bfi_y_a2 = [Bfi_y2;-Dfi_y];

Bfi_y_a=[];
Bfi_y_a(:,:,1) = Bfi_y_a1;
Bfi_y_a(:,:,2) = Bfi_y_a2;

Cfi_y_a = [Cfi_y zeros(size(Cfi_y,1))];

%Gfi_y = ss(Afi_y_a,Bfi_y_a,Cfi_y_a,Dfi_y);
%Gfi_y2 = ss(Afi_y,Bfi_y,Cfi_y,Dfi_y);

%% VARIAVEL THETA-X
Atheta_x = [0 1 0 0;
       -wtheta^2 -2*zetatheta*wtheta 0 0;
          0 0 0 1;
          g 0 0 -Cx];    
      
Btheta_x1 = [0;Ktheta*thetamax1*wtheta^2;0;0];
Btheta_x2 = [0;Ktheta*thetamax2*wtheta^2;0;0];
    
Ctheta_x = [0 0 1 0];
Dtheta_x = [0];

Atheta_x_a = [Atheta_x zeros([size(Atheta_x,1),size(Btheta_x1,2)]);
              -Ctheta_x zeros(size(Ctheta_x,1))];

Btheta_x_a1 = [Btheta_x1;-Dtheta_x];
Btheta_x_a2 = [Btheta_x2;-Dtheta_x];

Btheta_x_a=[];
Btheta_x_a(:,:,1) = Btheta_x_a1;
Btheta_x_a(:,:,2) = Btheta_x_a2;

Ctheta_x_a = [Ctheta_x zeros(size(Ctheta_x,1))];

%Gtheta_x = ss(Atheta_x_a,Btheta_x_a,Ctheta_x_a,Dtheta_x);
%Gtheta_x2 = ss(Atheta_x,Btheta_x,Ctheta_x,Dtheta_x);

%% ===============================================
%%  ========= PROJETO DOS CONTROLADORES ===========
%%  ===============================================
%% ================ MATRIZES DE PONDERACAO Q e R ====================
% VARIAVEL Z
Qz = [1 0 0;
      0 1 0;
      0 0 1200]; %120
Rz = 8; %8
% VARIAVEL PSI
Qpsi = [1 0 0;
        0 1 0;
        0 0 1];
Rpsi = 15;
% VARIAVEL FI-Y
Qfi_y = [1 0 0 0 0;
         0 1 0 0 0;
         0 0 1 0 0;
         0 0 0 1 0;
         0 0 0 0 240];%240
Rfi_y = 0.1;%0.1
% VARIAVEL THETA-X
Qtheta_x = [1 0 0 0 0;
            0 1 0 0 0;
            0 0 1 0 0;
            0 0 0 1 0;
            0 0 0 0 1000];
Rtheta_x = 3;

%% CALCULO DO GANHO K AUMENTADO - VIA LMI
disp('CONTROLE VIA LMI')

%Estado inicial
x0 = [1 1 1]';

alfa = 2; beta = 3; theta = 15; %Alocacao de Polos
K_z2 = lqrrobusto(Az_a,Bz_a,Cz_a,Dz,Qz,Rz,x0,alfa,beta,theta)

alfa = 0.4; beta = 4; theta = 60; %Alocacao de Polos
K_psi2 = lqrrobusto(Apsi_a,Bpsi_a,Cpsi_a,Dpsi,Qpsi,Rpsi,x0,alfa,beta,theta)

%Estado inicial
x0 = [1 1 1 1 1]';

alfa = 0.05; beta = 22; theta = 85; %Alocacao de Polos
Kfi_y2 = lqrrobusto(Afi_y_a,Bfi_y_a,Cfi_y_a,Dfi_y,Qfi_y,Rfi_y,x0,alfa,beta,theta)

alfa = 0.05; beta = 22; theta = 85; %Alocacao de Polos
Ktheta_x2 = lqrrobusto(Atheta_x_a,Btheta_x_a,Ctheta_x_a,Dtheta_x,Qtheta_x,Rtheta_x,x0,alfa,beta,theta)

%% =============================================================
%  ===== GANHO DE REALIMENTACAO DE ESTADO E INTEGRAL ===========
%  =============================================================
%% Z
Ki_z = K_z2(:,3);
K_z = K_z2(:,1:2);
%% PSI
Ki_psi = K_psi2(:,3);
K_psi = K_psi2(:,1:2);
%% FI-Y
Ki_fi_y = Kfi_y2(:,5);
Kfi_y = Kfi_y2(:,1:4);
%% THETA-X
Ki_theta_x = Ktheta_x2(:,5);
Ktheta_x = Ktheta_x2(:,1:4);

%% PARAMETROS VARIAVEIS
%Padrao
zmax = 1;
psimax = 1.74;
fimax = 0.26;
thetamax = 0.26;

%Limite inferior
zmax = 0.95;
psimax = 1;
fimax = 0.2;
thetamax = 0.15;

%Limite superior
zmax = 2;
psimax = 5.15;
fimax = 0.45;
thetamax = 0.52;

%% Executar arquivo .slx
sim('Malha_de_Controle_Drone')

%% POLOS DO SISTEMA
Af_z = Az_a + Bz_a(:,:,1)*K_z2;
polosZ = eig(Af_z)

Af_psi = Apsi_a + Bpsi_a(:,:,1)*K_psi2;
polosPSI = eig(Af_psi)

Af_fi = Afi_y_a(:,:,1) + Bfi_y_a(:,:,1)*Kfi_y2;
polosFI = eig(Af_fi)

Af_theta = Atheta_x_a(:,:,1) + Btheta_x_a(:,:,1)*Ktheta_x2;
polosTHETA = eig(Af_theta)

%% =============================================================
%%  ================== RESULTADOS DA SIMULACAO ==================
%%  =============================================================

%% ===============> Plot da Trajetoria em 3D <===================

if tipo == 1
    %REFERENCIA CIRCULAR
    figure
    t = tout;
    xref = sin(0.8*t);
    yref = cos(0.8*t);
    zref = 1.2*ones([1 length(t)]);
    psiref = 0;
    plot3(xref,yref,zref,'--r','LineWidth',2);
    title('Trajetória Circular do AR.Drone 2.0')
else
    %REFERENCIA OITO INCLINADO
    figure
    t = tout;
    xref = 0.5*sin(0.8*t);
    yref = sin(0.4*t);
    zref = 1.2 + 0.5*sin(0.4*t);
    psiref = -((pi/6)*sin(0.4*t));
    plot3(xref,yref,zref,'--r','LineWidth',2);
    title('Trajetória Circular do AR.Drone 2.0')
end

%Simulacao
hold on
plot3(x,y,z,'LineWidth',2);
xlabel('x')
ylabel('y')
zlabel('z')
legend('Trajetória Referência','Trajetória Realizada pelo Drone')
grid on
axis([-1.5 1.5 -1.5 1.5 0 2.2]) %Ajuste do Grafico
% ==================================================================

%% Plot dos Sinais de Saida e Sinais de Controle Individuais 
%% X
figure
plot(tout,x)
hold on
plot(t,xref)
legend('X da simulação','X de referência')
title('Saída X')
grid on
axis([0 60 -3 3])

figure
plot(tout,u_theta)
title('Sinal de Controle - U_{\theta}')
grid on
%% Y
figure
plot(tout,y)
hold on
plot(t,yref)
legend('Y da simulação','Y de referência')
title('Saída Y')
grid on
axis([0 40 -1.5 1.5])

figure
plot(tout,u_fi)
title('Sinal de Controle - U_{\phi}')
grid on

%% Z
figure
plot(tout,z)
hold on
plot(t,zref)
legend('Z da simulação','Z de referência')
title('Saída Z')
grid on

figure
plot(tout,uz)
title('Sinal de Controle - Uz')
grid on

%% PSI
figure
plot(tout,e_psi)
hold on
plot(t,psiref)
legend('\psi da simulação','\psi de referência')
title('Saída \psi')
grid on

figure
plot(tout,u_psi)
title('Sinal de Controle - U_{\psi}')
grid on