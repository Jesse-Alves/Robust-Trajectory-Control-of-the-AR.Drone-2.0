%%  TESTE DE ROBUSTEZ
tz2 = 0.8;
tpsi2 = 0.4;
Cx2 = 1;
Cy2 = 3;

% tz2 = 0.45;
% tpsi2 = 0.4;
% Cx2 = 0.9;
% Cy2 = 0.5;

tz = tz2;
tpsi = tpsi2;
Cx = Cx2;
Cy = Cy2;
%% SIMULINK
sim('Malha_de_Controle_Drone')

%% =============================================================
%% ================ SISTEMA EM MALHA FECHADA ===================
%% =============================================================
%% Tamanho da Fonte
fontsize = 20;
fontsize2 = 18;

%% VARIÁVEL FI-Y 
Afi_y = [0 1 0 0;
       -wfi^2 -2*zetafi*wfi 0 0;
         0 0 0 1;
         -g 0 0 -Cy];
Bfi_y = [0;
        Kfi*fimax*wfi^2;
        0;
        0];
Cfi_y = [0 0 1 0];
Dfi_y = [0];
%% VARIÁVEL THETA-X
Atheta_x = [0 1 0 0;
       -wtheta^2 -2*zetatheta*wtheta 0 0;
          0 0 0 1;
          g 0 0 -Cx];
Btheta_x = [0;
        Ktheta*thetamax*wtheta^2;
          0;
          0];
Ctheta_x = [0 0 1 0];
Dtheta_x = [0];
%% VARIÁVEL Z
Az = [0 1;
      0 -1/tz];
Bz = [0;
      (Kz*zmax)/(tz)];
Cz = [1 0];
Dz = 0;
%% VARIÁVEL PSI
Apsi = [0 1;
        0 -1/tpsi];
Bpsi = [0;
       (Kpsi*psimax)/(tpsi)];
Cpsi = [1 0];
Dpsi = 0;
 %% =============================================================
%% Z
Af_z = [Az+Bz*K_z Bz*Ki_z;
        -(Cz+Dz*K_z) -Dz*Ki_z];
Bf_z = [zeros(size(Az,1),1);eye(1,1)];
Cf_z = [Cz+Dz*Kz Dz*Ki_z];
Df_z = Dz;

SYSz = ss(Af_z,Bf_z,Cf_z,Df_z)
polosZ = eig(Af_z)
%% PSI
Af_psi = [Apsi+Bpsi*K_psi Bpsi*Ki_psi;
        -(Cpsi+Dpsi*K_psi) -Dpsi*Ki_psi];
Bf_psi = [zeros(size(Apsi,1),1);eye(1,1)];
Cf_psi = [Cpsi+Dpsi*Kpsi Dpsi*Ki_psi];
Df_psi = Dpsi;

SYSpsi = ss(Af_psi,Bf_psi,Cf_psi,Df_psi)
polosPSI = eig(Af_psi)
%% FI
Af_fi = [Afi_y+Bfi_y*Kfi_y Bfi_y*Ki_fi_y;
        -(Cfi_y+Dfi_y*Kfi_y) -Dfi_y*Ki_fi_y];
Bf_fi = [zeros(size(Afi_y,1),1);eye(1,1)];
Cf_fi = [Cfi_y+Dfi_y*Kfi_y Dfi_y*Ki_fi_y];
Df_fi = Dfi_y;

SYSfi = ss(Af_fi,Bf_fi,Cf_fi,Df_fi)
polosFI = eig(Af_fi)
%% THETA
Af_theta = [Atheta_x+Btheta_x*Ktheta_x Btheta_x*Ki_theta_x;
        -(Ctheta_x+Dtheta_x*Ktheta_x) -Dtheta_x*Ki_theta_x];
Bf_theta = [zeros(size(Atheta_x,1),1);eye(1,1)];
Cf_theta = [Ctheta_x+Dtheta_x*Ktheta_x Dtheta_x*Ki_theta_x];
Df_theta = Dtheta_x;

SYStheta = ss(Af_theta,Bf_theta,Cf_theta,Df_theta)
polosTHETA = eig(Af_theta)

%% ====================== PZMAP =============================

%% Z
figure
pzplot(SYSz,'r')

%Ajustes
title('Polos de Malha Fechada do Sistema','FontSize',fontsize)
%Configurações
xlabel('Eixo Real','FontSize',fontsize)
ylabel('Eixo Imaginário','FontSize',fontsize)
grid on
hm = findobj(gca, 'Type', 'Line');          % Handle To 'Line' Objects
hm(2).MarkerSize = 20;                      % ‘Zero’ Marker
hm(3).MarkerSize = 20;                      % ‘Pole’ Marker
hm(2).LineWidth = 5;                      % ‘Zero’ Marker
hm(3).LineWidth = 5;                      % ‘Pole’ Marker
%% PSI
hold on
pzplot(SYSpsi,'b')
hm = findobj(gca, 'Type', 'Line');          % Handle To 'Line' Objects
hm(2).MarkerSize = 20;                      % ‘Zero’ Marker
hm(3).MarkerSize = 20;                      % ‘Pole’ Marker
hm(2).LineWidth = 5;                      % ‘Zero’ Marker
hm(3).LineWidth = 5;                      % ‘Pole’ Marker

%% FI
hold on
pzplot(SYSfi,'k')
hm = findobj(gca, 'Type', 'Line');          % Handle To 'Line' Objects
hm(2).MarkerSize = 20;                      % ‘Zero’ Marker
hm(3).MarkerSize = 20;                      % ‘Pole’ Marker
hm(2).LineWidth = 5;                      % ‘Zero’ Marker
hm(3).LineWidth = 5;                      % ‘Pole’ Marker

%% THETA
hold on
pzplot(SYStheta,'m')
hm = findobj(gca, 'Type', 'Line');          % Handle To 'Line' Objects
hm(2).MarkerSize = 20;                      % ‘Zero’ Marker
hm(3).MarkerSize = 20;                      % ‘Pole’ Marker
hm(2).LineWidth = 5;                      % ‘Zero’ Marker
hm(3).LineWidth = 5;                      % ‘Pole’ Marker

legend('Subsistema z','Subsistema \psi','Subsistema \phi','Subsistema \theta')

%% =============================================================
%%  ================== RESULTADOS DA SIMULAÇÃO ==================
%%  =============================================================
%% ===============> Plot da Trajetória em 3D <===================
if tipo == 1
    %REFERÊNCIA CIRCULAR
    figure
    t = tout;
    xref = sin(0.8*t);
    yref = cos(0.8*t);
    zref = 1.2*ones([1 length(t)]);
    psiref = 0;
    plot3(xref,yref,zref,'r','LineWidth',2);
    title('Trajetória Circular','FontSize',fontsize)
    
    xref = sin(0.8*t-0.65);
    yref = cos(0.8*t-0.65);
else
    %REFERÊNCIA OITO INCLINADO
    figure
    t = tout;
    xref = 0.5*sin(0.8*t);
    yref = sin(0.4*t);
    zref = 1.2 + 0.5*sin(0.4*t);
    psiref = -((pi/6)*sin(0.4*t));
    plot3(xref,yref,zref,'r','LineWidth',2);
    title('Trajetória em Formato de Oito Inclinado','FontSize',fontsize)
    
    xref = 0.5*sin(0.8*t-0.65);
    yref = sin(0.4*t-0.33);
    zref = 1.2 + 0.5*sin(0.4*t-0.4);
    psiref = -((pi/6)*sin(0.4*t+0.9));
end

%Simulação
hold on
plot3(x,y,z,'LineWidth',2);
xlabel('Eixo x','FontSize',fontsize)
ylabel('Eixo y','FontSize',fontsize)
zlabel('Eixo z','FontSize',fontsize)
lgd = legend('Trajetória Referência','Trajetória Realizada pelo Drone')
lgd.FontSize = fontsize2;
grid on
axis([-1.5 1.5 -1.5 1.5 0 2.2]) %Ajuste do Gráfico
% ==================================================================

