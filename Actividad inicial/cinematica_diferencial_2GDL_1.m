%% Cinemática diferencial de robot péndulo de 2GDL
% Oscar Ortiz Torres A01769292

clear all
close all
clc

% Declaración de variables simbólicas
syms th1(t) l1 th2(t) l2

% Configuración del robot, '0' para la junta rotacional, '1' para junta prismática
RP = [0; 0]; % Rotacional Prismatica, el numero de 0s es por el numero de GDL

% Creación del vector de coordenadas generalizadas
Q = [th1, th2];
disp('Coordenadas_generalizadas');
pretty(Q);

% Creación del vector de velocidades generalizadas
Qp = diff(Q, t);
disp('Velocidades generalizadas');
pretty(Qp);

% Número de grados de libertad del robot
GDL = size(RP, 1);

% Vector de traslación de la articulación 1 respecto a 0
P(:,:,1) = [
            l1*cos(th1);
            l1*sin(th1);
            0
           ];

% Matriz de rotación de la junta 1 respecto a la 0
R(:,:,1) = [
            cos(th1)    -sin(th1)   0;
            sin(th1)    cos(th1)    0;
            0           0           1
           ];

% Vector de traslación de la articulación 2 respecto a 1
P(:,:,2) = [
            l2*cos(th2);
            l2*sin(th2);
            0
           ];

% Matriz de rotación de la junta 2 respecto a la 1
R(:,:,2) = [
            cos(th2)    -sin(th2)   0;
            sin(th2)    cos(th2)    0;
            0           0           1
           ];

% Factores de acoplamiento
a1 = [0;0;0];
a2 = 1;

% Definición de matriz homogenea del primer eslabón
MH1 = [
      R(:,:,1) P(:,:,1);
      a1'      a2
      ];

% Definición de matriz homogenea del segundo eslabón
MH2 = [
      R(:,:,2) P(:,:,2);
      a1'      a2
      ];

% Calculo de matriz homogenea global del sistema
MG = MH1 * MH2;

% Definición de variables en base a la matriz homogenea resultante
x1 = MG(1,4);
y1 = MG(2,4);
z1 = MG(3,4);

% Derivada pacial de x respecto a th1
Jv11 = functionalDerivative(x1, th1);
% Derivada pacial de x respecto a th2
Jv12 = functionalDerivative(x1, th2);

% Derivada pacial de y respecto a th1
Jv21 = functionalDerivative(y1, th1);
% Derivada pacial de y respecto a th2
Jv22 = functionalDerivative(y1, th2);

% Derivada pacial de z respecto a th1
Jv31 = functionalDerivative(z1, th1);
% Derivada pacial de z respecto a th2
Jv32 = functionalDerivative(z1, th2);

disp('Cinemática diferencial de la posición del péndulo')

% Cinemática diferencial del péndulo a partir de la cinemática directa
jv_d = simplify([ ...
                Jv11, Jv12; ...
                Jv21, Jv22; ...
                Jv31, Jv32 ...
                ]);
pretty(jv_d)