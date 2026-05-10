% Modelo:
% Lexus RX450h 2015
% https://www.caranddriver.com/lexus/rx/specs/2015/lexus_rx_lexus-rx450h_2015

% Parametros del modelo longitudinal:
% Masa
m = 2110; % kg
% Ancho de vía
B = 1.626; % m
% Altura
H = 1.694; % m
% Área frontal
A = 0.9 * B * H; % m^2
% Factor de resistencia aerodinámico
Cd = 0.4; % Estimado
% Factor de fricción
f = 0.015; % Estimado
% Densidad del aire
rho = 1.202; % kg/m^3
% Gravedad
g = 9.81; % m/s^2
% Peso
W = m * g; % N

% Parámetros del modelo lateral:
% Distancia entre ejes
L = 2.741; % m
% Distancia del CG al eje delantero
a = 1.234; % m
% Distancia del CG al eje trasero
b = L - a; % m
% Momento de inercia respecto al eje Z
Iz = 3960; % kgm2
% Rigidez de deriva delantera
Caf = 44000;
% Rigidez de deriva trasera
Car = 47000;
% Fracción de la masa total soportada en el eje delantero
mf = (m * b) / L; % kg
% Fracción de la masa total soportada en el eje trasero
mr = m - mf; % kg
% Gradiente de subviraje
Kv = (mf / (2 * Caf)) - (mr / (2 * Car));

% Parámetros de la rueda:
% Coeficiente de resistencia a la rodadura
fw = 0.01;
% Momento de inercia de la rueda
Iw = 0.65;
% Radio efectivo de la rueda
rw = 0.371;
% Coeficiente de fricción viscosa
bw = 0;
% Fuerza de fricción constante
fw = 0;