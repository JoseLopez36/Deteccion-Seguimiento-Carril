% Modelo:
% Tesla Model 3 Long Range RWD 2024
% https://www.caranddriver.com/reviews/a62446256/2024-tesla-model-3-long-range-rwd-test/

% Parámetros del modelo longitudinal:
% Masa
m = 1731; % kg (Aproximadamente 3817 lb)
% Ancho de vía
B = 1.585; % m
% Altura
H = 1.440; % m
% Área frontal
A = 0.9 * B * H; % m^2
% Factor de resistencia aerodinámico
Cd = 0.219; % Valor oficial Tesla Model 3 Highland
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
L = 2.875; % m
% Distancia del CG al eje delantero (asumiendo reparto de pesos 47% Delante / 53% Detrás)
a = 1.524; % m
% Distancia del CG al eje trasero
b = L - a; % m
% Momento de inercia respecto al eje Z (Estimación m * a * b)
Iz = 3500; % kgm2
% Rigidez de deriva delantera (Estimado)
Caf = 55000;
% Rigidez de deriva trasera (Estimado)
Car = 60000;
% Fracción de la masa total soportada en el eje delantero
mf = (m * b) / L; % kg
% Fracción de la masa total soportada en el eje trasero
mr = m - mf; % kg
% Gradiente de subviraje
Kv = (mf / (2 * Caf)) - (mr / (2 * Car));

% Parámetros de la rueda (Neumáticos 235/45R-18):
% Coeficiente de resistencia a la rodadura
fw = 0.01;
% Momento de inercia de la rueda
Iw = 1.0; % kgm2
% Radio efectivo de la rueda
rw = 0.334; % m
% Coeficiente de fricción viscosa
bw = 0;
% Fuerza de fricción constante
fw_const = 0;