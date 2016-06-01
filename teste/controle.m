%% ============Controle da velocidade angular do carro=================
%
% Sistema Mamdani
% andMethod: 'min'
% orMethod: 'max'
% defuzzMethod: 'centroid'
% impMethod: 'min'
% aggMethod: 'max'

% Strategy selector
% If the car is touching the curb and its orientation is straight, then stop; it is successfully parked.
% If there is enough back space, then go backward.
% If there is enough front space, then go forward.

function [vangular] = controle (sentido, distCalcada, orientacao)

% clear all
%input = [1 0];
%distCalcada = input(1)
%orientacao = input(2)

pkg load fuzzy-logic-toolkit

MIN_DIST = 0;
MAX_DIST = 10;
MIN_ORIENT = -0.9;
MAX_ORIENT = 0.9;
MIN_OMEGA = -1.2;
MAX_OMEGA = 1.2;

if (distCalcada <= MIN_DIST*0.999)
  distCalcada = MIN_DIST + 0.001;
endif
if (distCalcada >= MAX_DIST*0.999)
  distCalcada = MAX_DIST - 0.001;
endif

if (orientacao < MIN_ORIENT*0.999)
  orientacao = MIN_ORIENT + 0.001;
endif
if (orientacao > MAX_ORIENT*0.999)
  orientacao = MAX_ORIENT - 0.001;
endif

  
%% Fuzzificação
% Aqui esta etapa ocorre junto com a declaração das funções de pertinência
% Como
 
% Nomenclatura das variáveis
% service degree rule N -&gt; dsrN
% Grau de Pertinência do Serviço Regra N
 
% distCalcada
% Ao entrar com a variável na função gaussmf() eu já tenho a fuzzificação, ou seja, o grau de pertinência deste valor ao Conjunto Fuzzy "distancia do obstaculo".
% Se eu tivesse dado o intervalo de x, neste caso de 0 a 1000 às funcoes trapmf, eu teria uma curva representando a FP do Conjunto Fuzzy distCalcada.
distTouch = trapmf(distCalcada, [MIN_DIST 0.1 0.4 0.5]); % Touching
distClose = trapmf(distCalcada, [0.4 0.5 0.9 1.0]); % Close
distFar = trapmf(distCalcada, [0.9 1.0 9.9 MAX_DIST]); % Far
 
% orientacao
% Mesma explicação da de cima, mas para uma função de pertinência que representa o Conjunto Fuzzy orientacao (-pi/2 a pi/2).
orientMuitoDir = trapmf(orientacao, [MIN_ORIENT -0.8 -0.5 -0.4]); % muito_direita
orientPoucoDir = trapmf(orientacao, [-0.5 -0.4 -0.2 -0.1]); % pouco_direita
orientReto = trapmf(orientacao, [-0.2 -0.1 0.1 0.2]); % reto
orientPoucoEsq = trapmf(orientacao, [0.1 0.2 0.4 0.5]); % pouco_esquerda
orientMuitoEsq = trapmf(orientacao, [0.4 0.5 0.8 MAX_ORIENT]); % muito_esquerda
 
% Saida: Velocidade Angular (-1 a 1)
omega = MIN_OMEGA:0.1:MAX_OMEGA;
omgMuitoHorario = trimf(omega, [MIN_OMEGA -0.8 -0.4]); % muito_horario
omgPoucoHorario = trimf(omega, [-0.5 -0.3 -0.1]); % pouco_horario
omgZero = trimf(omega, [-0.15 0 0.15]); % zero
omgPoucoAntiHor = trimf(omega, [0.1 0.3 0.5]); % pouco_antihorario
omgMuitoAntiHor = trimf(omega, [0.4 0.8 MAX_OMEGA]); % muito_antihorario
 
% ========Aplicando as Regras========
% Distancia & orientacao -> Velocidade angular
% RX(distCalcada, orientacao, omega)

% Sentido para tras
if (sentido == -1)
	R1 = fuzzyficar2 (distFar, orientMuitoEsq, omgMuitoHorario);
	R2 = fuzzyficar2 (distFar, orientPoucoEsq, omgMuitoHorario);
	R3 = fuzzyficar2 (distFar, orientReto, omgPoucoHorario);
	R4 = fuzzyficar2 (distFar, orientPoucoDir, omgPoucoHorario);
	R5 = fuzzyficar2 (distFar, orientMuitoDir, omgZero);

	R6 = fuzzyficar2 (distClose, orientMuitoEsq, omgPoucoHorario);
	R7 = fuzzyficar2 (distClose, orientPoucoEsq, omgPoucoHorario);
	R8 = fuzzyficar2 (distClose, orientReto, omgPoucoAntiHor);
	R9 = fuzzyficar2 (distClose, orientPoucoDir, omgPoucoAntiHor);
	R10 = fuzzyficar2 (distClose, orientMuitoDir, omgPoucoAntiHor);
endif
% Sentido para frente
if (sentido == 1)
	R1 = fuzzyficar2 (distFar, orientMuitoEsq, omgPoucoHorario);
	R2 = fuzzyficar2 (distFar, orientPoucoEsq, omgZero);
	R3 = fuzzyficar2 (distFar, orientReto, omgZero);
	R4 = fuzzyficar2 (distFar, orientPoucoDir, omgPoucoAntiHor);
	R5 = fuzzyficar2 (distFar, orientMuitoDir, omgMuitoAntiHor);

	R6 = fuzzyficar2 (distClose, orientMuitoEsq, omgPoucoHorario);
	R7 = fuzzyficar2 (distClose, orientPoucoEsq, omgPoucoHorario);
	R8 = fuzzyficar2 (distClose, orientReto, omgZero);
	R9 = fuzzyficar2 (distClose, orientPoucoDir, omgPoucoAntiHor);
	R10 = fuzzyficar2 (distClose, orientMuitoDir, omgPoucoAntiHor);
endif

R11 = fuzzyficar1 (distTouch, omgZero);
 
% Agregação das regras
% Aqui as 13 regras são agregadas para formar um Conjunto Fuzzy de Saída do Sistema de Inferência.
% Neste caso usamos o operador MAX
output = max(R1,R2);
output = max(output,R3);
output = max(output,R4);
output = max(output,R5);
output = max(output,R6);
output = max(output,R7);
output = max(output,R8);
output = max(output,R9);
output = max(output,R10);
output = max(output,R11);
 
%% =======Defuzzificacao========
% Nesta etapa através de um método obtem-se um resultado escalar do conjunto fuzzy resultante do sistema de inferência.
% Aqui usamos o método da Centróide, mas existem outros.
 
vangular = defuzz(omega, output, 'centroid');
  
endfunction

% ====EOF====

