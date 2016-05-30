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


function [vangular] = controle (distCalcada, orientacao)

% clear all
%input = [1 0];
%distCalcada = input(1)
%orientacao = input(2)

pkg load fuzzy-logic-toolkit

MIN_DIST = 0;
MAX_DIST = 10;
MIN_ORIENT = -0.9;
MAX_ORIENT = 0.9;

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
dist1 = trimf(distCalcada, [1 2 MAX_DIST]); % Far
dist2 = trimf(distCalcada, [0.4 1 1.4]); % Close
dist3 = trimf(distCalcada, [MIN_DIST 0.3 0.6]); % Touching
 
% orientacao
% Mesma explicação da de cima, mas para uma função de pertinência que representa o Conjunto Fuzzy orientacao (-pi/2 a pi/2).
orient1 = trimf(orientacao, [MIN_ORIENT -0.7 -0.4]); % muito_horario
orient2 = trimf(orientacao, [-0.5 -0.3 -0.1]); % pouco_horario
orient3 = trimf(orientacao, [-0.15 0 0.15]); % reto
orient4 = trimf(orientacao, [0.1 0.3 0.5]); % pouco_antihorario
orient5 = trimf(orientacao, [0.4 0.7 MAX_ORIENT]); % muito_antihorario
 
% Saida: Velocidade Angular (-1 a 1)
omega = -1.2:0.1:1.2;
omg1 = trimf(omega, [-1.2 -0.8 -0.4]); % muito_dir
omg2 = trimf(omega, [-0.5 -0.3 -0.1]); % pouco_dir
omg3 = trimf(omega, [-0.15 0 0.15]); % zero
omg4 = trimf(omega, [0.1 0.3 0.5]); % pouco_esq
omg5 = trimf(omega, [0.4 0.8 1.2]); % muito_esq
 
% ========Aplicando as Regras========
% Distancia & orientacao -> Velocidade angular
% Marcha a ré
% R1: if(distCalcada is far & orientacao is reto) -> omega = muito_dir;
% R2: if(distCalcada is far & orientacao is pouco_horario) -> omega = pouco_dir;
% R3: if(distCalcada is far & orientacao is muito_horario) -> omega = zero;
% R4: if(distCalcada is far & orientacao is pouco_antihorario) -> omega = muito_dir;
% R5: if(distCalcada is far & orientacao is muito_antihorario) -> omega = muito_dir;

% R6: if(distCalcada is close & orientacao is muito_horario) -> omega = muito_esq;
% R7: if(distCalcada is close & orientacao is pouco_horario) -> omega = pouco_esq;
% R8: if(distCalcada is close & orientacao is reto) -> omega = pouco_esq;
% R9: if(distCalcada is close & orientacao is pouco_antihorario) -> omega = pouco_dir;
% R10: if(distCalcada is close & orientacao is muito_antihorario) -> omega = muito_dir;

% R11: if(distCalcada is touching) -> omega = zero;

R1 = fuzzyficar2 (dist1, orient1, omg1);
R2 = fuzzyficar2 (dist1, orient2, omg2);
R3 = fuzzyficar2 (dist1, orient3, omg3);
R4 = fuzzyficar2 (dist1, orient4, omg1);
R5 = fuzzyficar2 (dist1, orient5, omg1);

R6 = fuzzyficar2 (dist2, orient3, omg5);
R7 = fuzzyficar2 (dist2, orient2, omg4);
R8 = fuzzyficar2 (dist2, orient1, omg4);
R9 = fuzzyficar2 (dist2, orient4, omg2);
R10 = fuzzyficar2 (dist2, orient5, omg1);

R11 = fuzzyficar1 (dist3, omg3);
 
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



%{

2. Forward approaching the curb
2.1 Far -> R2
2.1.1 if(distCalcada == far & orientacao < R2) -> steering_fw_approaching = L2;
2.1.2 if(distCalcada == far & orientacao > R2) -> steering_fw_approaching = R2;
2.1.3 if(distCalcada == far & orientacao == R2) -> steering_fw_approaching = straight;
2.2 Close -> R1
2.2.1 if(distCalcada == far & orientacao < R1) -> steering_fw_approaching = L1;
2.2.2 if(distCalcada == far & orientacao > R1) -> steering_fw_approaching = R1;
2.2.3 if(distCalcada == far & orientacao == R1) -> steering_fw_approaching = straight;
2.3 Touching -> Straight
2.3.1 if(distCalcada == touching) -> steering_fw_approaching = straight;

3. Forward moving away from the rear obstacle
3.1 Far -> Straight
3.1.1 if(distCalcada == far & orientacao > straight) -> steering_fw_mov_away = L3;
3.1.2 if(distCalcada == far & orientacao < straight) -> steering_fw_mov_away = R3;
3.1.3 if(distCalcada == far & orientacao == straight) -> steering_fw_mov_away = straight;

%}
