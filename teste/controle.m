%% ============Controle da velocidade linear e angular do carro=================
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


function [vangular] = controle (distObst, orientacao)

% clear all
%input = [1 0];
%distObst = input(1)
%orientacao = input(2)

pkg load fuzzy-logic-toolkit
  
%% Fuzzificação
% Aqui esta etapa ocorre junto com a declaração das funções de pertinência
% Como
 
% Nomenclatura das variáveis
% service degree rule N -&gt; dsrN
% Grau de Pertinência do Serviço Regra N
 
% distObst
% Ao entrar com a variável na função gaussmf() eu já tenho a fuzzificação, ou seja, o grau de pertinência deste valor ao Conjunto Fuzzy "distancia do obstaculo".
% Se eu tivesse dado o intervalo de x, neste caso de 0 a 1000 às funcoes trapmf, eu teria uma curva representando a FP do Conjunto Fuzzy distObst.
dist1 = trapmf(distObst, [1.6 2 9.6 10]); % Far
dist2 = trapmf(distObst, [0.45 0.7 1.4 1.7]); % Close
dist3 = trapmf(distObst, [0 0.1 0.3 0.5]); % Touching
 
% orientacao
% Mesma explicação da de cima, mas para uma função de pertinência que representa o Conjunto Fuzzy orientacao (-pi/2 a pi/2).
orient1 = trimf(orientacao, [1.1 1 0.8]); % reto
orient2 = trimf(orientacao, [0.9 0.8 0.7]); % pouco_horario
orient3 = trimf(orientacao, [0.68 0.7 0.72]); % muito_horario
orient4 = trimf(orientacao, [-0.72 -0.7 -0.68]); % pouco_antihorario
orient5 = trimf(orientacao, [-0.7 -0.8 -0.9]); % muito_antihorario
 
% Saida: Velocidade Angular (-1 a 1)
omega = -1:0.05:1;
omg1 = trimf(omega, [-0.8 -0.6 -0.4]); % muito_esq
omg2 = trimf(omega, [-0.5 -0.3 -0.1]); % pouco_esq
omg3 = trimf(omega, [-0.15 0 0.15]); % zero
omg4 = trimf(omega, [0.1 0.3 0.5]); % pouco_dir
omg5 = trimf(omega, [0.4 0.6 0.8]); % muito_dir
 
% ========Aplicando as Regras========
% Distancia & orientacao -> Velocidade angular
% Marcha a ré
% R1: if(distObst is far & orientacao is reto) -> omega = pouco_esq;
% R2: if(distObst is far & orientacao is pouco_horario) -> omega = muito_esq;
% R3: if(distObst is far & orientacao is muito_horario) -> omega = muito_esq;
% R4: if(distObst is far & orientacao is pouco_antihorario) -> omega = zero;
% R5: if(distObst is far & orientacao is muito_antihorario) -> omega = zero;

% R6: if(distObst is close & orientacao is muito_horario) -> omega = pouco_esq;
% R7: if(distObst is close & orientacao is pouco_horario) -> omega = pouco_esq;
% R8: if(distObst is close & orientacao is reto) -> omega = pouco_dir;
% R9: if(distObst is close & orientacao is pouco_antihorario) -> omega = pouco_dir;
% R10: if(distObst is close & orientacao is muito_antihorario) -> omega = muito_dir;

% R11: if(distObst is touching) -> omega = zero;

R1 = fuzzificar2 (dist1, orient1, omg2);
R2 = fuzzificar2 (dist1, orient2, omg1);
R3 = fuzzificar2 (dist1, orient3, omg1);
R4 = fuzzificar2 (dist1, orient4, omg3);
R5 = fuzzificar2 (dist1, orient5, omg3);

R6 = fuzzificar2 (dist2, orient3, omg2);
R7 = fuzzificar2 (dist2, orient2, omg2);
R8 = fuzzificar2 (dist2, orient1, omg4);
R9 = fuzzificar2 (dist2, orient4, omg4);
R10 = fuzzificar2 (dist2, orient5, omg5);

R11 = fuzzificar1 (dist3, omg3);
 
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
 
vangular = defuzz(omega, output, 'centroid')
  
endfunction


% Sistema de Inferência
% OR - max
% AND - min
% THEN - min (implicação)

% Regras com 2 antecedentes e 1 consequente
function [Regra] = fuzzificar2 (dist, orientacao, omg)
	r1 = min(dist, orientacao);
	% Implicação da Regra
	for k=1:size(omg, 2)
	    Regra(k) = min(r1, omg(k));
	end
	clear k;
endfunction


% Regras com 1 antecedente e 1 consequente
function [Regra] = fuzzificar1 (dist, omg)
	r1 = dist;
	% Implicação da Regra
	for k=1:size(omg, 2)
	    Regra(k) = min(r1, omg(k));
	end
	clear k;
endfunction


% ====EOF====



%{

2. Forward approaching the curb
2.1 Far -> R2
2.1.1 if(distObst == far & orientacao < R2) -> steering_fw_approaching = L2;
2.1.2 if(distObst == far & orientacao > R2) -> steering_fw_approaching = R2;
2.1.3 if(distObst == far & orientacao == R2) -> steering_fw_approaching = straight;
2.2 Close -> R1
2.2.1 if(distObst == far & orientacao < R1) -> steering_fw_approaching = L1;
2.2.2 if(distObst == far & orientacao > R1) -> steering_fw_approaching = R1;
2.2.3 if(distObst == far & orientacao == R1) -> steering_fw_approaching = straight;
2.3 Touching -> Straight
2.3.1 if(distObst == touching) -> steering_fw_approaching = straight;

3. Forward moving away from the rear obstacle
3.1 Far -> Straight
3.1.1 if(distObst == far & orientacao > straight) -> steering_fw_mov_away = L3;
3.1.2 if(distObst == far & orientacao < straight) -> steering_fw_mov_away = R3;
3.1.3 if(distObst == far & orientacao == straight) -> steering_fw_mov_away = straight;

%}
