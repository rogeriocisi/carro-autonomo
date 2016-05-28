%% ============Controle da velocidade linear e angular do carro=================
%
% Sistema Mamdani
% andMethod: 'min'
% orMethod: 'max'
% defuzzMethod: 'centroid'
% impMethod: 'min'
% aggMethod: 'max'

% ========REGRAS============
%{
Distance -> Desired_Orientation
Backward/Far -> alto_horario
Backward/Close -> baixo_antihorario
Touching -> Straight
Forward -> inverte sentidos horario/antihorario das regras acima

Strategy selector
· If the car is touching the curb and its orientation is straight, then stop; it is successfully parked.
· If there is enough back space, then go backward.
· If there is enough front space, then go forward.
%}
% ==========================

% clear all


function [Regra] = fuzzificar (dist, orientacao, omg)
	r1 = min(dist, orientacao);
	% Implicação da Regra
	for k=1:size(omg, 2)
	    Regra(k) = min(r1, omg(k));
	end
	clear k;
endfunction



function [vangular] = controle (obstFr, obstTr, obstCurb, direcao, orientacao)

% Faz com que as regras se apliquem independente da orientacao
if (direcao == -1)
	orientacao = -orientacao
 
% Regras com dois antecedentes e 1 consequente.
 
%% Fuzzificação
% Aqui esta etapa ocorre junto com a declaração das funções de pertinência
% Como
 
% Nomenclatura das variáveis
% service degree rule N -&gt; dsrN
% Grau de Pertinência do Serviço Regra N
 
% CurbDist
% Ao entrar com a variável na função gaussmf() eu já tenho a fuzzificação, ou seja, o grau de pertinência deste valor ao Conjunto Fuzzy CurbDist.
% Se eu tivesse dado o intervalo de x, neste caso de 0 a 1000 às funcoes trapmf, eu teria uma curva representando a FP do Conjunto Fuzzy CurbDist.
curbDist = min(obstFr, obstTr);
dist1 = trapmf(curbDist, [2 2.5 999 1000]); % Far
dist2 = trapmf(curbDist, [0.4 0.7 1.2 2.5]); % Close
dist3 = trapmf(curbDist, [0 0.1 0.3 0.5]); % Touching
 
% Orientacao
% Mesma explicação da de cima, mas para uma função de pertinência que representa o Conjunto Fuzzy orientacao (-pi/2 a pi/2).
orient1 = trapmf(orientacao, [-1 -0.9 -0.5 -0.4]); % alto_horario
orient2 = trapmf(orientacao, [-0.5 -0.4 -0.2 -0.05]); % baixo_horario
orient3 = trapmf(orientacao, [-0.1 -0.05 0.05 0.1]); % straight
orient4 = trapmf(orientacao, [0.05 0.2 0.4 0.5]); % baixo_antihorario
orient5 = trapmf(orientacao, [0.4 0.5 0.9 1]); % alto_antihorario
 
% Saida: Velocidade Angular (-1 a 1)
omega = -1:0.05:1;
omg1 = trimf(omega, [-1 -0.7 -0.4]); % alto_horario
omg3 = trimf(omega, [-0.5 -0.25 0]); % baixo_horario
omg4 = trimf(omega, [-0.1 0 0.1]); % straight
omg5 = trimf(omega, [0 0.25 0.5]); % baixo_antihorario
omg7 = trimf(omega, [0.4 0.7 1]); % alto_antihorario
 
%% =========Sistema de Inferência===========
% Eu determinei os operadores a serem usados lá em cima
% OR - max
% AND - min
% THEN - min (implicação)

% ========Aplicando as Regras========
% R1: if(CurbDist is far & orientacao is alto_horario) -> omega = straight;
% R2: if(CurbDist is far & orientacao is baixo_horario) -> omega = baixo_antihorario;
% R3: if(CurbDist is far & orientacao is baixo_antihorario) -> omega = baixo_horario;
% R4: if(CurbDist is far & orientacao is alto_antihorario) -> omega = alto_horario;
% R5: if(CurbDist is close & orientacao is baixo_horario or alto_horario) -> omega = baixo_antihorario;
% R6: if(CurbDist is close & orientacao is baixo_antihorario or alto_antihorario) -> omega = baixo_horario;
% R7: if(CurbDist is touching) -> steering_backward = straight;
R1 = fuzzificar (dist1, orient4, omg1);
R2 = fuzzificar (dist1, orient2, omg2);
R3 = fuzzificar (dist2, orient3, omg3);
R4 = fuzzificar (dist2, orient4, omg4);
R5 = fuzzificar (dist3, orient5, omg5);
 
% Agregação das regras
% Aqui as 5 regras são agregadas para formar um Conjunto Fuzzy de Saída do Sistema de Inferência.
% Neste caso usamos o operador MAX
output = max(R1,R2);
output = max(output,R3);
output = max(output,R4);
output = max(output,R5);
 
%% =======Defuzzificacao========
% Nesta etapa através de um método obtem-se um resultado escalar do conjunto fuzzy resultante do sistema de inferência.
% Aqui usamos o método da Centróide, mas existem outros.
 
vangular = defuzz(omega, output, 'centroid')
  
endfunction

% ====EOF====



%{

2. Forward approaching the curb
2.1 Far -> R2
2.1.1 if(CurbDist == far & orientacao < R2) -> steering_fw_approaching = L2;
2.1.2 if(CurbDist == far & orientacao > R2) -> steering_fw_approaching = R2;
2.1.3 if(CurbDist == far & orientacao == R2) -> steering_fw_approaching = straight;
2.2 Close -> R1
2.2.1 if(CurbDist == far & orientacao < R1) -> steering_fw_approaching = L1;
2.2.2 if(CurbDist == far & orientacao > R1) -> steering_fw_approaching = R1;
2.2.3 if(CurbDist == far & orientacao == R1) -> steering_fw_approaching = straight;
2.3 Touching -> Straight
2.3.1 if(CurbDist == touching) -> steering_fw_approaching = straight;

3. Forward moving away from the rear obstacle
3.1 Far -> Straight
3.1.1 if(CurbDist == far & orientacao > straight) -> steering_fw_mov_away = L3;
3.1.2 if(CurbDist == far & orientacao < straight) -> steering_fw_mov_away = R3;
3.1.3 if(CurbDist == far & orientacao == straight) -> steering_fw_mov_away = straight;

%}
