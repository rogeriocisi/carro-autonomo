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

1. Backward strategy
1.1 Far -> L3
1.1.1 if(CurbDist == far & angle < L3) -> steering_backward = R3;
1.1.2 if(CurbDist == far & angle > L3) -> steering_backward = L3;
1.1.3 if(CurbDist == far & angle == L3) -> steering_backward = straight;
1.2 Close -> L1
1.3 Touching -> Straight

2. Forward approaching the curb
2.1 Far -> R2
2.2 Close -> R2
2.3 Touching -> Straight

3. Forward moving away from the rear obstacle
3.1 Far -> Straight
3.1.1 if(CurbDist == far & angle > straight) -> steering_fw_mov_away = L3;
3.1.2 if(CurbDist== far & angle==straight) -> steering_fw_mov_away = straight;
3.1.3 if(CurbDist == far & angle < straight) -> steering_fw_mov_away = R3;

4. Strategy selector
This rule base, or inference engine, decides which one of the three strategies must be applied. The
reasoning is as follows,
· If the car is touching the curb and its orientation is straight, then stop; it is successfully parked.
· If there is enough back space, then go backward.
· If the back obstacle is really near and the car is close to (or near) the curb, then go forward
towards the curb.
· If the back obstacle is really near and the car is far from the curb, then go forward moving away
from the rear obstacle.
%}
% ==========================

% clear all
 
function [vlinear, vangular] = controle (service, food)
 
 
% Valores de entrada do sistema
% [CurbDist Angle]
input = [3 0.5];
 
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
sdr1 = trapmf(input(1), [0 0.1 0.3 0.5]); % Touching
sdr2 = trapmf(input(1), [0.4 0.7 1.2 2.5]); % Close
sdr3 = trapmf(input(1), [2 2.5 999 1000]); % Far
 
% Angle
% Mesma explicação da de cima, mas para uma função de pertinência que representa o Conjunto Fuzzy Angle (-pi/2 a pi/2).
fdr1 = trapmf(input(2), [-1.57 -1.4 -1 -0.9]); % L3
fdr2 = trapmf(input(2), [-1 -0.9 -0.5 -0.4]); % L2
fdr3 = trapmf(input(2), [-0.5 -0.4 -0.2 -0.05]); % L1
fdr4 = trapmf(input(2), [-0.1 -0.05 0.05 0.1]); % Straight
fdr5 = trapmf(input(2), [0.05 0.2 0.4 0.5]); % R1
fdr6 = trapmf(input(2), [0.4 0.5 0.9 1]); % R2
fdr7 = trapmf(input(2), [0.9 1 1.4 1.57]); % R3
 
% Saida: Velocidade Angular (-1 a 1)
x2 = -1:0.0.5:1;
% Aqui a situação é diferente. Como o consequente representa a minha saída no formato de um conjunto Fuzzy
% eu não quero entrar com um simples valor na função, eu entro com o domínio (-1 a 1) da minha função de pertinência.
% Mais embaixo eu explico o porquê.
tdr1 = trimf(x2, [0 5 10]); % CHEAP TIP
tdr2 = trimf(x2, [10 15 20]); % AVERAGE TIP
tdr3 = trimf(x2, [20 25 30]); % GENEROUS TIP
 
%% =========Sistema de Inferência===========
% Eu determinei os operadores a serem usados lá em cima
% OR - max
% AND - min
% THEN - min (implicação)
% ========Regra 1========
% If service is poor or the food is rancid, then tip is cheap
% Produto cartesiano do consequente, como é OR, uso o MAX
% Quero o Máximo entre o grau de pertinência de SERVICE e FOOD
r1 = max(sdr1,fdr1);
% Implicação da Regra 1
% Lembra que no consequente eu uso o domínio todo da função de pertinência TIP
% e não um valor como no caso da fuzzificação de SERVICE e FOOD?
% Isto ocorre porque na implicação eu vou ver como os antecedentes modificam o consequente.
% Como estabelecido o operador de implicação é o MIN.
% Este FOR está aqui pois eu tenho que estabelecer o mínimo entre cada ponto da função de pertinência do consequente TIP e o resultado dos antecedentes.
for k=1:size(tdr1,2)
    R1(k)=min(r1,tdr1(k));
end
clear k;
 
% =======Regra 2=========
% If service is good, then tip is average
% Não há operação no antecedente, pois temos somenet um
r2=sdr2;
 
% Implicação
% Segue a mesma lógica da Regra 1
for k=1:size(tdr2,2)
    R2(k)=min(r2,tdr2(k));
end
clear k;
 
% ======Regra 3=======
% If service is excellent or food is delicious, then tip is generous
r3=max(sdr3,fdr2);
% Implicacao
for k=1:size(tdr3,2)
    R3(k)=min(r3,tdr3(k));
end
 
% Agregação das regras
% Aqui as 3 regras são agregadas para formar um Conjunto Fuzzy de Saída do Sistema de Inferência.
% Neste caso usamos o operador MAX
% O MATLAB só faz o máximo entre dois objetos, então determina-se o máximo entre a Regra 2 e Regra 3, depois entre o resultado destas e a Regra 1.
% Mas pode ser em outra ordem.
output = max(R1,max(R2,R3));
 
%% =======Defuzzificacao========
% Nesta etapa através de um método obtem-se um resultado escalar do conjunto fuzzy resultante do sistema de inferência.
% Aqui usamos o método da Centróide, mas existem outros.
 
tip = defuzz(x2,output,'centroid')
  
endfunction

% ====EOF====