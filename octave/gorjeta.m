%% ===============Problema da Gorjeta====================
%
% Sistema Mamdani
% andMethod: 'min'
% orMethod: 'max'
% defuzzMethod: 'centroid'
% impMethod: 'min'
% aggMethod: 'max'
 
% ========REGRAS============
% IF antecedentes THEN consequente
% If service is poor or the food is rancid, then tip is cheap
% If service is good, then tip is average
% If service is excellent or food is delicious, then tip is generous
% ==========================
clear all

pkg load fuzzy-logic-toolkit
 
% Valores de entrada do sistema
% [service food]
input = [3 3];
 
% 3 regras com dois antecedentes e 1 consequente.
 
%% Fuzzificação
% Aqui esta etapa ocorre junto com a declaração das funções de pertinência
% Como
 
% SERVICE
% Nomenclatura das variáveis
% service degree rule N -&gt; dsrN
% Grau de Pertinência do Serviço Regra N
 
% Ao entrar com a variável na função gaussmf() eu já tenho a fuzzificação, ou seja, o grau de pertinência deste valor ao Conjunto Fuzzy SERVICE.
% Se eu tivesse dado o intervalo de x, neste caso de 0 a 10, à função gaussmf() eu teria uma curva representando a FP do Conjunto Fuzzy SERVICE.
sdr1 = gaussmf(input(1), [1.5 0]); % POOR SERVICE
sdr2 = gaussmf(input(1), [1.5 5]); % GOOD SERVICE
sdr3 = gaussmf(input(1), [1.5 10]); % EXCELLENT SERVICE
 
% FOOD
% Mesma explicação da de cima, mas para uma função de pertinência trapezoidal que representa o Conjunto Fuzzy FOOD.
fdr1 = trapmf(input(2), [0 0.1 1 3]); % RANCID FOOD
fdr2 = trapmf(input(2), [7 9 9.9 10 ]); % DELICIOUS FOOD
 
% TIP
% Porcentagem da Gorjeta (0%-30%)
x2 = 0:0.1:30;
% Aqui a situação é diferente. Como o consequente representa a minha saída no formato de um conjunto Fuzzy
% eu não quero entrar com um simples valor na função, eu entro com o domínio (0-30) da minha função de pertinência.
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
% Não há operação no antecedente, pois temos somente um
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
 
% ====EOF====
