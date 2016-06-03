%% ============Fuzzyficar=================
%
% Sistema Mamdani
% andMethod: 'min'
% orMethod: 'max'
% defuzzMethod: 'centroid'
% impMethod: 'min'
% aggMethod: 'max'

% Sistema de Inferência
% OR - max
% AND - min
% THEN - min (implicação)

% Regras com 2 antecedentes e 1 consequente
function [Regra] = fuzzyficar2 (dist, orientacao, omg)
	r1 = min(dist, orientacao);
	% Implicação da Regra
	for k=1:size(omg, 2)
	    Regra(k) = min(r1, omg(k));
	end
	clear k;
endfunction
