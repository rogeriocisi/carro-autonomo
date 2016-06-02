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

% Regras com 1 antecedente e 1 consequente
function [Regra] = fuzzyficar1 (dist, omg)
	r1 = dist;
	% Implicação da Regra
	for k=1:size(omg, 2)
	    Regra(k) = min(r1, omg(k));
	end
	clear k;
endfunction
