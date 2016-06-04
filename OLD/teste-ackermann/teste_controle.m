%% ============Controle da velocidade linear e angular do carro=================

% clear all
 
function [vlinear, vangular] = controle (obstFr, obstTr, obstEs, obstDi, oriZ)
 
vlinear = max(obstFr,max(obstTr,max(obstEs,obstDi)));
vangular = oriZ;
  
endfunction

% ====EOF====