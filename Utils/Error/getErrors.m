function [errorVec] = getErrors(results)
%GETERRORS Summary of this function goes here
%   gets desired errors

ATE = results.ATE_translation_error;
ASE = results.ASE_translation_error;
ARE = results.ATE_rotation_error;
RTE = results.RPE_translation_error;
RSE = results.RPTE_translation_error;
RRE = results.RPE_rotation_error;

errorVec = [ATE; ARE; ASE; RTE; RRE; RSE];

end

