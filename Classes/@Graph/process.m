function [solver] = process(obj,config,measurementsCell,groundTruthCell,varargin)
%PROCESS processes measurements in batch or incremental and solves
%   processing is decided either by user input or config
%   allowing user to quickly change processing without altering config is
%   useful for running comparisons between batch and incremental

%processing chosen specifically, or from config
switch nargin
    case 4
        processing = config.processing;
    case 5
        processing = varargin{1};
    otherwise
        error('Require 4 or 5 inputs')
end

switch processing
    case 'batch'
        solver = obj.processBatch(config,measurementsCell,groundTruthCell);
    case 'incremental'
        solver = obj.processIncremental(config,measurementsCell,groundTruthCell);
end

end

