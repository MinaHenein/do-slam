function [dataNoisy] = addGaussianNoise(config,mu,sigma,dataGT,varargin)
%ADDNOISE Add gaussian noise to data
%   dataGT = m x n
%   mu   = mean, m x 1
%   sigma = standard deviation,m x 1
%   dataNoisy = m x n

% rng(config.rngSettings);

switch config.noiseModel
    case 'Gaussian'
        mu = repmat(mu,1,size(dataGT,2));
        sigma = repmat(sigma,1,size(dataGT,2));
        noise = normrnd(mu,sigma,size(dataGT));
        if nargin == 4
            dataNoisy = dataGT + noise;
        else
            switch varargin{1}
                case 'pose'
                    dataNoisy = zeros(size(dataGT));
                    for i = 1:size(dataGT,2)
                        dataNoisy(:,i) = config.relativeToAbsolutePoseHandle(dataGT(:,i),noise(:,i));
                    end
                otherwise; error('wrong type')
            end
        end
    case 'Off'
        dataNoisy = dataGT;
    otherwise
        error('config.noiseModel must be "Gaussian" or "Off"')



end

