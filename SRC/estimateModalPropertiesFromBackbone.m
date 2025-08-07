%========================================================================
% DESCRIPTION: 
% Matlab function estimating modal properties (frequency, damping ratio and
% (mass-normalized) mode shape. 
% To this end, the Fourier coefficient of the response must be given in 
% terms of displacement. Furthermore, the Fourier coefficients are turned 
% such that the fundamental harmonic component of a specific sensor is 
% purely real. Thus, the index of this sensor must be provided, e.g. the 
% drive point.
%
% Three versions are implemented: for force excitation (following [1] and  
% for base excitation in the model-free or the modal-based version [2].
%
% For force excitation, linear mass-normalized mode shapes are 
% required. Note that the number of linear modes included in the mode 
% shapes must not exceed the number of sensors (nSens >= nModes). The 
% sensor order in the linear mode shapes must be the same as in the 
% response Fourier coefficients. The index of the sensor to be purely real
% must be the drive point.
%
% In the model-free version for base excitation, a quadrature rule is 
% suggested to estimate the damping ration according to Eq. 28 in [2]. To
% this end, the weights must be provided. If linear mass-normalized mode
% shapes are further provided (optional), the nonlinear mode shape is 
% also mass-normalized as in the force excitation case.
%
% In the model-based version for base excitation, linearPhi^H*M*b must be 
% provided as well as linear mass-normalized mode shapes. Note that the 
% number of linear modes included in the mode  shapes must not exceed the 
% number of sensors (nSens >= nModes).
%
% 
% REFERENCES
% [1] P. Hippold, M. Scheel, L. Renson, M. Krack: Robust and fast backbone
%       tracking via phase-locked loops, MSSP 220 (2024), 
%       DOI: 10.1016/j.ymssp.2024.111670
% [2] F. Müller, L. Woiwode, J. Groß, M. Scheel, M. Krack: Nonlinear 
%       damping quantification from phase-resonant tests under base 
%       excitation, MSSP 177 (2022),  
%       DOI: 10.1016/j.ymssp.2022.109170
%
% INPUT
%   VARIABLE               MEANING                        TYPE
%   frequency_Hz           modal frequency in Hz          nHold x 1 double
%   Resp                   Fourier coeffcient of          (nHold x H+1 x 
%                           response in displacement        nSens) double
%                           (m), normal to surface         
%   Exc                    Fourier coefficient of         nHold x H+1 double
%                           excitation 
%   evalType               string indicating method       string      
%   indexReferenceSensor   index pointing to sensor       integer
%                           to be used for rotating
%                           Fourier coefficient to real 
%                           value
% for evalType = 'force'
%   varargin{1}            linear mass-normalized         nSens x nModes 
%                           mode shape (nSens >= nModes)           double 
% for evalType = 'modelFree'
%   varargin{1}            weights for quadrature rule    nSens x 1 double 
%   varargin{2}            optional: linear mass-         nSens x nModes 
%                           normalized mode shapes                   double
%                           (nSens >= nModes)
% for evalType = 'modelBased'
%   varargin{1}            linearPhi^H*M*b                nModes x 1 double 
%   varargin{2}            linear mass-normalized         nSens x nModes 
%                           mode shapes (nSens >= nModes)            double
%                           
% OUTPUT
%   VARIABLE                MEANING                     TYPE
%    modalFrequency_Hz      modal frequency in Hz       nHold x 1 double  
%    modalDampingRatio      modal damping ratio         nHold x 1 double 
%    Phi                    mass-normalized nonlinear   (nHold x H x 
%                            mode shape; only dynamic    nSens) double
%                            part
%    modalAmplitude         modal ampliutde in          nHold x 1 double
%                            sqrt(m^2 kg)
%    Resp                   Fourier coeffcients as in   (nHold x H+1 x 
%                            input, but rotated such that     nSens) double
%                            fundamental harmonic component
%                            of indexReferenceSensor is real
%    Exc                    Fourier coeffcients as in    nHold x H+1 double
%                            input, but rotated accordingly
%     
%========================================================================
% This file is part of NLtest.
% 
% COPYRIGHT AND LICENSING: 
% NLtest
% Copyright (C) 2025  
%       Malte Krack  (malte.krack@ila.uni-stuttgart.de) 
%       Maren Scheel (maren.scheel@ila.uni-stuttgart.de)
%       University of Stuttgart
% This program comes with ABSOLUTELY NO WARRANTY. 
% NLtest is free software, you can redistribute and/or modify it under the 
% GNU General Public License as published by the Free Software Foundation, 
% either version 3 of the License, or (at your option) any later version.
% For details on license and warranty, see http://www.gnu.org/licenses
% or gpl-3.0.txt.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [modalFrequency_Hz, modalDampingRatio,Phi,modalAmplitude,...
    Resp,Exc] = estimateModalPropertiesFromBackbone(...
    frequency_Hz,Resp,Exc,evalType,indexReferenceSensor,varargin)

    % check that frequency is given as column vectors
    if size(frequency_Hz,2)~=1
        error('Expecting frequency as column vector.')
    end
    % check that Exc and Resp have as many rows as recording phases
    if size(Exc,1)~=size(frequency_Hz,1)||...
            size(Resp,1)~=size(frequency_Hz,1)
        error(['Expecting the excitation and the response to have as '...
            'many rows as the frequency.'])
    end
    % check that Resp and Exc contain equal number of harmonics
    if size(Resp,2)~= size(Exc,2)
        error(['Expecting the response and the force to have same '...
            'number of columns (number of harmonics).'])
    end
    % check that index of reference sensor is a positive integer
    if numel(indexReferenceSensor)~=1||indexReferenceSensor<=0||...
            mod(indexReferenceSensor,1)~=0
        error('Expecting positive integer for index of reference sensor.')
    end

    % rotate dynamic complex Fourier coefficient such that one entry of 
    % response is real, namely the fundamental harmonic compontent of 
    % sensor indexReferenceSensor
    rotateAngle = angle(Resp(:,2,indexReferenceSensor));
    Resp(:,2:end,:) = Resp(:,2:end,:).*...
        exp(1i*repmat(-rotateAngle,1,size(Resp,2)-1,size(Resp,3)));
    Exc(:,2:end) = Exc(:,2:end).*...
        exp(1i*repmat(-rotateAngle,1,size(Resp,2)-1));

    switch evalType
        case 'force'
            fprintf(['Computing damping ratio for the case of force '...
                'excitation.\n'])
            fprintf(['Interpreting the fifth input as index for '...
                'drive point.\n'])
            indexSensorDrivePoint = indexReferenceSensor;
            % check consistency of drive point index
            if ~isscalar(indexSensorDrivePoint)||...
                    ~(mod(indexSensorDrivePoint,1)==0)||...
                    indexSensorDrivePoint>size(Resp,3)
                error(['Index of drive point sensor must be a scalar '...
                    'integer and in range of number of sensors.'])
            end
            % check dimension of mode shapes
            if length(varargin)>= 1
                linearPhi = varargin{1};
                if size(linearPhi,1) ~= size(Resp,3)
                    error(['Number of sensors in linear mode shapes and'...
                        ' backbone measurement must match.'])
                end
            else
                error(['Linear mass-normalized mode shapes must be '...
                    'provided as sixth input.'])
            end

            % compute modal amplitude
            modalAmplitude = transpose(vecnorm(...
                linearPhi\transpose(squeeze(Resp(:,2,:))),2,1)); 

            % mass-normalized mode shape, Eq. 47 in [1]
            Phi =  Resp./repmat(modalAmplitude,1,size(Resp,2),size(Resp,3));
            Phi = Phi(:,2:end,:); % restrict to dynamic part
            
            % compute modal damping ratio according to Eq. 46 in [1]
            modalDampingRatio = ...
                (real(Phi(:,1,indexSensorDrivePoint)).*abs(Exc(:,2))) ./...
                (2*(2*pi*frequency_Hz).^2 .* modalAmplitude);

        case 'modelFree'
            fprintf(['Computing damping ratio for the case of base '...
                'excitation (model-free version).\n'])
            fprintf(['Interpreting the sixth input as weights for '...
                'integration.\n'])
            weights = varargin{1};
            % check dimension of weights
            if size(weights,1)~=1 || size(weights,2)~=size(Resp,3)
                error(['Weights must be given as row vector with '...
                        'number of entries equals number of sensors.'])
            end
            % compute modal damping ratio according to Eq. 28 in [2]
            IQ = weights*transpose(squeeze(Resp(:,2,:)));
            IQQ = weights * transpose(abs(squeeze(Resp(:,2,:))).^2);
            modalDampingRatio = 1/2*abs(IQ)./abs(IQQ).*abs(Exc(:,2).'); 
            modalDampingRatio = transpose(modalDampingRatio);
            
            % optional: mass-normalize mode shapes
            if length(varargin)==2
                linearPhi = varargin{2};
                if size(linearPhi,1) ~= size(Resp,3)
                    error(['Number of sensors in linear mode shapes and '...
                            'backbone measurement must match.'])
                end
                modalAmplitude = transpose(vecnorm(...
                linearPhi\transpose(squeeze(Resp(:,2,:))),2,1)); 
                % mass-normalized mode shape, Eq. 47 in [1]
                Phi =  Resp./...
                    repmat(modalAmplitude,1,size(Resp,2),size(Resp,3));
                Phi = Phi(:,2:end,:); % restrict to dynamic part
            else
                Phi = [];
                modalAmplitude = [];
            end 

        case 'modelBased'
            fprintf(['Computing damping ratio for the case of base '...
                'excitation (model-based version).\n'])
            fprintf('Interpreting the sixth input as Phi_lin^H*M*b.\n')
            PhiMb_lin = varargin{1};
            % check dimension of mode shapes
            if length(varargin)~=2
                error(['Linear mass-normalized mode shapes are required'...
                    ' must be provided as seventh input.'])
            else
               linearPhi = varargin{2}; 
               if size(linearPhi,1) ~= size(Resp,3)
                   error(['Number of sensors in linear mode shapes and '...
                            'backbone measurement must match.'])
               end
            end
            % check dimension of linearPhi^H*M*b
            if size(PhiMb_lin,2)~=1 || size(PhiMb_lin,1)~=size(linearPhi,2)
                error(['linearPhi^H*M*b must be a column vector with number'...
                        ' of entries equals number of modes in linearPhi.'])
            end
            
            Eta = linearPhi\transpose(squeeze(Resp(:,2,:)));
            % Modal damping ratio (Eq. 14 in [2])
            modalDampingRatio = 1/2*abs((Eta'*PhiMb_lin).*Exc(:,2))./...
                ((2*pi*frequency_Hz).^2.*abs(diag(Eta'*Eta)));

            modalAmplitude = vecnorm(Eta,2,1); 
            % mass-normalized mode shape, Eq. 47 in [1]
            Phi =  Resp./repmat(modalAmplitude,1,size(Resp,2),size(Resp,3));
            Phi = Phi(:,2:end,:); % restrict to dynamic part
    end
    modalFrequency_Hz = frequency_Hz;
end