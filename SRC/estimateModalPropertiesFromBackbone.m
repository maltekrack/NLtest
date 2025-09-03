%========================================================================
% DESCRIPTION: 
% This function identifies amplitude-dependent modal parameters of 
% well-separated modes from data acquired during backbone tracking. More
% specifically, this function outputs modal frequency 
% (modalFrequency_Hz), damping ratio (modalDampingRatio), mass-normalized 
% Fourier coefficients of the deflection shape (Phi), and the 
% corresponding modal amplitude (modalAmplitude).
% 
% For each point on the backbone curve, the complex Fourier coefficients 
% of response (Resp) and excitation (Exc) need to be provided, together 
% with the excitation frequency (frequency_Hz). It is presumed that the
% response is provided in terms of displacement, i.e., if velocity or
% acceleration is measured, the Fourier coefficients should be integrated
% in the frequency domain before providing them to this function.
% 
% The identification method, in particular, the damping quantification
% depends on the type of excitation. This is specified with the input 
% excType (['force'|'base']). 
% For base excitation, both the model-free and the model-based variant 
% from [2] are implemented, and to be specified as varargin{1}
% (['modelFree'|'modelBased']). In the model-free variant, numerical 
% quadrature weights are to be provided as varargin{2} to approximate the
% integrals in Eq. 28 in [2]. In the model-based variant, the vector 
% linearPhi^H*M*b in Eq. 18 in [2] is to be provided as varargin{2}.
% 
% Under some conditions, linear mass-normalized modal deflection shapes 
% (linearPhi) are needed. This is the case for force excitation and for 
% base excitation in the model-based variant (plus in the model-free 
% variant if the nonlinear mass-normalized deflection shape is to be 
% determined). Of course, the number of response sensors must be larger or 
% equal to the number of modes. If linearPhi is not needed, you may use []
% as argument.
% 
% In the case of force excitation, the response sensor at the drive point
% must be specified (indexReferenceSensor). The input Fourier coefficients
% are phase normalized in this function in such a way that the fundamental
% Fourier coefficient of the reference response sensor is real and
% positive. The output Fourier coefficients have the same name (Resp, Exc).
% In the case of base excitation, the reference sensor index is only used
% for this phase normalization.
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
%   frequency_Hz           excitation frequency in Hz     nBBP x 1 double
%   Resp                   Fourier coeffcients of         nBBP x H+1 x 
%                           response                       nSens double
%   Exc                    Fourier coefficients of        nBBP x H+1 double
%                           excitation 
%   indexReferenceSensor   index of reference             integer
%                           response sensor 
%   linearPhi              linear mass-normalized         nSens x nModes 
%                           deflection shapes              double 
%   excType                excitation type specifier      string
% if excType = 'base'
%   varargin{1}            variant specifier              string
%   if varargin{1} = 'modelBased'
%       varargin{2}        PhiMb_lin                      nModes x 1 double
%   if varargin{1} = 'modelFree'
%       varargin{2}        weights                        nSens x 1 double
%                           
% OUTPUT
%   VARIABLE                MEANING                     TYPE
%    modalFrequency_Hz      modal frequency in Hz       nBBP x 1 double  
%    modalDampingRatio      modal damping ratio         nBBP x 1 double 
%    Phi                    mass-normalized nonlinear   (nBBP x H+1 x 
%                            mode shape                  nSens) double
%    modalAmplitude         modal amplitude in          nBBP x 1 double
%                            m*sqrt(kg)
%    Resp, Exc              phase normalized input
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
function [modalFrequency_Hz,modalDampingRatio,Phi,modalAmplitude,...
    Resp,Exc] = estimateModalPropertiesFromBackbone(...
    frequency_Hz,Resp,Exc,indexReferenceSensor,linearPhi,excType,varargin)
    %% Handle user input

    % Check if frequency is given as column vector
    if size(frequency_Hz,2)~=1
        error('Expecting frequency as column vector.');
    end
    
    % The number of rows in frequency_Hz is interpreted as number of
    % backbone points
    nBBP = size(frequency_Hz,1);
    fprintf(['Estimating modal parameters for ' ...
        num2str(nBBP) ' points on the backbone curve.\n']);

    % The size of Resp in the third dimension is interpreted as number of
    % sensors
    nSens = size(Resp,3);

    % Check if Exc and Resp have nBBP rows
    if size(Exc,1)~=nBBP || size(Resp,1)~=nBBP
        error(['Excitation and response must have as many rows as '...
            'backbone points.']);
    end

    % Check if Exc has at least two columns (the second column corresponds
    % to the fundamental harmonic)
    if size(Exc,2)<2
        error(['Excitation input must have at least 2 columns. ' ...
            'The second column corresponds to the required ' ...
            'fundamental harmonic Fourier coefficient.']);
    end

    % Check if index of reference sensor is a positive integer in
    % admissible range
    if ~isscalar(indexReferenceSensor)||indexReferenceSensor<=0|| ...
            ~(mod(indexReferenceSensor,1)==0)||...
            indexReferenceSensor>nSens
        error(['indexReferenceSensor must be valid index for ' ...
            '3rd dimension of Resp.']);
    end

    % Check linearPhi, if provided
    if nargin>=5 && ~isempty(linearPhi)
        % Interpret second dimension as number of modes
        nModes = size(linearPhi,2);

        % Check dimensions
        if size(linearPhi,1)~=nSens
            error(['1st dimension of linearPhi must equal number of ' ...
                'response sensors (' num2str(nSens) ' here).']);
        elseif nSens<nModes
            error(['Number of response sensors must exceed ' ...
                'number of provided linear modes']);
        end
    else
        % If linearPhi is not provided, define variables accordingly
        nModes = 0;
        linearPhi = zeros(nSens,0);
    end

    % Handle unspecified excType
    if nargin<6
        fprintf(['No excitation type specified, assuming force ' ...
            'excitation.\n']);
        excType = 'force';
    end

    % Handle nModes==0 
    if nModes==0
        if stcmp(excType,'force')
            error(['In the case of force excitation, mass-normalized ' ...
                'linear modal deflection shape(s) must be ' ...
                'provided (linearPhi).']);
        elseif strcmp(excType,'base') && strcmp(varargin{1},'modelBased')
            error(['In the case of base excitation, mass-normalized ' ...
                'linear modal deflection shape(s) must be ' ...
                'provided (linearPhi) for the model-based variant ' ...
                'of modal parameter identification.']);
        end
    end

    %% Shift the phase of Exc and Resp consistently to have fundamental 
    % Fourier coefficient of reference response real and >0.

    % Determine angle of corresponding complex Fourier coefficient
    rotateAngle = angle(Resp(:,2,indexReferenceSensor));

    % Rotate complex Fourier coefficients Resp
    H = size(Resp,2)-1;
    Resp = Resp .* exp(-1i*repmat(rotateAngle*(0:H),...
        1,1,size(Resp,3)));

    % Rotate complex Fourier coefficients Exc
    H = size(Exc,2)-1;
    Exc = Exc .* exp(-1i*rotateAngle*(0:H));

    %% Identify modal frequency

    % In accordance with Single-Nonlinear-Mode Theory, the excitation 
    % frequency at phase resonance equals the modal frequency. Here we 
    % also presume sufficient control quality that the provided data
    % corresponds indeed to the phase resonant backbone.
    modalFrequency_Hz = frequency_Hz;

    %% Identify modal damping ratio, mass-normalized deflection shapes and 
    % corresponding modal amplitude depending on type of excitation
    switch excType
        case 'force'
            % Interpret indexReferenceSensor as index of drive point
            % response sensor
            indexSensorDrivePoint = indexReferenceSensor;
            fprintf(['Treating index ' num2str(indexSensorDrivePoint) ...
                ' in 3rd dimension of Resp as drive point response.\n']);

            % Modal amplitude and mass-normalized deflection shape
            [modalAmplitude,Phi] = modalAmplitudeAndPhi(Resp,linearPhi);

            % Modal damping ratio (Eq. 46 in [1])
            modalDampingRatio = ...
                (real(Phi(:,2,indexSensorDrivePoint)).*abs(Exc(:,2))) ./...
                (2*(2*pi*frequency_Hz).^2 .* modalAmplitude);

        case 'base'
            % Interpret Exc as base displacement (as opposed to
            % acceleration)
            fprintf(['Treating provided excitation as base ' ...
                'displacement Fourier coefficients.\n']);

            % Interpret varargin{1} as model-based vs. model-free variant
            % specifier
            variant = varargin{1};
            switch variant
                case 'modelFree'
                    % Interpret varargin{2} as weights
                    weights = varargin{2}(:).';

                    % Check dimension of weights
                    if size(weights,2)~=nSens
                        error(['Number of elements in weights must equal '...
                            'number of response sensors.']);
                    end

                    % Modal amplitude and mass-normalized deflection shape
                    [modalAmplitude,Phi] = ...
                        modalAmplitudeAndPhi(Resp,linearPhi);

                    % Modal damping ratio (Eq. 28 in [2], where integrals
                    % are approximated by numerical quadrature using
                    % provided weights, and presuming that the response is
                    % restricted to the direction of the base motion)
                    IQ = weights*transpose(squeeze(Resp(:,2,:)));
                    IQQ = weights * transpose(abs(squeeze(Resp(:,2,:))).^2);
                    modalDampingRatio = 1/2*abs(conj(IQ(:)).*Exc(:,2))./...
                        abs(IQQ(:));

                case 'modelBased'
                    % Interpret varargin{2} as quantity linearPhi'*M*b
                    PhiMb_lin = varargin{2};

                    % Check dimensions of PhiMb_lin
                    if size(PhiMb_lin,2)~=1 || size(PhiMb_lin,1)~=nModes
                        error(['PhiMb_lin must be a column vector with'...
                            ' number of rows equal to number of ' ...
                            'provided linear modes (' num2str(nModes) ...
                            'here).']);
                    end

                    % Modal amplitude, mass-normalized deflection shape,
                    % and contributions of linear modes to fundamental
                    % harmonic
                    [modalAmplitude,Phi,Eta1] = ...
                        modalAmplitudeAndPhi(Resp,linearPhi);

                    % Modal damping ratio (Eq. 18 in [2])
                    modalDampingRatio = abs((Eta1'*PhiMb_lin).*Exc(:,2))./...
                        (2*modalAmplitude.^2);

                otherwise
                    error(['Invalid specifier ' variant ' for ' ...
                        'variant of modal parameter estimation ' ...
                        'in case of base excitation']);
            end
        otherwise
            error(['Allowed excitation type specifiers are ' ...
                '[force|base]']);
    end
end

%% NESTED FUNCTION: modal amplitude, mass-normalized deflection shape, 
% contributions of linear modes to fundamental harmonic
function [modalAmplitude,Phi,Eta1] = modalAmplitudeAndPhi(Resp,linearPhi)

% Handle empty linearPhi
if isempty(linearPhi)
    modalAmplitude = [];
    Phi = [];
    return;
end

% Fundamental Fourier coefficient vector of linear modal 
% coordinates (Eq. 20 in [2])
Eta1 = linearPhi\transpose(squeeze(Resp(:,2,:)));

% Modal amplitude (from Eq. 9 and 17 in [2])
modalAmplitude = vecnorm(Eta1,2,1);
modalAmplitude = modalAmplitude(:);

% % THIS YIELDS THE SAME AMPLITUDE
% % Modal amplitude (Eq. 47 in [1])
% modalAmplitude = transpose(vecnorm(...
%     linearPhi\transpose(squeeze(Resp(:,2,:))),2,1));

% Mass-normalized modal deflection shape (Eq. 48 in [1])
Phi =  Resp ./ ...
    repmat(modalAmplitude,1,size(Resp,2),size(Resp,3));

end