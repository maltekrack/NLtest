%========================================================================
% DESCRIPTION: 
% This function implements the theory-driven design of the proportional and
% integral gains for phase control (PLL) and response amplitude control of 
% a structure driven by an electrodynamic shaker.
% 
% The input comprises the cutoff frequency of the adaptive filter,
% linear modal parameters of the structure under test (SUT) and the plant
% with regard to the target mode, and the exciter moving mass, see 
% DOC/BackboneTracking.md. If no plant properties are provided, an ideal 
% exciter is assumed (zero moving mass; plant modal parameters identical 
% to that of SUT).
% 
% The methodology is described in [1], and equation numbers in the comments
% correspond to that reference. The underlying assumptions are stated in
% DOC/BackboneTracking.md under 'What to do if the controller does not
% work?', together with instructions how to check their validity, and means
% to counter them. 
% 
% The design of the response amplitude control is not published yet.
% The 'conservative' option of the phase control design can be useful
% if increased fluctuations of the phase lag error are observed in 
% conjunction with response amplitude control. Compared to the 'optimal'
% (default) setting, this is expected to come at the cost of a longer 
% phase settling duration.
% 
% MANDATORY INPUT
%   VARIABLE                    MEANING                         TYPE
%   omLP                        equivalent cutoff frequency of  1x1 double
%                               adaptive filter in rad/s
%   linearModalFrequency_Hz     linear modal frequency of       1x1 double
%                               SUT in Hz
%   linearModalDampingRatio     linear modal damping ratio      1x1 double
%                               of SUT
%   linearPhiEx                 linear mass-normalized mode     1x1 double
%                               shape of SUT at drive point
% OPTIONAL INPUT
%   linearPlantFrequency_Hz     linear modal frequency of       1x1 double
%                               plant in Hz
%   linearPlantDampingRatio     linear modal damping ratio      1x1 double
%                               of plant
%   linearPlantPhiEx            linear mass-normalized mode     1x1 double
%                               shape of plant at drive point
%   exciterMass_kg              mass of moving parts of exciter 1x1 double
%                               in kg
%   design                      option for phase control        string
%                               ['optimal'|'conservative']
%
% OUTPUT
%   VARIABLE                    MEANING                         TYPE
%   gainsPhaseController        gains of phase controller       struct
%       .kp                     proportional gain in 1/s        1x1 double
%       .ki                     integral gain in 1/s^2          1x1 double
%   gainsAmplitudeController    gains of amplitude controller   struct     
%       .kp                     proportional gain in V/m        1x1 double
%       .ki                     integral gain in V/(m s)        1x1 double
%
% REFERENCES
% [1] P. Hippold, M. Scheel, L. Renson, M. Krack: Robust and fast backbone
%       tracking via phase-locked loops, MSSP 220 (2024), 
%       DOI: 10.1016/j.ymssp.2024.111670
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

function [gainsPhaseController, gainsAmplitudeController] = ...
    controllerDesignBackboneTracker(omLP, linearModalFrequency_Hz, ...
    linearModalDampingRatio, linearPhiEx, linearPlantFrequency_Hz, ...
    linearPlantDampingRatio, linearPlantPhiEx, exciterMass_kg, design)

    disp('**INFO on theory-driven control design:')

    %% Handle user input

    % Check mandatory input (linear modal properties)
    if numel(linearModalFrequency_Hz)~=1 || linearModalFrequency_Hz<=0
        error('Expecting positive scalar for linear modal frequency.');
    end
    if numel(linearModalDampingRatio)~=1 || linearModalDampingRatio<=0
        error('Expecting positive scalar for linear modal damping ratio.');
    end
    if numel(linearPhiEx)~=1 || ~isreal(linearPhiEx)
        error(['Expecting real scalar for linear mode shape of target mode ' ...
            'at drive point.']);
    end

    % Check optional input (plant properties)
    if nargin < 5
        disp('- No plant properties provided. Assuming ideal exciter.');
        linearPlantFrequency_Hz = linearModalFrequency_Hz;
        linearPlantDampingRatio = linearModalDampingRatio;
        linearPlantPhiEx        = linearPhiEx;
        exciterMass_kg          = 0;
    else
        if numel(linearPlantFrequency_Hz)~=1 || linearPlantFrequency_Hz<=0
            error('Expecting positive scalar for plant linear modal frequency.');
        end
        if numel(linearPlantDampingRatio)~=1 || linearPlantDampingRatio<=0
            error('Expecting positive scalar for plant linear modal damping ratio.');
        end
        if numel(linearPlantPhiEx)~=1 || ~isreal(linearPlantPhiEx)
            error(['Expecting real scalar for linear plant mode shape of ' ...
                'target mode at drive point.'])
        end
    end

    % Check optional input (phase control design specifier)
    if nargin < 9
        % Use option 'optimal' as default.
        design = 'optimal';
        fprintf('- Controller design option is set to optimal. [DEFAULT]\n');
    else
        if strcmp(design,'optimal') || strcmp(design,'conservative')
            fprintf('- Controller design option is set to %s.\n',design);
        else
            error('Innvalid controller design option.');
        end
    end

    %% Auxiliary parameters

    % SUT decay rate (Eq. 30)
    delta_s = linearModalDampingRatio * linearModalFrequency_Hz * 2*pi;

    % Exciter/structure mass ratio (Eq. 16)
    mu_ex = linearPhiEx^2 * exciterMass_kg;
    
    % Plant decay rate (Eq. 31 / Eq. A.5)
    delta_p = ( linearPlantDampingRatio*linearPlantFrequency_Hz *2*pi )*...
        (1 + mu_ex);

    % Dimensionless plant decay rate (Eq. 32)
    delta_p_bar = delta_p/omLP; 
    
    % Plant constant relevant for amplitude control
    chi = (1 + mu_ex) / (2*linearModalFrequency_Hz*2*pi*omLP) ...
        * linearPlantPhiEx^2;

    %% Controller gains
    
    % --------- optimal real part for amplitude and phase control ---------
    
    lambda_R = -(delta_p_bar + 1)/3; % Eq. 37
    
    % --------- optimal imaginary part for phase control ------------------
    switch design
        case 'conservative'
            % The conservative design minimizes the frequency overshoot 
            % (Sec. 3.3 / Fig. 3 in [1]).
            lambda_I_phase = 0;
        otherwise
            % The optimal design provides a tradeoff between maximum phase
            % error during the transient and maximum frequency overshoot
            % (cf. Sec. 3.3 / Fig. 3-4 in [1]).

            % Define maximum phase error as function of imaginary part of 
            % the eigenvalue pair (Eq. C.28).
            epsilon_max = @(I) 1/omLP * delta_p/delta_s * 2/(lambda_R^2+I.^2) ...
                .* exp(-2*(lambda_R./I).*atan(I/lambda_R));

            % Calculate limit value of the maximum phase error for zero 
            % imaginary part (Eq. C.33)
            epsilon_max_0 = 1/omLP * delta_p/delta_s * ...
                2/lambda_R^2 * exp(-2);
            
            % Case distinction depending on plant decay rate (Eq. C.30):
            if delta_p_bar < 0.5 || delta_p_bar > 2
                n = 0;
            else
                n = 1;
            end

            % Define auxiliary variable 'eta' and maximum frequency 
            % overshoot as function of imaginary part of the eigenvalue 
            % pair (Eq. 31f)
            eta = @(I) I.*(I.^2 + 3*lambda_R^2 - delta_p_bar) ./ ...
                (lambda_R*(2*lambda_R^2-delta_p_bar));
            Omega_bar_max = @(I) 1./(lambda_R^2 + I.^2) * ...
                exp(2*lambda_R./I*(n*pi-atan(eta(I)))) * ...
                ( (1+lambda_R^2./I.^2)*(2*lambda_R^2-delta_p_bar) + ...
                ((1+lambda_R^2./I.^2)*(-2*lambda_R^2 + delta_p_bar) - ...
                lambda_R^2 - I.^2)*(1-eta(I).^2)/(1+eta(I).^2) - ...
                lambda_R*(lambda_R^2./I + I)*2.*eta(I)./(1+eta(I).^2));
            
            % Solve for optimal imaginary part of eigenvalue pair 
            % (Eq. C.34 / Fig. 4 in [1])
            solopt = optimset('Display','none');
            [lambda_I_phase,~,exitFlag] = fsolve( ...
                @(I) (epsilon_max(I)/epsilon_max_0 - Omega_bar_max(I)), ...
                0.75,solopt);
            if exitFlag <= 0
                error(['Optimization equation could not be solved. Check ' ...
                    'plant parameters.']);
            end
    end
    
    % --------- controller gains for phase control ------------------------
    
    % Dimensionless controller gains (Eq. 41-42)
    kp_bar_phase = delta_s/delta_p * ...
        (3*lambda_R^2 + lambda_I_phase^2 - delta_p_bar);
    ki_bar_phase = delta_s/delta_p * ...
        (-lambda_R*(lambda_R^2 + lambda_I_phase^2));
    
    % Dimensional controller gains (Eq. 33-34)
    gainsPhaseController.kp = kp_bar_phase * omLP / (1 + mu_ex);
    gainsPhaseController.ki = ki_bar_phase * omLP^2 / (1 + mu_ex);
    
    % --------- controller gains for amplitude control --------------------
    
    % The optimal imaginary part for amplitude control is zero
    lambda_I_amp = 0;

    % Dimensionless controller gains
    kp_bar_amp = 1/chi * (3*lambda_R^2 + lambda_I_amp^2 - delta_p_bar);
    ki_bar_amp = 1/chi *(-lambda_R*(lambda_R^2 + lambda_I_amp^2));
    
    % Dimensional controller gains
    gainsAmplitudeController.kp = kp_bar_amp;
    gainsAmplitudeController.ki = ki_bar_amp * omLP;
end