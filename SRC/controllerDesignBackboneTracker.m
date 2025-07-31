%========================================================================
% DESCRIPTION: 
% Matlab function for theory-driven design of controller gains for phase 
% control (PLL) and response amplitude control of a structure forced by an 
% electrodynamic shaker.
% 
% The methodology is described in [1]. Equation numbers in the comments
% refer to equations in [1].
%
% The controller design is performed analytically based on a model of the 
% plant consisting of the structure under test (SUT) and the exciter under 
% a few simplifying assumptions. First, the model of the SUT is based on 
% single-non-inear-mode-theory. The response should therefore be dominated 
% by a single mode. Second, only a linearized model of the plant (in form 
% of linear modal properties) is known prior to backbone tracking and the 
% design point is therefore chosen in the linear regime. It is assumed that
% the design which is optimal in the linear regime is still sufficiently 
% good in terms of stability and settling time in the relevant nonlinear 
% regime. Third, the state-space model of the closed loop is linearized 
% around the fixed point at resonance which requires the states to remain 
% sufficiently close to that fixed point for the design to be valid. 
% Fourth, a phase-neutral exciter (i.e. zero phase shift between input 
% voltage and force) is assumed which simplifies the expressions and 
% thereby enables closed-form controller design. How these assumptions can 
% be checked and what measures are possible if they are violated is
% discussed in DOC/BackboneTracking.md.
% 
% The identification of the plant properties from linear modal tests is 
% described in Appendix A of [1].
%
% Two options exist for the controller design: 'optimal' provides a
% tradeoff between the maximum phase error during the transient and the 
% maximum frequency overshoot (see [1] for precise definition). 
% 'conservative' minimizes the frequency overshoot at the cost of larger
% transient phase errors and typically slower settling. The default and 
% recommended option is 'optimal'. The 'conservative' option can be useful
% if increased fluctuations of the phase lag error are observed in 
% conjunction with response amplitude control.
% 
% Besides the PLL controller design from [1], the function also implements
% a design approach for the gains of a PI-controller for the response
% amplitude. This novel approach is not published in the literature, yet.
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
%   design                      option to choose between        string
%                               optimal and conservative 
%                               design
%
% OUTPUT
%   VARIABLE                    MEANING                         TYPE
%   gainsPhaseController        controller gains of             struct
%                               PI-controller for phase control        
%       .kp                     proportional gain in 1/s        1x1 double
%       .ki                     integral gain in 1/s^2          1x1 double
%   gainsAmplitudeController    controller gains of             struct
%                               PI-controller for response 
%                               amplitude control        
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

    disp('**INFO on systematic controller design:')

    %% check user inputs
    % check plant properties
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

    if nargin < 5
        disp(['- No plant EMA provided. Controller parameters are computed' ...
            'neglecting the exciter influence.']);
        % no EMA of plant provided, assume ideal exciter
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

    if nargin < 9
        % use optimized design as defaut
        design = 'optimal';
        fprintf('- Controller design option is set to optimal. [DEFAULT]\n');
    else
        if strcmp(design,'optimal') || strcmp(design,'conservative')
            fprintf('- Controller design option is set to %s.\n',design);
        else
            error('Innvalid controller design option.');
        end
    end

    %% compute auxiliary parameters

    % structure decay rate (see Eq. 30)
    delta_s = linearModalDampingRatio * linearModalFrequency_Hz * 2*pi;

    % exciter/structure mass ratio (see Eq. 16)
    mu_ex = linearPhiEx^2 * exciterMass_kg;
    
    % plant decay rate (see Eq. 31 / Eq. A.5)
    delta_p = ( linearPlantDampingRatio*linearPlantFrequency_Hz *2*pi )*...
        (1 + mu_ex);
    
    % plant constant that is relevant for amplitude control
    chi = (1 + mu_ex) / (2*linearModalFrequency_Hz*2*pi*omLP) ...
        * linearPlantPhiEx^2;

    %% compute controller parameters
    
    % --------- optimal real part for amplitude and phase control ---------
    % dimensionless plant decay rate (see Eq. 32)
    delta_p_bar = delta_p/omLP; 
    
    % optimal real part of closed-loop eigenvalues (see Eq. 37)
    lambda_R = -(delta_p_bar + 1)/3;
    
    % --------- optimal imaginary part for phase control ------------------
    switch design
        case 'conservative'
            % the conservative design minimizes the frequency overshoot 
            % (cf. Sec. 3.3 / Fig. 3) 
            lambda_I_phase = 0;
        otherwise
            % The optimal design provides a tradeoff between maximum phase
            % error during the transient and maximum frequency overshoot. 
            % (cf. Sec. 3.3 / Fig. 3-4)

            % maximum phase error as function of imaginary part of 
            % the eigenvalue pair (see Eq. C.28)
            epsilon_max = @(I) 1/omLP * delta_p/delta_s * 2/(lambda_R^2+I.^2) ...
                .* exp(-2*(lambda_R./I).*atan(I/lambda_R));
            % limit value of the maximum phase error for zero imaginary 
            % part (see Eq. C.33)
            epsilon_max_0 = 1/omLP * delta_p/delta_s * 2/lambda_R^2 * exp(-2);
            
            % computation of maximum frequency overshoot
            % case separation depending on plant decay rate (see Eq. C.30)
            if delta_p_bar < 0.5 || delta_p_bar > 2
                n = 0;
            else
                n = 1;
            end
            % maximum frequency overshoot as function of imaginary part of 
            % the eigenvalue pair (see Eq. 31)
            eta = @(I) I.*(I.^2 + 3*lambda_R^2 - delta_p_bar) ./ ...
                (lambda_R*(2*lambda_R^2-delta_p_bar));

            Omega_bar_max = @(I) 1./(lambda_R^2 + I.^2) * ...
                exp(2*lambda_R./I*(n*pi-atan(eta(I)))) * ...
                ( (1+lambda_R^2./I.^2)*(2*lambda_R^2-delta_p_bar) + ...
                ((1+lambda_R^2./I.^2)*(-2*lambda_R^2 + delta_p_bar) - ...
                lambda_R^2 - I.^2)*(1-eta(I).^2)/(1+eta(I).^2) - ...
                lambda_R*(lambda_R^2./I + I)*2.*eta(I)./(1+eta(I).^2));
            
            % solve for optimal imaginary part of eigenvalue pair 
            % (see Eq. C.34 / Fig. 4)
            solopt = optimset('Display','none');
            [lambda_I_phase,~,exitFlag] = fsolve( ...
                @(I) (epsilon_max(I)/epsilon_max_0 - Omega_bar_max(I)), ...
                0.75,solopt); 
            if exitFlag <= 0
                error(['Optimization equation could not be solved. Check ' ...
                    'plant parameters.']);
            end
    end
    
    % --------- controller parameters for phase control -------------------
    % compute dimensionless controller gains (see Eq. 41-42)
    kp_bar_phase = delta_s/delta_p * ...
        (3*lambda_R^2 + lambda_I_phase^2 - delta_p_bar);
    ki_bar_phase = delta_s/delta_p * ...
        (-lambda_R*(lambda_R^2 + lambda_I_phase^2));
    
    % convert to dimensional controller gains (see Eq. 33-34)
    gainsPhaseController.kp = kp_bar_phase * omLP / (1 + mu_ex);
    gainsPhaseController.ki = ki_bar_phase * omLP^2 / (1 + mu_ex);
    
    % --------- controller parameters for amplitude control ---------------
    % optimal imaginary part for amplitude control is zero
    lambda_I_amp = 0;

    % compute dimensionless controller gains
    kp_bar_amp = 1/chi * (3*lambda_R^2 + lambda_I_amp^2 - delta_p_bar);
    ki_bar_amp = 1/chi *(-lambda_R*(lambda_R^2 + lambda_I_amp^2));
    
    % convert to dimensional controller gains
    gainsAmplitudeController.kp = kp_bar_amp;
    gainsAmplitudeController.ki = ki_bar_amp * omLP;
end

