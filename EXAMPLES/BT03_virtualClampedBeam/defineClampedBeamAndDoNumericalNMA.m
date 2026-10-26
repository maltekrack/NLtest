%========================================================================
% DESCRIPTION: 
% Matlab function setting up the plant model for the doubly-clamped beam
% example and performing a nonlinear modal analysis as reference.
%
% The structure under test is a doubly-clamped beam that exhibits a 
% pronounced geometric stiffening nonlinearity, inspired by the test rig 
% from [Mueller.2022]. The given example configuration is instrumented with 
% 5 response sensors, and is base-excited by a Brüel&Kjaer Type 4808 shaker
% via a rigid frame. The response sensor at midspan of the beam is used for 
% control.
% 
% The beam is modelled using analytical linear mode shapes (see comments 
% within file 'modalBeamModel.m'). The coupled plant consisting of 
% structure under test and exciter is cast into first-order state space 
% form. See 'ModelClampedBeamDescription.md' for further information.
%
% As numerical reference, nonlinear modal parameters are computed directly
% from the mathematical model of the structure under test in accordance
% with the Extended Periodic Motion Concept. To this end, Harmonic Balance 
% and numerical path continuation are used.
% 
% Model setup and numerical reference computation require the tool NLvib.
% Please download recent NLvib version from 
% https://github.com/maltekrack/NLvib/. Extract it in a (arbitrary sub-)
% folder within the SRC folder (subfolders of SRC will automatically be 
% added to the search path) or extract it anywhere else and extend the 
% search path accordingly.
% 
% INPUT
%   VARIABLE                MEANING                         TYPE
%     indexTargetMode         index of mode to be analyzed    double
%
%
% OUTPUT
%   VARIABLE                MEANING                         TYPE
%     virtualBeam             contains matrices and           struct
%                               parameters needed 
%                               for Simulink model 
%       FIELD NAMES
%         A, B, B_nl, ...     state-space matrices
%           C_va, C_d, ...
%         Cbase_va, Cbase_d,
%         T_nl,               localization matrices
%         p, E,               parameters of the nonlinearity
%         sensorMatrix, ...   indices corresponding to sensor    
%           sensorVectorBase,   outpts, respectively. 
%          
%
%     excitationQuantity      quantity measured by the        string
%                               excitation sensor (i.e. 
%                               'acceleration', 'velocity'  
%                               or 'displacement')
%     responseQuantity        quantity measured by each       5 x 1 cell
%                               virtual response sensor        array
%                               (i.e. 'acceleration', 
%                               'velocity' or 'displacement')
%     indexSensorControl      index of the drive point among  double
%                               the response sensors
%     sensorLocations         positions of the response 
%                               sensors along the beam in m.  5 x 1 double
%                               Required for damping 
%                               quantification in 
%                               post-processing.
%     linearSUTModes          linear modal properties         struct
%                               of the structure's first
%                               three modes
%       FIELD NAMES
%       frequency_Hz          modal frequency in Hz           1 x 5 double
%       modalDampingRatio     modal damping ratio             1 x 5 double
%       Phi                   mass-normalized deflection      5 x 5 double
%                               shape at five locations
%
%     linearPlantModes        linear modal properties         struct
%                               of the plant's first
%                               three modes
%       FIELD NAMES
%       frequency_Hz          modal frequency in Hz           1 x 5 double
%       modalDampingRatio     modal damping ratio             1 x 5 double
%       Phi                   mass-normalized deflection      5 x 5 double
%                               shape at five locations
%
%     m_ex                    the exciter's moving mass in    double
%                               kg
%
%     beamMass_kg             mass of beam's free lenghth     double
%       
%     nonlinearModeSUT        nonlinear modal properties      struct
%                               of the structure under test
%                               with the index specified by
%                               the user in accordance with
%                               EPMC (numerical reference);
%                               path continuation yields 'n'
%                               solution points in the 
%                               specified amplitude range
%       FIELD NAMES
%       a                     modal amplitude                 1 x n double
%       Resp1                 fundamental harmonic Fourier    5 x n double
%                               coefficient vector of the 
%                               displacement at the sensor 
%                               nodes
%       frequency_Hz          nonlinear modal frequency       1 x n double
%       modalDampingRatio     nonlinear modal damping ratio   1 x n double
% 
% REFERENCES
% [Mueller.2022] F. Müller, L. Woiwode, J. Gross, M. Scheel, and M. Krack:
%       Nonlinear damping quantification from phase-resonant tests under 
%       base excitation. Mechanical Systems and Signal Processing (2022).
%       DOI: 10.1016/j.ymssp.2022.109170
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

function [virtualBeam, excitationQuantity, responseQuantity, indexSensorControl, ...
    sensorLocations, ...
    linearSUTModes, linearPlantModes, m_ex, beamMass_kg,...
    nonlinearModeSUT] = ... 
    defineClampedBeamAndDoNumericalNMA(indexTargetMode)

    fprintf('Setting up the plant model...\n');
        
    %% Define structure under test
    nMod = 5; % number of linear modes to be computed

    % System parameters 
    L_mm = 140;         % free length (beam)
    h_mm = 0.8;         % thickness (beam)
    w_mm = 8;           % width (beam)
    rho_kg_m3 = 7683;   % density (beam)
    beamMass_kg = rho_kg_m3*(L_mm*h_mm*w_mm)/1000^3; % mass (beam, free length)
    E_Pa = 210e9;       % Young's modulus (beam)
    
    kt_N_m = 3.3e7;     % axial clamping stiffness
    kr_Nm_rad = 125;    % rotational clamping stiffness
    Dmod = ones(nMod,1)*2e-2;      % linear modal damping ratio (beam; set equal for all modes)
    
    %% Set up modal beam model
    % Auxiliary parameters of the cross section
    I_m4 = 1/12*w_mm*h_mm^3*1e-12;
    A_m2 = w_mm*h_mm*1e-6;
    
    % Call function that sets up the actual model, see function for definition
    % of output.
    [M,~,PHIfun,~,om,b,gam] = modalBeamModel(L_mm*1e-3,rho_kg_m3,E_Pa,...
        A_m2,I_m4,kr_Nm_rad,kt_N_m,nMod,Dmod);
    K = diag(om.^2);
    D = 2*diag(Dmod.*om);
    
    % convert polynomial stiffness coefficients from 4D-array to
    % coefficient-exponent representation
    [E,p] = coeffs_NLvib_format(b); 

    % Setup of beam model as mechanicalSystem object 
    beam = System_with_PolynomialStiffnessNonlinearity(M,D,K,p,E,gam);

    %% Specify instrumentation
    relativeSensorLocations = linspace(0.1,0.9,5); % sensor position / length
    responseQuantity = repmat({'velocity'},length(relativeSensorLocations),1);
    indexSensorControl = ceil(length(relativeSensorLocations)/2); % use sensor at midspan af the beam
    excitationQuantity = 'acceleration';
    sensorLocations = [0;relativeSensorLocations';1] * L_mm / 1000; % in m;...
    % add locations of clamp as they will be used in post-processing

    %% simulate result of EMA
    % mode shapes at sensor location
    Phi = PHIfun(relativeSensorLocations);
    
    % For this virtual test case, the modes must include the mode to be
    % tracked (mode 1) and maximum number of modes is nMod.
    indexLinearModes= 1:nMod;
    
    % store linear modal properties
    linearSUTModes.linearPhi = Phi(:,indexLinearModes);
    linearSUTModes.linearModalFrequency_Hz = transpose(om(indexLinearModes))/2/pi;
    linearSUTModes.linearModalDampingRatio = transpose(Dmod(indexLinearModes));

    %% Nonlinear modal analysis using harmonic balance
    analysis = 'NMA';

    % Analysis parameters
    imod = indexTargetMode;           % mode to be analyzed
    if numel(imod)~=1 || imod<=0|| mod(imod,1)~=0 || imod > length(om)
        error(['indexTargetMode must be a positive integer and not '...
            'exceed ' num2str(length(om)) ' for this set-up.'])
    end
    H = 7;              % harmonic order
    N = 2^8;           % number of time samples per period
    log10a_s = -6.2;      % start vibration level (log10 of modal mass)
    log10a_e = -4.3;    % end vibration level (log10 of modal mass)
    inorm = 1;   % coordinate for phase normalization

    % Initial guess vector x0 = [Psi;om;del], where del is the modal
    % damping ratio, estimate from underlying linear system 
    Psi = zeros((2*H+1)*beam.n,1);
    Psi(beam.n+imod) = 1;
    x0 = [Psi;om(imod);Dmod(imod)];
    psiscl = max(abs((Psi)));

    % Solve and continue w.r.t. Om
    ds = .01;
    Sopt = struct('flag',1,'stepadapt',1,'dynamicDscale',1,...
        'Dscale',[1e-0*psiscl*ones(size(Psi));...
        om(imod);1e-1;1e0]);
    X_NM = solve_and_continue(x0,...
        @(X) HB_residual(X,beam,H,N,analysis,inorm),...
        log10a_s,log10a_e,ds,Sopt);

    % Interpret solver output and store results in nonlinearModeSUT
    % variable
    Psi_NM = X_NM(1:end-3,:);
    nonlinearModeSUT.frequency_Hz = X_NM(end-2,:)/(2*pi);
    nonlinearModeSUT.modalDampingRatio = X_NM(end-1,:);
    log10a_NM = X_NM(end,:);
    nonlinearModeSUT.a = 10.^log10a_NM;
    Q_NM = Psi_NM.*repmat(nonlinearModeSUT.a,size(Psi_NM,1),1);
    % fundamental harmonic component at sensor locations
    nonlinearModeSUT.Resp1 = Phi*((Q_NM(beam.n+(1:beam.n),:) -...
        1i*Q_NM(2*beam.n+(1:beam.n),:)));

    %% define exciter model
    
    % data of shaker Brüel&Kjaer Type 4808
    k_ex  = 16320; % spring constant Table-Ground (N/m)
    d_ex  = 95.97; % damping constant Table-Ground (Ns/m)
    m_ex  = 0.252 + 1.0; % mass of armature + mounting frame (kg)
    R     = 0.8; % resistance (Ohm)
    G     = 8.84; % shaker ratio of thrust to coil current (N/A)
    
    % auxiliary exciter parameters
    % exciter natural frequency
    om_ex = sqrt(k_ex/m_ex); 
    % exciter damping ratio
    zeta_ex  = (d_ex + G^2/R) / (2*m_ex*om_ex); 
    
    % simulated plant EMA
    linearPlantModes.linearPlantFrequency_Hz = linearSUTModes.linearModalFrequency_Hz;
    linearPlantModes.linearPlantDampingRatio = linearSUTModes.linearModalDampingRatio;
    linearPlantModes.linearPlantPhi = sqrt(abs(Phi(:,indexLinearModes)).*gam.'*G/(R*m_ex));
    
    %% rigid coupling of exciter armature to drive point  
    % The system's equation of motion  and the state-space model of the 
    % plant are further explained in ModelClampedBeamDescription.md.
    M_c = [beam.M, gam; gam'/m_ex , 1+beamMass_kg/m_ex];
    D_c = [beam.D, zeros(beam.n,1); zeros(1,beam.n), 2*zeta_ex*om_ex];
    K_c = [beam.K, zeros(beam.n,1); zeros(1,beam.n), om_ex^2];
    
    %% set up state-space model of virtual plant
    % states: [generalized displacement; generalized velocity] (of the 
    % reduced order model)
    % input: voltage (to the exciter)
    % output: physical displacements, velocities, and accelerations at 
    % sensor locations and at base

    % linear state-space matrix
    virtualBeam.A = [zeros(beam.n+1), eye(beam.n+1); ...
        -M_c\K_c, -M_c\D_c];
 
    % input matrix for voltage
    virtualBeam.B = [zeros(beam.n+1,1); M_c\[zeros(beam.n,1); G/(R*m_ex)]]; 
    % input matrix for nonlinear force
    virtualBeam.B_nl = [zeros(beam.n+1,beam.n);...
        -eye(beam.n); ...
        zeros(1,beam.n)]; 

    % matrix to recover physical displacement at location of nonlinear 
    % element (here: distributed stiffness nonlinearity in modal coordinates)
    virtualBeam.T_nl = [eye(beam.n,beam.n), zeros(beam.n,beam.n+2)]; 
    
    % output matrix for (physical) acceleration and velocity
    virtualBeam.C_va = kron(eye(2),Phi) * [eye(beam.n,beam.n), zeros(beam.n,beam.n+2); ...
        zeros(beam.n,beam.n+1),eye(beam.n,beam.n), zeros(beam.n,1)];
    virtualBeam.Cbase_va = [zeros(1,beam.n),1,zeros(1,beam.n+1); ...
        zeros(1,2*beam.n+1),1];
    % output matrix for (physical) displacement
    virtualBeam.C_d = Phi*[eye(beam.n,beam.n), zeros(beam.n,beam.n+2)];
    virtualBeam.Cbase_d = [zeros(1,beam.n), 1, zeros(1,beam.n+1)];

    virtualBeam.sensorMatrix = zeros(length(relativeSensorLocations),3*length(relativeSensorLocations));
    for ii = 1:length(relativeSensorLocations)
        switch responseQuantity{ii}
            case 'displacement'
                virtualBeam.sensorMatrix(ii,ii) = 1;
            case 'velocity'
                virtualBeam.sensorMatrix(ii,length(relativeSensorLocations)+ii) = 1;
            case 'acceleration'
                virtualBeam.sensorMatrix(ii,2*length(relativeSensorLocations)+ii) = 1;
            otherwise
                error(['Invalid response quantity for sensor %d. Choose'...
                    ' displacement, velocity or acceleration'],ii);
        end
    end

    switch excitationQuantity
        case 'displacement'
            virtualBeam.sensorVectorBase = [1, 0, 0];
        case 'velocity'
            virtualBeam.sensorVectorBase = [0, 1, 0];
        case 'acceleration'
            virtualBeam.sensorVectorBase = [0, 0, 1];
        otherwise
            error(['Invalid excitation quantity for base excitation.' ...
                'Choose displacement, velocity or acceleration.']);
    end

    % properties of nonlinearity
    virtualBeam.E = E;
    virtualBeam.p = p;

    %% Display parameters related to design assumptions
    p1 = 2*zeta_ex*om_ex/om(indexTargetMode) / abs((om_ex/om(indexTargetMode))^2-1-beamMass_kg/m_ex);
    p2 = gam(indexTargetMode)^2/(2*Dmod(indexTargetMode)*(m_ex+beamMass_kg));
    fprintf('**INFO on plant\n')
    fprintf('- Phase neutrality parameter (should be << 1): %.3f\n',p1);
    fprintf('- SUT feedback parameter (should be << 1): %.3f\n',p2);

    fprintf('...done\n');
end

%% Nested function: coeffs_NLvib_format
function [E,p] = coeffs_NLvib_format(b)
% Converts polynomial stiffness nonlinearities from the format used in
% SimSAS to the format used in NLvib.

num_DOF = size(b,4); % number of degrees of freedom of the system
n_cub_coeffs = sum(cumsum(1:num_DOF)); % resulting number of cubic stiffness coefficients

% initialize matrices for coefficients and exponents
E = zeros(n_cub_coeffs, num_DOF); % coefficients
p = zeros(n_cub_coeffs, num_DOF); % exponents

% sort coefficients
for rr = 1:num_DOF % loop over lines of ODE (=DOFs)
    ctr = 1;
    for ii = 1:num_DOF
        for jj = ii:num_DOF
            for kk = jj:num_DOF
               % cubic terms
               E(ctr,rr) = b(ii,jj,kk,rr);
               ctr = ctr+1;
            end
        end
    end
end

% Create exponent matrix p
ctr = 1;
for ii = 1:num_DOF
    for jj = ii:num_DOF
        for kk = jj:num_DOF
           if (ii == jj) && (ii == kk)
               p(ctr,ii) = 3; % q_i^3
           elseif (ii == jj)
               p(ctr,ii) = 2; % q_i^2 * q_k
               p(ctr,kk) = 1;
           elseif (ii == kk)
               p(ctr,ii) = 2; % q_i^2 * q_j
               p(ctr,jj) = 1;
           elseif (jj == kk)
               p(ctr,jj) = 2; % q_j^2 * qi
               p(ctr,ii) = 1;
           else
               p(ctr,ii) = 1; % q_i * q_j * q_k
               p(ctr,jj) = 1;
               p(ctr,kk) = 1;
           end
           ctr = ctr+1;
        end
    end
end

end