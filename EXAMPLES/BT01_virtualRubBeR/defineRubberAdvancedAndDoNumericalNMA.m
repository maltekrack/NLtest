%========================================================================
% DESCRIPTION: 
% Matlab function setting up the plant for the advanced virtual RubBeR
% example and performing a nonlinear modal analysis as reference.
%
% The structure under test is a cantilevered beam with an elastic dry 
% friction element, inspired by the RubBeR test rig [Scheel.2020]. The 
% given example configuration is instrumented with 7 response sensors, and 
% is driven by a Brüel&Kjaer Type 4809 shaker applied via stinger at the 
% beam's tip, which is collocated with 7th sensor location (cf. Fig. 5 in 
% [Scheel.2020]). 
% 
% The beam is modelled using finite elements and Euler-Bernoulli beam 
% theory. A reduced-order model of the beam is derived using the common 
% Hurty-/Craig-Bampton method. Dry friction is modelled by a single Jenkins 
% Element attached at ~ 1/3 of the length measured from the clamping. A 
% simple electro-mechanical model is used for the exciter, see e.g. 
% [Hippold.2024]. The coupled plant consisting of structure under test and 
% exciter is cast into first-order state space form. See 
% DOC/BackboneTracking.md for further information.
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
%     indexTargetMode       index of mode to be analyzed    double
%
%
% OUTPUT
%   VARIABLE                MEANING                         TYPE
%     virtualRubber         contains matrices and           struct
%                            parameters needed 
%                            for Simulink model 
%       FIELD NAMES
%        A, B, B_nl, ...    state-space matrices
%         C_va, C_d, ...
%         C_f, C_fnl, D
%        T_nl, w_sens,...   localization matrices
%         w_drive 
%        kt, muN            parameters of the nonlinearity
%
%   responseQuantity        quantity measured by each       1 x 7 cell
%                            virtual response sensor        array
%                            (i.e. 'acceleration', 
%                            'velocity' or 'displacement')
%   indexSensorDrivePoint   index of the drive point among  double
%                           the response sensors
%   linearSUTModes          linear modal properties         struct
%                            of the structure's first
%                            three modes
%       FIELD NAMES
%       frequency_Hz        modal frequency in Hz           1 x 3 double
%       modalDampingRatio   modal damping ratio             1 x 3 double
%       Phi                 mass-normalized deflection      7 x 3 double
%                            shape at seven nodes
%
%   linearPlantModes        linear modal properties         struct
%                            of the plant's first
%                            three modes
%       FIELD NAMES
%       frequency_Hz        modal frequency in Hz           1 x 3 double
%       modalDampingRatio   modal damping ratio             1 x 3 double
%       Phi                 mass-normalized deflection      7 x 3 double
%                            shape at seven nodes
%
%   m_ex                    the exciter's moving mass in    double
%                            kg
%       
%   a_NM                    modal amplitude of the          1 x n double
%                            numerical reference
%   Resp1_NM                fundamental harmonic Fourier    7 x n double
%                            coeficients of the 
%                            displacements at the sensor 
%                            nodes
%   om_NM                   nonlinear modal frequency of    1 x n double
%                            the numerical reference
%   del_NM                  nonlinear modal damping ratio   1 x n double
%                            of the numerical reference
% 
% REFERENCES
% [Hippold.2024] P. Hippold, M. Scheel, L. Renson, M. Krack: Robust and  
%       fast backbone tracking via phase-locked loops, MSSP 220 (2024), 
%       DOI: 10.1016/j.ymssp.2024.111670
% [Scheel.2020] M. Scheel, T. Weigele, M. Krack (2020): Challenging an 
%       experimental nonlinear modal analysis method with a new strongly 
%       friction-damped structure, Journal of Sound and Vibration. 
%       DOI: 10.1016/j.jsv.2020.115580
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

function [virtualRubber, responseQuantity, indexSensorDrivePoint, ...
    linearSUTModes, linearPlantModes, m_ex, ...
    a_NM, Resp1_NM, om_NM, del_NM] = ... 
    defineRubberAdvancedAndDoNumericalNMA(indexTargetMode)

    fprintf('Setting up the plant model...\n');
        
    %% Define structure under test
    
    % Properties of the beam
    len = .710;             % length in m
    height = .05;           % height in the bending direction in m 
    thickness = .06;        % thickness in the third dimension in m
    E = 210e9*0.6725;         % Young's modulus in N/m^2
    rho = 7850;             % density in kg/m^3
    BCs = 'clamped-free';   % constraints
    n_nodes = 15;           % number of equidistant nodes along length

    % parameters of elastic coulomb element
    inode = 6; % node at which nonlinearity is attached
    kt = 1e7; % linear tangential contact stiffness in N/m
    muN = 15; % friction coeff. times normal force in N

    % We assume that each translational FE DOF is a possible sensor
    % location and select seven sensors along the beam.
    sensors = 2:2:14;
    numberOfSensors = length(sensors); 

    % Setup one-dimensional finite element model of an Euler-Bernoulli beam
    beam = FE_EulerBernoulliBeam(len,height,thickness,E,rho,BCs,n_nodes);

    % Apply elastic Coulomb element
    add_nonlinear_attachment(beam,inode,'trans','elasticDryFriction',...
        'stiffness',kt,'friction_limit_force',muN,'ishysteretic',1)

    % Localization of drive point (tip of the beam). Forcing is defined as
    % unit forcing since the goal here is to obtain a localization vector.
    add_forcing(beam,15,'trans',1);

    % localization matrix for sensors
    Tresp = full(sparse(1:numberOfSensors,2*sensors-1,1,numberOfSensors,...
        beam.n));

    %% Hurty/Craig-Bampton reduction
    nm = 3;
    ROMofBeam = CMS_ROM(beam,find(beam.nonlinear_elements{1}.force_direction),nm,'CB');
    ROMofBeam.Tresp = Tresp * ROMofBeam.T;

    %% Modal analysis of the linearized reduced system

    % Modes for sticking contact
    Kt_red = kt*(ROMofBeam.nonlinear_elements{1}.force_direction * ...
        ROMofBeam.nonlinear_elements{1}.force_direction');
    [Phi_st_red,OM2] = eig(ROMofBeam.K + Kt_red, ROMofBeam.M);
    om_st = sqrt(diag(OM2));
    % Sorting
    [om_st,ind] = sort(om_st); Phi_st_red = Phi_st_red(:,ind);
    % mass-normalization
    Phi_st_red = Phi_st_red ./ repmat(sqrt(diag(Phi_st_red' *...
        ROMofBeam.M * Phi_st_red))',ROMofBeam.n,1)  ;

    % Modes for no contact
    [~,OM2] = eig(ROMofBeam.K, ROMofBeam.M);
    om_free = sqrt(diag(OM2));
    % Sorting
    om_free = sort(om_free);

    %% define modal damping

    zeta_lin = 1e-2*ones(size(om_st)); % linear modal damping ratio
    ROMofBeam.D = ((Phi_st_red.')\diag(2*om_st.*zeta_lin))/Phi_st_red;

    %% Nonlinear modal analysis using harmonic balance
    analysis = 'NMA';

    % Analysis parameters
    imod = indexTargetMode;           % mode to be analyzed
    if numel(imod)~=1 || imod<=0|| mod(imod,1)~=0 || imod > length(om_st)
        error(['indexTargetMode must be a positive integer and not '...
            'exceed ' num2str(length(om_st)) ' for this set-up.'])
    end
    H = 7;              % harmonic order
    N = 2^8;           % number of time samples per period
    log10a_s = -5.5;      % start vibration level (log10 of modal mass)
    log10a_e = -3.1;    % end vibration level (log10 of modal mass)
    inorm = ROMofBeam.n-1;   % coordinate for phase normalization

    % Initial guess vector x0 = [Psi;om;del], where del is the modal
    % damping ratio, estimate from underlying linear system (sticking)
    model = ROMofBeam; n = size(Phi_st_red,1);
    om = om_st(imod); phi = Phi_st_red(:,imod);
    Psi = zeros((2*H+1)*n,1);
    Psi(n+(1:n)) = phi;
    x0 = [Psi;om;0];
    psiscl = max(abs((Psi)));

    % Solve and continue w.r.t. Om
    ds = .01;
    Sopt = struct('flag',1,'stepadapt',1,'dynamicDscale',1,...
        'Dscale',[1e-0*psiscl*ones(size(Psi));...
        (om_st(imod)+om_free(imod))/2;1e-1;1e0]);
    fscl = mean(abs(model.K*phi));
    X_NM = solve_and_continue(x0,...
        @(X) HB_residual(X,model,H,N,analysis,inorm,fscl),...
        log10a_s,log10a_e,ds,Sopt);

    % Interpret solver output
    Psi_NM = X_NM(1:end-3,:);
    om_NM = X_NM(end-2,:);
    del_NM = X_NM(end-1,:);
    log10a_NM = X_NM(end,:);
    a_NM = 10.^log10a_NM;
    Q_NM = Psi_NM.*repmat(a_NM,size(Psi_NM,1),1);
    % fundamental harmonic component at sensor locations
    Resp1_NM = ROMofBeam.Tresp*((Q_NM(n+(1:n),:)-1i*Q_NM(2*n+(1:n),:)));

    %% save
    Mred = ROMofBeam.M;
    Kred = ROMofBeam.K;
    Dred = ROMofBeam.D;
    wnlred = ROMofBeam.nonlinear_elements{1}.force_direction;
    wexred = ROMofBeam.Fex1;
    wrespred = ROMofBeam.Tresp;
    
    %% simulate result of EMA
    
    % for this virtual test case the modes must include the mode to be
    % tracked (mode 1) and maximum number of modes is 3.
    indexLinearModes= [1,2,3];
    
    % Transform linear (stuck) mode shapes back to nodal FE coordinates and
    % truncate to sensor DOFs.
    linearSUTModes.linearPhi = wrespred * Phi_st_red(:,indexLinearModes);
    % store natural frequency and damping
    linearSUTModes.linearModalFrequency_Hz = transpose(om_st(indexLinearModes))/2/pi;
    linearSUTModes.linearModalDampingRatio = transpose( ...
        diag(Phi_st_red(:,indexLinearModes).' * Dred ...
        * Phi_st_red(:,indexLinearModes)) ./ (2*om_st(indexLinearModes)) );
    %% Specify instrumentation

    % The virtual beam is instrumented with 7 sensors distributd along the
    % length. The following vector specifies which quantity each sensor
    % measures. First entry is closest to root, last entry closest to tip.
    responseQuantity =  {...
        'displacement', ...
        'velocity', ...
        'displacement', ...
        'velocity', ...
        'displacement', ...
        'velocity', ...
        'acceleration'};

    % Among the 7 sensor locations, give index for drive point location,
    % i.e. where the shaker is attached. That sensor is also used for control.
    indexSensorDrivePoint = 7;

    %% define exciter model
    
    % data of shaker Brüel&Kjaer Type 4809
    k_ex  = 9932; % spring constant Table-Ground (N/m)
    d_ex  = 21.51; % damping constant Table-Ground (Ns/m)
    m_ex  = 0.057; % (kg)
    R     = 2; % resistance (Ohm)
    G     = 6.78; % shaker ratio of thrust to coil current (N/A)
    
    % auxiliary exciter parameters
    % exciter natural frequency
    om_ex = sqrt(k_ex/m_ex); 
    % exciter damping ratio
    zeta_ex  = (d_ex + G^2/R) / (2*m_ex*om_ex); 
    % exciter/structure mass ratio
    mu_ex = (linearSUTModes.linearPhi(indexSensorDrivePoint,:).^2).' * m_ex; 
    
    % simulated exciter EMA, see [Hippold.2024]
    linearPlantModes.linearPlantFrequency_Hz = sqrt((linearSUTModes.linearModalFrequency_Hz.^2 +...
        mu_ex*(om_ex/2/pi)^2) ./ (1+mu_ex));
    linearPlantModes.linearPlantDampingRatio = (linearSUTModes.linearModalDampingRatio +...
        mu_ex*zeta_ex) ./ (1+mu_ex);
    linearPlantModes.linearPlantPhi = sqrt( linearSUTModes.linearPhi .* ...
        repmat(linearSUTModes.linearPhi(indexSensorDrivePoint,:),length(sensors),1) ...
        .* G ./ (R*(1+mu_ex.')) );
    
    %% rigid coupling of exciter armature to drive point  
    M_c = Mred + wexred * m_ex * wexred';
    D_c = Dred + wexred * m_ex*2*zeta_ex*om_ex * wexred';
    K_c = Kred + wexred * m_ex*om_ex^2 * wexred';
    
    %% set up state-space model of virtual plant
    % The state-space system equations is further explained in the
    % README.md to this example.

    % states: [generalized displacement; generalized velocity] (of the 
    % reduced order model)
    % input: voltage (to the exciter)
    % output: physical displacements, velocities, and accelerations at 
    % sesor locations and excitation force

    nred = size(Mred,1);
    
    % linear state-space matrix
    virtualRubber.A = [zeros(nred), eye(nred); ...
        -M_c\K_c, -M_c\D_c];
 
    % input matrix for voltage
    virtualRubber.B = [zeros(nred,1); M_c\wexred*G/R]; 
    % input matrix for nonlinear force
    virtualRubber.B_nl = [zeros(nred,1); ...
        -M_c\wnlred]; 

    % matrix to recover physical displacement at location of nonlinear 
    % element
    virtualRubber.T_nl = [wnlred', zeros(1, nred,1)]; 
    
    % output matrix for (physical) acceleration and velocity
    virtualRubber.C_va = kron(eye(2),wrespred);
    % output matrix for (physical) displacement
    virtualRubber.C_d = kron([1, 0],wrespred);

    virtualRubber.sensorMatrix = zeros(numberOfSensors,3*numberOfSensors);
    for ii = 1:numberOfSensors
        switch responseQuantity{ii}
            case 'displacement'
                virtualRubber.sensorMatrix(ii,ii) = 1;
            case 'velocity'
                virtualRubber.sensorMatrix(ii,numberOfSensors+ii) = 1;
            case 'acceleration'
                virtualRubber.sensorMatrix(ii,2*numberOfSensors+ii) = 1;
            otherwise
                error(['Invalid response quantity for sensor %d. Choose'...
                    ' displacement, velocity or acceleration'],ii);
        end
    end
    
    % output matrix (vector) for linear contribution to excitation force
    virtualRubber.C_f = [m_ex * (wexred'*(M_c\K_c)...
        - om_ex^2*wexred'), ...
        m_ex * (wexred' * (M_c\D_c) - 2*zeta_ex*om_ex*wexred')];
    % output matrix (scalar) for nonlinear contribution to excitation force
    virtualRubber.C_fnl = m_ex*wexred' * ...
        (M_c\wnlred);
    
    % throughput matrix (scalar) for voltage
    virtualRubber.D = G/R * (1 - m_ex*wexred' * (M_c\wexred));
    
    % properties of friction element
    virtualRubber.kt = kt;
    virtualRubber.muN = muN;

    fprintf('...done\n');
end