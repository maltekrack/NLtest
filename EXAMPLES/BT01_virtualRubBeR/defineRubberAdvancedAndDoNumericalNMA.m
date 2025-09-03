%========================================================================
% DESCRIPTION: 
% This function sets up a mathematical model of a friction-damped 
% cantilevered beam, inspired by the RubBeR test rig [Scheel.2020]. For 
% numerical reference, the modal parameters are computed directly from the 
% mathematical model of the structure under test in accordance with the 
% Extended Periodic Motion Concept (EPMC).
% 
% The beam is modelled using 1D finite elements and Euler-Bernoulli theory. 
% A reduced-order model of the beam is derived using the common 
% Hurty-/Craig-Bampton method. Friction-damping is introduced by a single
% elastic dry friction element (Jenkins element) at ~1/3 from the clamping.
% More specifically, it is attached to pocket 2, labeled with 'friction 
% force' in Fig. 5 of [Scheel.2020]. The 7 sensor locations indicated in 
% that figure are defined for virtual testing of different instrumentation
% configurations. For the exciter, a simple electro-mechanical model is 
% defined, somewhat representative of a Brüel&Kjaer Type 4809 shaker. The 
% stinger is assumed as rigidly attached to the drive point. The exciter
% can be attached to each of the 7 sensor nodes; on default, the free
% end of the beam is used as drive point. The state space model of the 
% plant is described in section 'State-space model of the advanced 
% virtual experiment' of DOC/BackboneTracking.md.
% 
% To set up the model and to compute the reference modal parameters,
% the tool NLvib is used. Please download the recent NLvib version from 
% https://github.com/maltekrack/NLvib/. Extract it in a sub-folder within 
% the SRC folder or extract it anywhere else and extend the search path
% accordingly.
% 
% INPUT
%   VARIABLE                MEANING                         TYPE
%     indexTargetMode       index of mode to be analyzed    double
%
%
% OUTPUT
%   VARIABLE                MEANING                         TYPE
%   virtualRubber         contains matrices and           struct
%                            parameters needed 
%                            for Simulink model
%   responseQuantity      quantity measured by each       1 x 7 cell
%                            sensor node                    array
%                            (i.e. 'acceleration', 
%                            'velocity' or 'displacement')
%   indexSensorDrivePoint index of the drive point among  double
%                            the sensor nodes
%   linearModesSUT        linear modal properties         struct
%                            of the structure under test
%                            (information is gathered on
%                            the 3 lowest-frequency modes
%                            on default)
%       FIELD NAMES
%       frequency_Hz        modal frequency in Hz           1 x 3 double
%       modalDampingRatio   modal damping ratio             1 x 3 double
%       Phi                 mass-normalized deflection      7 x 3 double
%                            shape at seven nodes
%
%   linearModesPlant        linear modal properties         struct
%                            of the plant's first
%                            three modes
%       FIELD NAMES
%       frequency_Hz        modal frequency in Hz           1 x 3 double
%       modalDampingRatio   modal damping ratio             1 x 3 double
%       Phi                 mass-normalized deflection      7 x 3 double
%                            shape at seven nodes
%   exciterMass             exciter moving mass in kg       double
%   nonlinearModeSUT        nonlinear modal properties      struct
%                            of the structure under test
%                            with the index specified by
%                            the user in accordance with
%                            EPMC (numerical reference);
%                            path continuation yields 'n'
%                            solution points in the 
%                            specified amplitude range
%       FIELD NAMES
%       a                   modal amplitude                 1 x n double
%       Resp1               fundamental harmonic Fourier    7 x n double
%                            coefficient vector of the 
%                            displacement at the sensor 
%                            nodes
%       frequency_Hz        nonlinear modal frequency       1 x n double
%       modalDampingRatio   nonlinear modal damping ratio   1 x n double
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
    linearSUTModes, linearPlantModes, exciterMass, nonlinearModeSUT] = ...
    defineRubberAdvancedAndDoNumericalNMA(indexTargetMode)
    %% Define structure under test, incl. drive point and sensor config.
    
    % Set up one-dimensional finite element model of Euler-Bernoulli beam
    len = .710;             % length in m
    height = .05;           % height in the bending direction in m
    thickness = .06;        % thickness in the third dimension in m
    E = 210e9*0.6725;       % elastic modulus in N/m^2
    rho = 7850;             % mass density in kg/m^3
    BCs = 'clamped-free';   % boundary conditions
    n_nodes = 15;           % number of equidistant nodes along length
    beam = FE_EulerBernoulliBeam(len,height,thickness,E,rho,BCs,n_nodes);

    % Add elastic dry friction element
    kt = 1e7;               % (tangential) contact stiffness in N/m
    muN = 15;               % friction coeff. times normal force in N
    inode = 6;              % node index to which element is attached
    add_nonlinear_attachment(beam,inode,'trans','elasticDryFriction',...
        'stiffness',kt,'friction_limit_force',muN,'ishysteretic',1)    

    % Add forcing to drive point
    inode = n_nodes;        % node index to which force is applied
    force = 1;              % force set to 1 to obtain localization vector
    add_forcing(beam,inode,'trans',force);

    % Specify sensor nodes as every second node starting with node 3, 
    % counting from clamping to tip. With 15 nodes, this yields 7 sensor 
    % nodes. These correspond to those in Fig. 5 of [Scheel.2020].
    idxSensorNodes = (3:2:n_nodes)-1;
    numberOfSensors = numel(idxSensorNodes);
    responseQuantity =  {...
        'displacement', ...
        'velocity', ...
        'displacement', ...
        'velocity', ...
        'displacement', ...
        'velocity', ...
        'acceleration'};

    % Specify index of drive point sensor (here: ollocated w/ last sensor)
    indexSensorDrivePoint = numberOfSensors;
    
    % Localization matrix for displacement degrees of freedom of sensors
    Tresp = full(sparse(1:numberOfSensors,2*idxSensorNodes-1,1,...
        numberOfSensors,beam.n));

    %% Hurty/Craig-Bampton reduction 

    % Specify number of normal modes
    nm = 3;

    % Specify coordinate associated with the nonlinear element to be 
    % retained as boundary coordinate
    ib = find(beam.nonlinear_elements{1}.force_direction);

    % Apply reduction
    ROM = CMS_ROM(beam,ib,nm,'CB');

    % Set up matrix for retrieving displacement at sensor nodes 
    ROM.Tresp = Tresp * ROM.T;

    %% Linear modes of structure under test (sticking contact)

    % Effective stiffness including contact
    wNL = ROM.nonlinear_elements{1}.force_direction;
    Keff = ROM.K + kt*(wNL*wNL');

    % Solve eigenvalue problem
    [Phi,OM2] = eig(Keff,ROM.M);

    % Sort by ascending frequency; mass-normalize deflection shape
    [om,ind] = sort(sqrt(diag(OM2)));
    Phi = Phi(:,ind);
    Phi = Phi ./ repmat(sqrt(diag(Phi' *...
        ROM.M * Phi))',ROM.n,1)  ;

    % Set linear modal damping
    zeta_lin = 1e-2*ones(size(om));
    ROM.D = ((Phi.')\diag(2*om.*zeta_lin))/Phi;
    
    % For this virtual test case the modes must include the mode to be
    % tracked (mode 1) and the maximum number of modes is 3. For more
    % modes, the number of normal modes in the reduction, and possibly the
    % number of nodes in the FE model should be increased.
    indexLinearModes= [1,2,3];
    
    % Mimick experimental linear modal analysis by storing modal 
    % frequencies, damping ratios and deflection shape entries at sensor
    % nodes in linearSUTModes variable.
    linearSUTModes.linearModalFrequency_Hz = om(indexLinearModes)'/(2*pi);
    linearSUTModes.linearModalDampingRatio = diag(Phi(:,indexLinearModes).' * ROM.D ...
        * Phi(:,indexLinearModes))' ./ (2*om(indexLinearModes)');
    linearSUTModes.linearPhi = ROM.Tresp * Phi(:,indexLinearModes);

    %% Nonlinear modal analysis of target mode

    % Check if specified indexTargetMode has suitable value
    if numel(indexTargetMode)~=1 || indexTargetMode<=0|| ...
            mod(indexTargetMode,1)~=0 || indexTargetMode > length(om)
        error(['indexTargetMode must be a positive integer and not '...
            'exceed ' num2str(length(om)) ' for this set-up.'])
    end

    % NLvib/Harmonic Balance solver parameters
    H = 7;              % harmonic order
    N = 2^8;            % number of time samples per period
    analysis = 'NMA';   % analysis type ('NMA' corresponds to EPMC)
    log10a_s = -5.5;    % start vibration level (log10 of modal mass)
    log10a_e = -3.1;    % end vibration level (log10 of modal mass)
    inorm = ROM.n-1;    % coordinate for phase normalization

    % Initial guess vector x0 = [Psi;om;del], where del is the modal
    % damping ratio, estimate from underlying linear system (sticking)
    n = size(Phi,1);
    omlin = om(indexTargetMode);
    philin = Phi(:,indexTargetMode);
    Psi = zeros((2*H+1)*n,1);
    Psi(n+(1:n)) = philin;
    x0 = [Psi;omlin;0];

    % Set continuation and preconditioning options, call solve and continue
    ds = .01;
    psiscl = max(abs((Psi)));
    Sopt = struct('flag',1,'stepadapt',1,'dynamicDscale',1,...
        'Dscale',[1e-0*psiscl*ones(size(Psi));...
        om(indexTargetMode);1e-1;1e0]);
    fscl = mean(abs(ROM.K*philin));
    X_NM = solve_and_continue(x0,...
        @(X) HB_residual(X,ROM,H,N,analysis,inorm,fscl),...
        log10a_s,log10a_e,ds,Sopt);

    % Interpret solver output and store results in nonlinearModeSUT
    % variable
    Psi_NM = X_NM(1:end-3,:);
    nonlinearModeSUT.frequency_Hz = X_NM(end-2,:)/(2*pi);
    nonlinearModeSUT.modalDampingRatio = X_NM(end-1,:);
    log10a_NM = X_NM(end,:);
    nonlinearModeSUT.a = 10.^log10a_NM;
    Q_NM = Psi_NM.*repmat(nonlinearModeSUT.a,size(Psi_NM,1),1);
    Q_NM1 = Q_NM(n+(1:n),:)-1i*Q_NM(2*n+(1:n),:);
    nonlinearModeSUT.Resp1 = ROM.Tresp*Q_NM1;

    %% Define exciter model
    
    % Data of shaker Brüel&Kjaer Type 4809
    k_ex = 9932;            % spring constant Table-Ground (N/m)
    d_ex = 21.51;           % damping constant Table-Ground (Ns/m)
    exciterMass = 0.057;    % (kg)
    R = 2;                  % resistance (Ohm)
    G = 6.78;               % shaker ratio of thrust to coil current (N/A)
    
    % Auxiliary exciter parameters
    % exciter natural frequency
    om_ex = sqrt(k_ex/exciterMass); 
    % exciter damping ratio
    zeta_ex = (d_ex + G^2/R) / (2*exciterMass*om_ex); 
    % exciter/structure mass ratio
    mu_ex = (linearSUTModes.linearPhi(indexSensorDrivePoint,:).^2).' * ...
        exciterMass; 
    
    % Mimick experimental linear modal analysis of plant by storing 
    % properties as defined in [Hippold.2024]
    linearPlantModes.linearPlantFrequency_Hz = ...
        sqrt((linearSUTModes.linearModalFrequency_Hz.^2 +...
        mu_ex*(om_ex/2/pi)^2) ./ (1+mu_ex));
    linearPlantModes.linearPlantDampingRatio = ...
        (linearSUTModes.linearModalDampingRatio +...
        mu_ex*zeta_ex) ./ (1+mu_ex);
    linearPlantModes.linearPlantPhi = sqrt( linearSUTModes.linearPhi .* ...
        repmat(linearSUTModes.linearPhi(indexSensorDrivePoint,:),...
        numberOfSensors,1) ...
        .* G ./ (R*(1+mu_ex.')) );

    %% Set up state-space model of virtual plant
    % states: [generalized displacement; gen. velocity] (reduced model)
    % input: voltage (to the exciter)
    % output: physical displacements, velocities, and accelerations at 
    % sensor locations and excitation force

    % Auxiliary matrices and vectors
    Mred = ROM.M;
    Kred = ROM.K;
    Dred = ROM.D;
    wnlred = ROM.nonlinear_elements{1}.force_direction;
    wexred = ROM.Fex1;
    wrespred = ROM.Tresp;

    % Number of reduced coordinates of structure under test
    nred = size(Mred,1);

    % Implement rigid coupling of exciter armature to drive point
    M_c = Mred + wexred * exciterMass * wexred';
    D_c = Dred + wexred * exciterMass*2*zeta_ex*om_ex * wexred';
    K_c = Kred + wexred * exciterMass*om_ex^2 * wexred';
    
    % Linear state-space matrix
    virtualRubber.A = [zeros(nred), eye(nred); ...
        -M_c\K_c, -M_c\D_c];
 
    % Input matrix for voltage
    virtualRubber.B = [zeros(nred,1); M_c\wexred*G/R]; 
    
    % Input matrix for nonlinear force
    virtualRubber.B_nl = [zeros(nred,1); ...
        -M_c\wnlred]; 

    % Matrix to recover displacement at location of nonlinear element
    virtualRubber.T_nl = [wnlred', zeros(1, nred,1)]; 
    
    % Output matrix for (physical) acceleration and velocity
    virtualRubber.C_va = kron(eye(2),wrespred);

    % Output matrix for (physical) displacement
    virtualRubber.C_d = kron([1, 0],wrespred);

    % Sensor matrix
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
    
    % Output matrix (vector) for linear contribution to excitation force
    virtualRubber.C_f = [exciterMass * (wexred'*(M_c\K_c)...
        - om_ex^2*wexred'), ...
        exciterMass * (wexred' * (M_c\D_c) - 2*zeta_ex*om_ex*wexred')];

    % Output matrix (scalar) for nonlinear contribution to excitation force
    virtualRubber.C_fnl = exciterMass*wexred' * ...
        (M_c\wnlred);
    
    % Throughput matrix (scalar) for voltage
    virtualRubber.D = G/R * (1 - exciterMass*wexred' * (M_c\wexred));
    
    % Add fields for properties of friction element
    virtualRubber.kt = kt;
    virtualRubber.muN = muN;
end