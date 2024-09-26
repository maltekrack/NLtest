%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script post-processes data acquired during Phase Resonance 
% Testing of the benchmark system of the Tribomechadynamics Research
% Challenge. In the tests, the voltage input to the shaker was stepped,
% while a resonant phase lag between response and excitation was the
% objective of a phase-locked loop.
% 
% The challenge is described in [1]. The tests were part of Project 3 of 
% the Tribomechadynamics Research Camp 2022 in Stuttgart [2]. The test 
% methods are described in [3]. CAD models, technical drawings and design 
% documentation are available in [4].
% 
% In this script, one selects 1 out of 4 configurations (and assembly and
% run number in case of configurations 1-2). The backbone is then
% determined in terms of modal frequency and damping ratio, as function of
% the response level. Also, the modal energy contribution is estimated, and
% frequency response curves are reconstructed, using single-nonlinear-mode
% theory, for specified base acceleration levels. Figures similar to 
% Figures 8, 11 and 13 in [3] are generated.
% 
% The corresponding data has to be available in the BASEFOLDER specified 
% by the user. The data can be downloaded from [5]. For information on the 
% data file names and content structure, see README in [5].
% 
% REFERENCES
% [1] http://tmd.rice.edu/ tribomechadynamics-research-challenge-2021/
% [2] http://tmd.rice.edu/tribomechadynamics-research-camp/2022-graduate-projects/
% [3] https://arxiv.org/abs/2403.07438
% [4] https://doi.org/10.18419/darus-3147
% [5] https://doi.org/10.18419/darus-4484
% [6] https://doi.org/10.1016/j.ymssp.2022.109170
% [7] https://doi.org/10.25518/2684-6500.180
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of NLtest available via
% https://github.com/maltekrack/NLtest.
% 
% COPYRIGHT (C) 2024
%   Malte Krack (malte.krack@ila.uni-stuttgart.de) 
%   Maren Scheel (maren.scheel@ila.uni-stuttgart.de)
%   and the co-authors of [3].
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars;
close all;
clc;
addpath('../../SRC');
%% KEY USER INPUT

% Folder with measurements
BASEFOLDER = '';

% Specify configuration, assembly and run number
CONFIG = 1; % [1|2|3|4]
ASSEMB = 2; % [1|2|3] 
RUN = 2;
% Each configuration was tested for three assemblies.
% Only some configurations were tested multiple runs before re-assembly:
% RUN [1|2|3|4|5]   for CONFIG = 1; ASSEMB = 1;
% RUN [1|2|3|4]     for CONFIG = 1; ASSEMB = 2;
% RUN [1|2|3]       for CONFIG = 2; ASSEMB = 1;
% RUN [1|2]         for CONFIG = 2; ASSEMB = 2;

% Base acceleration levels for ECT reconstruction
ReconstrBaseAcc_m_s2 = 2:5;
% ReconstrBaseAcc_m_s2 = [];   % Assign empty array if you wish to omit the
%                              % reconstruction.

% Specify options for post-processing
options.target_number_of_last_periods_of_hold_phase_to_process = 70;
%% LOAD DATA FROM FILES AND PREPARE FOR PROCESSING

% Read linear modal data (column 1: first bending; column 2: first torsion)
modes = readmatrix([BASEFOLDER '/config' num2str(CONFIG) '/Config' ...
    num2str(CONFIG) '_linear_complex_modes']);
Philin = modes(1:15,:);     % non-normalized modal deflection shapes
omlin = 2*pi*modes(16,:);   % linear modal frequency in rad/s

% Set modal frequency of first bending as nominal frequency (in Hz)
options.nominalFreq = omlin(1)/(2*pi);

% Generate data and step file name
tmp = [BASEFOLDER '/config' num2str(CONFIG) '/Config' ...
    num2str(CONFIG) '_PRT'];
dataFileName = [tmp '_Assemb' num2str(ASSEMB)];
if CONFIG==1 || CONFIG==2
    dataFileName = [dataFileName '_' num2str(RUN)];
end
stepFileName = [tmp '_amplitude_steps'];

% Read data file
data = readmatrix(dataFileName);

% Read step file
steps = readmatrix(stepFileName);

% Interpret data
time_s = data(:,1);                         % time in s
velocity_m_s = data(:,2:18);                % velocity in m/s
instantaneousFrequency_Hz = data(:,19);     % instant. frequency in Hz
instantaneousPhase_degree = data(:,20);     % instant. phase in °

% Determine base velocity (excitation) and relative velocity at sensor
% location 8 (response)
excitation_m_s = ( velocity_m_s(:,16) + velocity_m_s(:,17) ) / 2;
relativeVelocity_m_s = velocity_m_s(:,1:15)-repmat(excitation_m_s,1,15);
response_m_s = relativeVelocity_m_s(:,8);
%% ESTIMATE FOURIER COEFFICIENTS FROM PRT (end of hold phases)
[frequency_Hz,Excitation_m_s,Response_m_s,RelativeVelocity_m_s] = ...
    estimateFourierCoefficientsFromSteppedSine(steps,time_s,...
    instantaneousFrequency_Hz,...
    excitation_m_s,response_m_s,relativeVelocity_m_s,...
    options);

% Response level: magnitude of fundamental harmonic displacement at sensor
% location 8
responseLevel_mm = 1e3*abs(Response_m_s(:,2))./(2*pi*frequency_Hz);
%% DETERMINE MODAL DAMPING RATIO
% The model-free method proposed in [6] is used. To this end, the integrals
% in Eq. 28 are evaluated using the trapezoidal rule. Note that the
% velocities are measured in the direction of the base motion, simplifying
% the right hand side of Eq. 28 substantially.

% Specify coordinates of MPV sensors (see Figure 3 in [3]), needed for
% integration
x_m = (-150:50:150)*1e-3;
z_m = (-50:50:50)*1e-3;

% Loop over amplitudes
modalDampingRatio = zeros(size(responseLevel_mm));
for ia = 1:length(responseLevel_mm)
    % Base displacement amplitude (fundamental harmonic)
    Qb = Excitation_m_s(ia,2)./(1i*2*pi*frequency_Hz(ia));

    % Relative displacement ...
    Q = reshape(RelativeVelocity_m_s{ia}(2,:)/(1i*2*pi*frequency_Hz(ia)),...
        length(x_m)-2,length(z_m));

    % Extend by zero values at the clamping (x=+/-150mm)
    Q = [zeros(length(z_m),1) transpose(Q) zeros(length(z_m),1)];

    % Apply trapezoidal quadrature rule for approximating integrals
    IQ = trapz(x_m,trapz(z_m,Q));
    IQQ = trapz(x_m,trapz(z_m,abs(Q).^2));

    % Evaluate Eq. 28 in [3]
    modalDampingRatio(ia) = 1/2*abs(IQ)/abs(IQQ)*abs(Qb);
end
%% ILLUSTRATE MODAL PROPERTIES (Figure 8 in [3])

% Modal frequency
[~,itp] = max(responseLevel_mm);
figure(1); hold on; grid on;
plot(responseLevel_mm(1:itp),frequency_Hz(1:itp),...
    'o','linewidth',1.5,'markersize',2)
plot(responseLevel_mm(itp+1:end),frequency_Hz(itp+1:end),...
    's','linewidth',1.5,'markersize',2)
xlabel('displacement amplitude (panel center) in mm');
ylabel('\omega /(2\pi) in Hz');
set(gca,'xlim',[0 2.5],'ylim',[94 109]);

% Modal damping ratio
figure(2); hold on; grid on;
plot(responseLevel_mm(1:itp),modalDampingRatio(1:itp)*1e2,...
    'o','linewidth',1.5,'markersize',2)
plot(responseLevel_mm(itp+1:end),modalDampingRatio(itp+1:end)*1e2,...
    's','linewidth',1.5,'markersize',2)
xlabel('displacement amplitude (panel center) in mm');
ylabel('damping ratio in %');
set(gca,'xlim',[0 2.5],'ylim',[0 1]);
%% RECONSTRUCTION OF ECT RESULTS (Figure 11 in [3])
for iECT=1:length(ReconstrBaseAcc_m_s2)
    om = 2*pi*frequency_Hz;
    D = modalDampingRatio;
    Qbabs = abs(Excitation_m_s(:,2)./(1i*om));
    rad = (2*D*ReconstrBaseAcc_m_s2(iECT)./(om.^2.*Qbabs)).^2 - ...
        ( 1 - (1-2*D.^2).^2 );
    OmL = sqrt(om.^2.*( 1-2*D.^2 - sqrt(rad) ));
    OmR = sqrt(om.^2.*( 1-2*D.^2 + sqrt(rad) ));

    % Illustrate results
    figure; hold on;
    plot(om/(2*pi),responseLevel_mm,'ko');
    plot(OmL(rad>=0)/(2*pi),responseLevel_mm(rad>=0),'cx');
    plot(OmR(rad>=0)/(2*pi),responseLevel_mm(rad>=0),'cx');
    set(gca','xlim',[min(om/(2*pi))*.9 1.1*max(om/(2*pi))],'ylim',[0 2.5])
end
%% DETERMINE MODAL ENERGY CONTRIBUTIONS (Figure 13 in [3])

% Phase normalization: rotate so that max. magnitude entry has zero phase
[~,imx] = max(abs(Philin(:,1)));
Philin(:,1) = Philin(:,1)*abs(Philin(imx,1))/Philin(imx,1);
[~,imx] = max(abs(Philin(:,2)));
Philin(:,2) = Philin(:,2)*abs(Philin(imx,2))/Philin(imx,2);

% Mass normalization: This follows the model-free approach in [6]. It is
% assumed that the 5 sensors in the central row have twice the mass
% associated with them as the 5 at the top and the 5 at the bottom row. The
% mass is actually not needed, since relative energy contributions are
% depicted.
% NOTE: Zero-padding and trapezoidal integration as for modal damping ratio
% yields the same result.
m = sqrt(sum([ones(5,2);2*ones(5,2);ones(5,2)].*Philin.^2));
Philin = Philin./repmat(m,size(Philin,1),1);

% Loop over hold phases
H = size(Excitation_m_s,2)-1;
E = zeros(size(Excitation_m_s,1),size(Philin,2),H);
for iHold=1:size(E,1)
    % Convert velocity to displacement Fourier coefficients (and transpose
    % and restrict to dynamic part)
    RelativeDisplacement_m = ...
        transpose(RelativeVelocity_m_s{iHold}(2:end,:) ./ ...
        repmat(1i*2*pi*(1:H)'*...
        frequency_Hz(iHold),1,size(relativeVelocity_m_s,2)));
    % Estimate complex modal amplitudes via least-squares regression
    eta = Philin\RelativeDisplacement_m;
    % Contribution to period-average of the mechanical energy, per mode and
    % harmonic, see e.g. [7]
    E(iHold,:,:) = 1/4*( ...
        repmat(((1:H)*2*pi*frequency_Hz(iHold)),size(eta,1),1).^2 + ...
        repmat(omlin(:),1,size(eta,2)).^2 ) .* abs(eta).^2;
end
ref = squeeze(E(:,1,1));

% % Identify relevant contributions
% squeeze(sum(E./repmat(ref,1,size(E,2),size(E,3))>1e-2,1)>=1)

% Illustrate results
figure; hold on;
plot(responseLevel_mm,squeeze(E(:,1,2)./ref));
plot(responseLevel_mm,squeeze(E(:,2,2)./ref));
legend('m=1,h=2','m=2,h=2','LOCATION','NW');
xlabel('center displacement in mm');
ylabel('E(m,h)/E(1,1)');
