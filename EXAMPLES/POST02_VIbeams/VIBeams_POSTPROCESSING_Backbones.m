%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script post-processes backbone measurements from [1]. The test rig
% involves two similar cantilevered beams undergoing soft collisions via 
% a unilateral spring. The test methods are described in [1].
% 
% In this script, one selects 1 out of 3 backbone runs. The backbone is
% then determined in terms of modal frequency and damping ratio, as 
% function of the response level. Also, the modal energy contribution is 
% estimated. Plots similar to those in Figure 10 in [1] are generated.
% 
% The modal energy contributions slightly deviate from those shown in
% Figure 10(d). In particular, the contribution of the 4th mode of the 
% right beam to the fundamental harmonic (RB,m=4,h=1) is much lower. This
% is because the linear modal model has been slightly improved.
% 
% The corresponding data has to be available in the BASEFOLDER specified 
% by the user. The data can be downloaded from [2]. For information on the 
% data file names and content structure, see README in [2]. Besides
% measurement data, you also find CAD files and technical drawings, as well
% as a PDF indicating the measurement points in [2].
% 
% REFERENCES
% [1] https://doi.org/10.25518/2684-6500.180
% [2] https://doi.org/10.18419/darus-4504
% [3] https://doi.org/10.1016/j.ymssp.2022.109170
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of NLtest available via
% https://github.com/maltekrack/NLtest.
% 
% COPYRIGHT (C) 2024
%   Malte Krack (malte.krack@ila.uni-stuttgart.de) 
%   Maren Scheel (maren.scheel@ila.uni-stuttgart.de)
%   Lukas Woiwode
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars;
close all;
clc;

filepath = fileparts(mfilename("fullpath"));
cd(filepath);

addpath('../../SRC');
%% Check if folder 'results' exists. Otherwise create it.
list = dir;
[exists,ind] =  ismember('results',{list.name}); 
if ~exists || ~list(ind).isdir
    mkdir results;
end
%% KEY USER INPUT

% Folder with measurements
BASEFOLDER = './DATA/measurements';

% Nominal parameters (for illustration)
gap = .17e-3;
om_linref = 28.2*2*pi;

% Specify configuration, assembly and run number
RUN = 1; % [1|2|3]

% Set of MPV sensors to be used for post-processing
MPVset = setdiff(1:16,[2 8 12]);

% Specify options for post-processing
options.computeFourierCoefficients = 'FFT'; 
options.target_number_of_last_periods_of_hold_phase_to_process = 70;
options.phaseTolerance = deg2rad(1); % 1 degree

%% SET UP LINEAR MODAL BEAM MODEL 
[MODSPEC,om_lin,PhiMb_lin,Phibar_lin] = setUpLinearModalBeamModel(MPVset);

%% LOAD DATA FROM FILES AND PREPARE FOR FURTHER PROCESSING
disp(['Processing backbone run ' num2str(RUN) '.']);

% Set modal frequency of first bending of RB as nominal frequency
options.nominalFrequency = om_linref/(2*pi);

% Generate file name
fname = [BASEFOLDER '/Backbone_SteppedResponseAmplitude_' num2str(RUN)];

% Load time-amplitude profile
steps = readmatrix([fname '_amplitude_steps']);

% Load MPV measurements
disp('Loading backbone measurement for MPV point 17.');
data = readmatrix([fname '_MPV_Point_17']);
time_s = data(:,1);
baseVelocity_m_s = data(:,2);
relativeVelocity_m_s = zeros(size(time_s,1),length(MPVset));
for ii=1:length(MPVset)
    iSens = MPVset(ii);
    if iSens<=9
        PTSPEC = ['_' num2str(iSens)]; 
    else
        PTSPEC = num2str(iSens); 
    end
    disp(['Loading backbone measurement for MPV point ' num2str(iSens) '.']);
    data = readmatrix([fname '_MPV_Point_' PTSPEC]);
    relativeVelocity_m_s(:,ii) = data(:,2) - baseVelocity_m_s;
end

% Load control signals and SPV measurements
disp('Loading backbone control signals and SPV measurements.');
data = readmatrix([fname '_dspace']);
% Truncate to common time span
isCommonTime = data(:,1)>=max(data(1,1),time_s(1)) & ...
    data(:,1)<=min(data(end,1),time_s(end));
data = data(isCommonTime,:);
isCommonTime = time_s>=max(data(1,1),time_s(1)) & ...
    time_s<=min(data(end,1),time_s(end));
time_s = time_s(isCommonTime);
relativeVelocity_m_s = relativeVelocity_m_s(isCommonTime,:);
% Shift start of first hold phase (otherwise it will not be considered in
% the function 'estimateFourierCoefficientsFromSteppedSine')
steps(1,1) = max(time_s(1),data(1,1));
% Interpolate control signals and SPV measurements onto MPV time axis
instantaneousFrequency_Hz = interp1(data(:,1),data(:,5),time_s);
excitation_m_s = interp1(data(:,1),data(:,3),time_s);
response_m_s = interp1(data(:,1),data(:,4),time_s);

%% DETERMINE HOLD PHASES

% Interpret step intput
step_times = steps(:,1);
step_values = steps(:,2);
% Truncate according to available data
data_available = step_times>=time_s(1) & step_times<=time_s(end);
step_times = step_times(data_available);
step_values = step_values(data_available);

% Identify hold phases and keep those that are long enough
isStartOfHoldPhase = find(diff(step_values)==0);
holdPhase_start = step_times(isStartOfHoldPhase);
holdPhase_end = step_times(isStartOfHoldPhase+1);
% If hold time is too short, scrap hold phase
isLongEnough = (holdPhase_end-holdPhase_start)*options.nominalFrequency > ...
    options.target_number_of_last_periods_of_hold_phase_to_process;
holdPhase_end = holdPhase_end(isLongEnough);
holdPhase_start = holdPhase_start(isLongEnough);
numHoldPhases = size(holdPhase_end,1);

% create flagSettling needed for postprocessing
flagSettling = false(size(time_s));
for iHold = 1:numHoldPhases
    flagtemp = holdPhase_start(iHold)<=time_s & time_s<= holdPhase_end(iHold);
    flagSettling = flagSettling | flagtemp;
end
flagSettling(1)=false; % start with a non-recording phase
%% ESTIMATE FOURIER COEFFICIENTS FROM BACKBONE MEASUREMENTS
[frequency_Hz,Excitation_m_s,Response] = ...
    estimateFourierCoefficientsFromSteppedSine(time_s,...
    instantaneousFrequency_Hz,...
    excitation_m_s,[relativeVelocity_m_s response_m_s],flagSettling,...
    options);

Response_m_s = Response(:,:,14);
Response_m_s_1H = Response_m_s(:,2);
RelativeVelocity_m_s = Response(:,:,1:13);

% Remove points that do not meet the phase resonance condition to a
% specified tolerance
meetsPhaseTolerance = abs(-pi/2 - ...
    angle(Response_m_s(:,2)./Excitation_m_s(:,2))) < ...
    options.phaseTolerance;
        
frequency_Hz = frequency_Hz(meetsPhaseTolerance);
Excitation_m_s = Excitation_m_s(meetsPhaseTolerance,:);
% Response_m_s_1H = Response_m_s_1H(meetsPhaseTolerance);
Response_m_s = Response_m_s(meetsPhaseTolerance,:);
RelativeVelocity_m_s = RelativeVelocity_m_s(meetsPhaseTolerance,:,:);
%% DETERMINE MODAL DAMPING RATIO AND MODAL ENERGY CONTRIBUTIONS
% The model-based method proposed in [3] is used, which corresponds to Eq.
% 14 in [1].

% Convert velocity to displacement Fourier coefficients
H = size(Excitation_m_s,2)-1; % number of harmonics
RelativeDisplacement_m = RelativeVelocity_m_s ./ ...
    repmat((2*pi*frequency_Hz)*1i*(0:H),1,1,size(RelativeVelocity_m_s,3));
% Base acceleration amplitude
Aexc_m_s2 = (1i*(2*pi*frequency_Hz)*(0:H)).*Excitation_m_s;

[modalFrequency_Hz,modalDampingRatio] = ...
    estimateModalPropertiesFromBackbone(frequency_Hz,...
    RelativeDisplacement_m,Aexc_m_s2,'modelBased',9,PhiMb_lin,Phibar_lin);

% Loop over hold times
E = zeros(size(Phibar_lin,2),H,size(Excitation_m_s,1));
for iHold = 1:length(frequency_Hz)
    % Estimate complex modal amplitudes via least-squares regression
    Eta = Phibar_lin\transpose(squeeze(RelativeDisplacement_m(iHold,2:end,:)));
    
    % Contribution to period-average of the mechanical energy, per mode and
    % harmonic (Eq. 4 in [1])
    E(:,:,iHold) = 1/4*( ...
        repmat(((1:H)*(2*pi*frequency_Hz(iHold))),size(Eta,1),1).^2 + ...
        repmat(om_lin(:),1,size(Eta,2)).^2 ) .* abs(Eta).^2;
end


% Reference: RB,m=1,h=1
[~,mref] = ismember('RB,m=1',MODSPEC);
href = 1;
ref = E(mref,href,:);

% Identify most important contributions
[row,col] = find(sum(E./repmat(ref,size(E,1),size(E,2),1)>.7e-2,3)>=1);
% Remove reference
iref = find(row==mref & col==1);
row(iref) = []; col(iref) = [];

figure; hold on;
legstr = cell(length(row),1);
for i=1:length(row)
    plot(modalFrequency_Hz/(om_linref/(2*pi)),...
        squeeze(E(row(i),col(i),:)./ref));
    legstr{i} = [MODSPEC{row(i)} ',h=' num2str(col(i))];
end
legend(legstr{:});
xlabel('\Omega/\omega_1^{RB}');
ylabel('E(m,h)/E^{RB}(1,1)');
set(gca,'yscale','log','xlim',[.99 1.03],'ylim',[2e-4 2e0]);
%% ILLUSTRATE MODAL PROPERTIES (Figure 10 in [1])

% Define response and excitation level as in [1]
aresp = abs(Response_m_s(:,2)./(1i*2*pi*modalFrequency_Hz));
aexc = abs(1i*2*pi*modalFrequency_Hz.*Excitation_m_s(:,2));

% Response level vs. excitation frequency
figure; hold on; grid on;
plot(modalFrequency_Hz/(om_linref/(2*pi)),aresp/gap,'g--x');
xlabel('\Omega/\omega_1^{RB}');
ylabel('aresp/g');
set(gca,'xlim',[.99 1.03],'ylim',[.8 1.7]);

% Excitation level vs. excitation frequency
figure; hold on; grid on;
plot(modalFrequency_Hz/(om_linref/(2*pi)),aexc,'g--x');
xlabel('\Omega/\omega_1^{RB}');
ylabel('aexc in m/s^2');
set(gca,'xlim',[.99 1.03],'ylim',[0 .04]);

% Modal damping ratio vs. response level
figure; hold on; grid on;
plot(aresp/gap,modalDampingRatio*1e2,'g--x');
xlabel('aresp/g');
ylabel('damping ratio in %');
set(gca,'xlim',[.8 1.7],'ylim',[0. .35]);
%% Save results
save(['results/Backbone_' num2str(RUN) '.mat'],'modalFrequency_Hz',...
    'Excitation_m_s','Response_m_s','RelativeVelocity_m_s',...
    'modalDampingRatio');
