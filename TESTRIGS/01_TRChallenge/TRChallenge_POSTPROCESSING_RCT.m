%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script post-processes data acquired during Response Controlled 
% Testing of the benchmark system of the Tribomechadynamics Research
% Challenge. In the tests, the set value of the phase lag between response 
% and excitation was stepped, while the response level was kept fixed,
% using feedback control.
% 
% The challenge is described in [1]. The tests were part of Project 3 of 
% the Tribomechadynamics Research Camp 2022 in Stuttgart [2]. The test 
% methods are described in [3]. CAD models, technical drawings and design 
% documentation are available in [4].
% 
% In this script, one selects 1 out of 4 configurations. This script then 
% post-processes the corresponding data and generates a figure similar to 
% Figure 10 in [3].
% 
% The corresponding data has to be available in the BASEFOLDER specified 
% by the user. The data can be downloaded from [5]. For information on the 
% data file names and content structure, see README in [5].
% 
% REFERENCES
% [1] http://tmd.rice.edu/ tribomechadynamics-research-challenge-2021/
% [2] http://tmd.rice.edu/tribomechadynamics-research-camp/2022-graduate-projects/
% [3] https://doi.org/10.25518/2684-6500.206
% [4] https://doi.org/10.18419/darus-3147
% [5] https://doi.org/10.18419/darus-4484
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

% Specify configuration [1|2|3|4]
CONFIG = 4;

% Specify options for post-processing
options.target_number_of_last_periods_of_hold_phase_to_process = 500;

% Read representative fundamental frequency for configuration and set
% as nominal
modes = readmatrix([BASEFOLDER 'config' num2str(CONFIG) '/Config' ...
    num2str(CONFIG) '_linear_complex_modes']);
options.nominalFreq = modes(end,1);

% Specify whether to plot circle fits
illustrateCircleFits = true;
%% AVAILABLE RESPONSE LEVELS PER CONFIGURATION
switch CONFIG
    case 1
        responseLevel_mm = [0.40 0.55 0.70 0.85 1.00 1.10 1.20 1.30];
    case 2
        responseLevel_mm = [0.40 0.70 1.00 1.10];
    case 3
        responseLevel_mm = [0.3 0.5 0.7 0.9 1.0 1.1 1.2];
    case 4
        responseLevel_mm = .9; %[0.3 0.4 0.5 0.6 0.7 0.9 1.0 1.1 1.2];
    otherwise
        error(['No available response levels specified for CONFIG' ...
            num2str(CONFIG)]);
end
%% Loop over response levels
modalFrequency_Hz = nan(length(responseLevel_mm),1);
modalDampingRatio_mean = nan(length(responseLevel_mm),1);
modalDampingRatio_std = nan(length(responseLevel_mm),1);
for iRespLev = 1:length(responseLevel_mm)
    %% LOAD DATA FROM FILES AND PREPARE FOR PROCESSING
    disp(['Processing data file ' num2str(iRespLev) ...
        ' of ' num2str(length(responseLevel_mm)) '.']);

    % Generate data and step file name
    tmp = [BASEFOLDER 'config' num2str(CONFIG) '/Config' ...
        num2str(CONFIG) '_RCT'];
    switch CONFIG 
        case 1
            dataFileName = [tmp '_' strrep(num2str(...
                responseLevel_mm(iRespLev),'%.2f'),'.','pt') 'mm'];
            stepFileName = [dataFileName '_phase_steps'];
        case {2,3,4}
            dataFileName = [tmp '_' strrep(num2str(...
                responseLevel_mm(iRespLev),'%.1f'),'.','pt') 'mm'];
            stepFileName = [tmp '_phase_steps'];
    end

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
    %% ESTIMATE FOURIER COEFFICIENTS FROM RCT (end of hold phases)
    [frequency_Hz,Excitation_m_s,Response_m_s,~,indPROCESS] = ...
        estimateFourierCoefficientsFromSteppedSine(steps,time_s,...
        instantaneousFrequency_Hz,excitation_m_s,response_m_s,[],...
        options);
    %% FILTER STEPS ACCORDING TO QUALITY METRICS

    % Check if actual mean response level is within 2 % of target
    actualResponseLevel_mm = abs(Response_m_s(:,2)) ./ ...
        (2*pi*frequency_Hz)*1e3;
    isResponseLevelAsSpecified = abs(actualResponseLevel_mm - ...
        responseLevel_mm(iRespLev)) < 2e-2;

    % Check if standard deviation of instantaneous frequency is below 
    % 0.2 Hz, and that of the instantaneous phase lag is below 3.5°
    isFrequencyConstant = zeros(size(indPROCESS,1),1);
    isPhaselagConstant = zeros(size(indPROCESS,1),1);
    for iHold = 1:size(indPROCESS,1)
        stdFreq_Hz = ...
            std(instantaneousFrequency_Hz(indPROCESS(iHold,:)));
        stdPhase_degree = ...
            std(instantaneousPhase_degree(indPROCESS(iHold,:)));
        isFrequencyConstant(iHold) = stdFreq_Hz < 0.2;
        isPhaselagConstant(iHold) = stdPhase_degree < 3.5;
    end

    % All quality checks must be satisfied
    isValid = isResponseLevelAsSpecified & isFrequencyConstant & ...
        isPhaselagConstant;

    if any(isValid)
        %% ESTIMATE MODAL FREQUENCY AND DAMPING RATIO VIA 'CIRCLE FIT'
        % Sub-select frequency, response and excitation values from steps
        % satisfying quality metrics; the analysis is limited to the
        % fundamental harmonic
        Resp1H = Response_m_s(isValid,2);
        Exc1H = Excitation_m_s(isValid,2);
        freq_Hz = frequency_Hz(isValid);

        % Estimate frequency response function
        FRF = Resp1H./Exc1H;

        % Apply circle fit method
        [modalFrequency_Hz(iRespLev),modalDampingRatio_mean(iRespLev),...
            modalDampingRatio_std(iRespLev)] = ...
            circleFit(freq_Hz,FRF,illustrateCircleFits);

        % Report warning if not enough useful data available for circle fit
        if isnan(modalDampingRatio_std(iRespLev))
            warning(['Not enough useful data available to estimate ' ...
                'damping ratio via circle fit for response level '...
                num2str(responseLevel_mm(iRespLev)) ' mm according to '...
                'specified quality metrics.']);
        end

    else
        warning(['No useful data available for response level '...
            num2str(responseLevel_mm(iRespLev)) ' mm according to '...
            'specified quality metrics.']); 
    end
end
%% ILLUSTRATE MODAL PROPERTIES

% Modal frequency
figure(iRespLev+1); hold on; grid on;
plot(responseLevel_mm,modalFrequency_Hz,...
    'md-','LineWidth',2);
xlabel('displacement amplitude (panel center) in mm');
ylabel('\omega /(2\pi) in Hz');
set(gca,'xlim',[0 2],'ylim',[94 109]);

% Modal damping ratio
figure(iRespLev+2); hold on; grid on;
errorbar(responseLevel_mm,...
    modalDampingRatio_mean*100,modalDampingRatio_std*100,...
    'md-','LineWidth',2);
xlabel('displacement amplitude (panel center) in mm');
ylabel('damping ratio in %');
set(gca,'xlim',[0 2],'ylim',[0 1]);

% save('RCT_Config4_0_9mm.mat','freq_Hz','FRF',...
%     'Excitation_m_s','Response_m_s','instantaneousFrequency_Hz',...
%     'indPROCESS');
