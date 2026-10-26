%========================================================================
% DESCRIPTION: 
% This script post-proccesses data recorded during backbone tracking of a
% physical experiment under base excitation.
% 
% The provided data is from the TRChallenge test rig [1]. Base excitation
% was applied via shaker and slip table. The measured excitation is the 
% velocity at one clamping of the panel, the measured response is the
% velocity at the center of the panel relative to the clamping.
% 
% The following post-processing tasks are done:
%       1. The Fourier coefficients of all recorded responses and the
%         applied force are estimated for each backbone point.
%       2. The amplitude-dependent modal parameters are extracted.
%       3. The phase error at each backbone point is evaluated.
%
% This script shall serve as point of departure for post-processing data
% recorded during backbone tracking of your own test rig. If the above
% post-processing tasks are sufficient for your test objectives, ideally,
% you should only have to specify the 'timeStampSettings' and the
% 'timeStampData' in the section "Load test settings, measurement data, 
% and modal data of SUT". The modal data of the structure under test (SUT)
% is also needed for post-processing task 2; for variable names and data
% format, see the same section.
% 
% NOTE: The data recorded and saved in the real experiment is raw data. If 
% a signal delay time of a sensor should be compensated for, it must be
% done so also here during post-processing!
% Some further hints on options related to the estimation of the Fourier
% coefficients (adaptive filter vs. FFT) are given in the comments of the
% section "Estimate Fourier coefficients".
% 
% See README.md and DOC/BackboneTracking.md for further information.
% 
% REFERENCES
% [1] https://doi.org/10.18419/darus-3147
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
clearvars;
close all;
clc;
filepath = fileparts(mfilename("fullpath"));
cd(filepath);
addpath('../../SRC');
%% Load test settings, measurement data, and modal data of SUT
% As long as you did not modify the save commands in the other scripts, it
% is sufficient to specify the automatically generated time stamp here.
timeStampSettings = '2025_01_24-15_25';
timeStampData = '2025_01_24-15_26';
BASEFOLDER = 'DATA';
fileNameSettings = sprintf('settings_%s',timeStampSettings);
fileNameMeasurementData = sprintf('measurement_%s',timeStampData);
load([BASEFOLDER filesep fileNameSettings],'settings');
load([BASEFOLDER filesep fileNameMeasurementData],'measurement');
% Concatenate control sensor response and additional responses. Here it is
% presumed that all additional signals are responses. The type of response 
% (displacement vs. velocity vs. acceleration) is specified in
% 'settings.responseQuantity' in the initialization script.
response = zeros(size(measurement.response,1),...
    numel(settings.responseQuantity));
idxAddResp = 0;
for idxResp=1:numel(settings.responseQuantity)
    if idxResp==settings.indexSensorControl
        response(:,idxResp) = measurement.response;
    else
        idxAddResp = idxAddResp + 1;
        response(:,idxResp) = measurement.additionalSignals{idxAddResp};
    end
end
% NOTE: 'linearModalFrequency_Hz' and 'linearModalDampingRatio' must have
% the same number of elements. The element with index 
% 'settings.indexTargetMode' must correspond to the target mode.
% 'linearPhi' corresponds to the mass-normalized modal deflection shape. 
% Its columns are sorted in the same way as 'linearModalFrequency_Hz' and 
% 'linearModalDampingRatio'. Its rows must be sorted as the variable 
% 'response' defined above.
load([BASEFOLDER filesep 'linearSUTModes'], 'linearModalFrequency_Hz', ...
    'linearModalDampingRatio','linearPhi');

% Compensate measurement delay (hard-coded for delayed excitation). The
% delay of the LDV used to measure the excitaiton is 440µs.
numberOfSamplesDelay = round(440e-6 / settings.samplingTime_s);
% delayed signal
measurement.excitation = measurement.excitation(numberOfSamplesDelay+1:end);
% non-delayed signals
measurement.time_s = measurement.time_s(1:end-numberOfSamplesDelay);
response = response(1:end-numberOfSamplesDelay,:);
measurement.excitationFrequency_Hz = measurement.excitationFrequency_Hz(1:end-numberOfSamplesDelay);
measurement.excitationPhase_rad = measurement.excitationPhase_rad(1:end-numberOfSamplesDelay);
measurement.flagSettling = measurement.flagSettling(numberOfSamplesDelay+1:end);



%% Estimate Fourier coefficients

% Let user know what is currently being done. This comment applies to all
% other occurrences of 'fprintf' in this section.
fprintf('*INFO on post-processing:\n');
fprintf('- Post-processing measurement recorded at\n');
fprintf('%s\n', string(datetime(timeStampData,...
    'InputFormat','yyyy_MM_dd-HH_mm','format','MMMM d, yyyy - HH:mm')));

% Specify method for estimating Fourier coefficients
% ['adaptiveFilter'|'FFT'].
% ATTENTION: If you want to use 'FFT' here, you need to record a reasonable 
% number of periods during the hold phase. For this, specify 
% 'settings.recordedPeriods' in the initialization script.
postProcessingOptions.computeFourierCoefficients = 'adaptiveFilter'; 

% Adopt number of periods to be processed depending on method.
switch postProcessingOptions.computeFourierCoefficients 
    case 'adaptiveFilter'
       postProcessingOptions.target_number_of_last_periods_of_hold_phase_to_process = 1;
    case 'FFT'
        postProcessingOptions.target_number_of_last_periods_of_hold_phase_to_process = ...
            settings.recordedPeriods;
end
fprintf('- The last %d period(s) of each recording phase are evaluated.\n', ...
    postProcessingOptions.target_number_of_last_periods_of_hold_phase_to_process);

% If adaptive filtering is used for post-processing, adopt the settings of 
% the adaptive filter used during the test.
% ATTENTION: If you specify a lower cutoff frequency here, the resulting
% estimate of the Fourier coefficient does not necessarily stabilize during
% post-processing!
switch postProcessingOptions.computeFourierCoefficients 
    case 'adaptiveFilter' 
        postProcessingOptions.H = settings.H; 
        postProcessingOptions.omLP = settings.omLP;
        fprintf(['- %d harmonics are used for the adaptive filter. '...
            'The parameter omLP/omlin is %d.\n'], ...
            postProcessingOptions.H,postProcessingOptions.omLP / ...
            (2*pi*linearModalFrequency_Hz(settings.indexTargetMode)));
end

% Specify time increment for interpolating measured time series.
% NOTE: The function 'estimateFourierCoefficientsFromSteppedSine' relies on
% equidistant sampling. Non-equidistant samples typically occur if the data
% is from a virtual experiment simulated with variable time step
% integrator, or if DAQ hardware does not sample (precisely) equidistantly. 
% If non-equidistant sampling is detected, i.e., interpolation is
% necessary, but postProcessingOptions.samplingTimeNew is not specified, 
% the mean time increment of the time series is used.
% Here, we adopt the sampling time specified in the test settings.
postProcessingOptions.samplingTimeNew = settings.samplingTime_s;

% Call function 'estimateFourierCoefficientsFromSteppedSine' in SRC folder.
% This outputs complex Fourier coefficients from the given input in the
% settled time spans (indicated by 'flagSettling').
% ATTENTION: If you adjust the function call, please make sure that the
% units remain consistent concerning 'Hz', 'rad', and 's'.
[frequency_Hz,Excitation,Response,indPROCESS,~,~,~,~,instPhase,excFC,respFC] = ...
    estimateFourierCoefficientsFromSteppedSine(measurement.time_s,...
    measurement.excitationFrequency_Hz,...
    measurement.excitation,response,measurement.flagSettling,...
    postProcessingOptions,measurement.excitationPhase_rad);

% Determine order of highest estimated harmonic.
H = size(Excitation,2)-1;

% Convert excitation to displacement unit ('_m' refers to meter) in the
% frequency domain
switch settings.excitationQuantity
    case 'displacement'
        Excitation_m = Excitation;
    case 'velocity'
        Excitation_m = Excitation ./ (1i*2*pi*frequency_Hz*(0:H));
        % Zeroth harmonic cannot be retrieved.
        Excitation_m(:,1) = NaN(size(frequency_Hz));
    case 'acceleration'
        Excitation_m = Excitation ./ (1i*2*pi*frequency_Hz*(0:H)).^2;
        % Zeroth harmonic cannot be retrieved.
        Excitation_m(:,1) = NaN(size(frequency_Hz));
    otherwise
        error(['Unknowns excitaiton quantity specifier ' ...
            settings.excitationQuantity]);
end

% Convert all responses to displacement unit ('_m' refers to meter) in the
% frequency domain.
Response_m = zeros(size(Response));
for iResponse = 1:size(Response,3)
    switch settings.responseQuantity{iResponse}
        case 'displacement'
            Response_m(:,:,iResponse) = ...
                Response(:,:,iResponse);
        case 'velocity'
            Response_m(:,:,iResponse) = ...
                Response(:,:,iResponse)./...
                (1i*2*pi*frequency_Hz*(0:H));
            % Zeroth harmonic cannot be retrieved.
            Response_m(:,1,iResponse) = NaN(size(frequency_Hz)); 
        case 'acceleration'
            Response_m(:,:,iResponse) = ...
                Response(:,:,iResponse)./...
                (1i*2*pi*frequency_Hz*(0:H)).^2;
            % Zeroth harmonic cannot be retrieved.
            Response_m(:,1,iResponse) = NaN(size(frequency_Hz)); 
        otherwise
            error(['Unknowns response quantity specifier ' ...
                settings.responseQuantity{iResponse}]);
    end
end

%% Identify modal parameters and plot vs. response amplitude
% ATTENTION: Ensure consistent units are used (e.g. SI system). E.g., 
% displacement response is in m, linearPhi in 1/sqrt(kg), frequency in Hz,
% excitation in N.
weightsTrapezoidal = 1;
[modalFrequency_Hz,modalDampingRatio,Phi,modalAmplitude] = ...
    estimateModalPropertiesFromBackbone(frequency_Hz,Response_m,...
    Excitation_m, settings.indexSensorControl,linearPhi,...
    'base','modelFree', weightsTrapezoidal);

% Determine magnitude of fundamental harmonic of response and sensor used 
% for phase control.
responseLevelControlSensor_m = ...
    abs(Response_m(:,2,settings.indexSensorControl));

% Plot results.
figure; tiledlayout(2,1);
nexttile(1);
hold on;box on;grid on;
plot(responseLevelControlSensor_m*1000,modalFrequency_Hz,...
    'g.-','LineWidth',1,'DisplayName','nonlinear');
set(gca,'xlim',...
    [min(responseLevelControlSensor_m) max(responseLevelControlSensor_m)]*1000);
xlabel('fundamental harmonic response amplitude in mm');
ylabel('frequency in Hz');
% legend;
nexttile(2);
hold on;box on;grid on
plot(responseLevelControlSensor_m*1000,modalDampingRatio*100,...
    'g.-','LineWidth',1,'DisplayName','nonlinear');
set(gca,'xlim',...
    [min(responseLevelControlSensor_m) max(responseLevelControlSensor_m)]*1000);
xlabel('fundamental harmonic response amplitude in mm');
ylabel('modal damping ratio in %');

%% Compute and display phase error in evaluated time frame
% Specify target phase lag (hardcoded)
targetPhaseLag_rad = -pi/2;
% Compute instantaneous phase lag error (i.e. deviation from the target)
phaseLagError_rad = angle(respFC(:,2)) - angle(excFC(:,2)) - targetPhaseLag_rad;
% Wrap to +/-pi interval
phaseLagError_rad = atan2(sin(phaseLagError_rad),cos(phaseLagError_rad));
% Loop over backbone points and extract minimum, maximum, and mean of the
% phase lag error in the respective recording phase.
meanPhaseError_deg = zeros(size(indPROCESS,1),1);
minPhaseError_deg = zeros(size(indPROCESS,1),1);
maxPhaseError_deg = zeros(size(indPROCESS,1),1);
for indBackbonePoint = 1:size(indPROCESS,1)
    meanPhaseError_deg(indBackbonePoint) = ...
        rad2deg(mean(phaseLagError_rad(indPROCESS(indBackbonePoint,:))));
    minPhaseError_deg(indBackbonePoint) = ...
        rad2deg(min(phaseLagError_rad(indPROCESS(indBackbonePoint,:))));
    maxPhaseError_deg(indBackbonePoint) = ...
        rad2deg(max(phaseLagError_rad(indPROCESS(indBackbonePoint,:))));
end

% Visualize error for each backbone point. Show the error range only over
% the recording period (where settling/convergence has been determined). 
negErrorBar = meanPhaseError_deg - minPhaseError_deg;
posErrorBar = maxPhaseError_deg - meanPhaseError_deg;

figure; hold on; box on;
idxUp = find(diff(settings.excAmp)>=0);
idxDn = find(diff(settings.excAmp)<0);
errorbar(responseLevelControlSensor_m(idxUp)*1000,meanPhaseError_deg(idxUp), ...
    negErrorBar(idxUp),posErrorBar(idxUp),'g.','LineWidth',1, 'DisplayName', 'Ramp Up');
errorbar(responseLevelControlSensor_m(idxDn)*1000,meanPhaseError_deg(idxDn), ...
    posErrorBar(idxDn),negErrorBar(idxDn),'r.','LineWidth',1, 'DisplayName', 'Ramp Down');
xlabel('fundamental harmonic response amplitude in mm');
ylabel('phase control error in °');
legend();