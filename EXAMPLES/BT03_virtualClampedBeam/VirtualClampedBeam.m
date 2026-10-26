%========================================================================
% DESCRIPTION: 
% This script does a virtual backbone test using a mathematical model of 
% a doubly-camped beam exhibiting pronounced geometric nonlinearity 
% inspired by the test rig used in [Mueller.2022].
% 
% The given example configuration is instrumented with 5 response 
% sensors, and is base-excited by a Brüel&Kjaer Type 4808 shaker via 
% a rigid frame. The response sensor at midspan of the beam is used for 
% control.
% 
% As numerical reference, the modal parameters are computed directly
% from the mathematical model of the structure under test in accordance
% with the Extended Periodic Motion Concept. To this end, Harmonic Balance 
% and numerical path continuation are used. Further, it is shown that the
% poor man's estimate of the modal damping ratio obtained online during
% backbone tracking is inaccurate, and that the data acquired from multiple
% response sensors can be used to obtain an accurate result via the
% function 'estimateModalPropertiesFromBackbone'.
% 
% To set up the plant model and to compute the reference modal parameters,
% the tool NLvib is used.
%
% See DOC/BackboneTracking.md for further information.
% 
% REFERENCES
% [Mueller.2022] F. Mueller, L. Woiwode, J. Gross, M. Scheel, M. Krack 
% (2022): Nonlinear damping quantification from phase-resonant tests under
% base excitation, Mechanical Systems and Signal Processing. 
% https://doi.org/10.1016/j.ymssp.2022.109170
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
clear;
clc;
close all;
filepath = fileparts(mfilename("fullpath"));
cd(filepath);
addpath(genpath(['..' filesep '..' filesep 'SRC']));
requireNLvib;
%% Specify sampling time in s
% NOTE: In the virtual test, this corresponds to the fixed time step of the
% simulation.
settings.samplingTime_s = 1e-4;
%% Define plant model, compute numerical reference using EPMC

% Specify index of mode whose backbone is to be tracked.
% NOTE: Sorting must be consistent with linear modal data loaded below!
idxTargetMode = 1;
settings.indexTargetMode = idxTargetMode;

% Define plant model using separate function
[virtualBeam, excitationQuantity, responseQuantity, indexSensorControl, ...
    sensorLocations, ...
    LMA_SUT, LMA_plant, exciterMass_kg, beamMass_kg, ...
    NM] = ...
    defineClampedBeamAndDoNumericalNMA(settings.indexTargetMode);

% Adopt responseQuantity and drive point index
% If you wish to vary the number of sensors, their response quantitiy
% and/or the location of the drive point, you may do so in the section
% "Specify instrumentation" of the function
% "defineClampedBeamAndDoNumericalNMA".
settings.excitationQuantity = excitationQuantity;
settings.responseQuantity = responseQuantity;
settings.indexSensorControl = indexSensorControl;

% Extract parameters of target mode.
settings.linearModalFrequency_Hz = LMA_SUT.linearModalFrequency_Hz(idxTargetMode);
settings.linearPlantFrequency_Hz = LMA_plant.linearPlantFrequency_Hz(idxTargetMode);
settings.linearModalDampingRatio = LMA_SUT.linearModalDampingRatio(idxTargetMode);
settings.linearPlantDampingRatio = LMA_plant.linearPlantDampingRatio(idxTargetMode);
settings.linearPhiC = LMA_SUT.linearPhi(indexSensorControl,idxTargetMode);
settings.linearPlantPhiC = LMA_plant.linearPlantPhi(indexSensorControl,idxTargetMode);

% Store masses of exciter and SUT in settings structure.
settings.exciterMass_kg = exciterMass_kg;
settings.SUTMass_kg = beamMass_kg;

%% Specify backbone stepping

% Test type ['steppedVoltage'|'steppedAmplitude']
settings.testType = 'steppedVoltage';

% Lowest and highest excitation level
switch settings.testType 
    case 'steppedVoltage'
        % For this test type, levels are voltage amplitudes in V.
        lowestLevel = 0.8;
        highestLevel = 8; 
    case 'steppedAmplitude'
        % For this test type, levels are displacement amplitudes at the
        % drive point in m.
        lowestLevel = 1e-4;
        highestLevel = 1e-3;
end

% Number of excitation levels
numberOfLevels = 10;

% Excitation profile (equidistant here)
settings.levels = linspace(lowestLevel,highestLevel,numberOfLevels);

%% Settling detection
settings.tolerancePhaseShift_degree = .5; 
settings.periodsInTolerance = 50;

%% Specify adaptive filter parameters
% NOTE: Since all signals are noise free in this virtual test, the 
% cutoff frequency can be varied in a wide range. A typical value is used
% here.
settings.omLP = settings.linearModalFrequency_Hz*2*pi / 10;
% NOTE: In experimental practice, the harmonic order can be limited by the
% hardware running the controller. Here, the same value as for the Harmonic
% Balance compuutation of the numerical reference is used.
settings.H = 7;

%% Check settings (and add any unspecified ones by default values)
settings = checkSettingsBackboneTracker(settings);

%% Select control gains in a theory-driven way
[settings.gainsPhaseController, settings.gainsAmplitudeController] = ...
    controllerDesignBackboneTracker(settings);
% If you want to adjust the control gains, do it here.
% settings.gainsPhaseController.kp = 0;
% settings.gainsPhaseController.ki = 0;
% settings.gainsAmplitudeController.kp = 0;
% settings.gainsAmplitudeController.ki = 0;

%% Simulate virtual test
fprintf('---------------------------------------------------\n');
fprintf('Simulation of experiment started\n');
tic;
sim('modelClampedBeam');
simulationComputationalTime_s = toc;
fprintf('Simulation of experiment succeeded\n')
fprintf('The computational time of the simulation %.2f s.\n',...
    simulationComputationalTime_s);
fprintf('---------------------------------------------------\n');
fprintf('The duration of the virtual experiment is %.2f s.\n',...
    excitation.time(end));

% Store data in variables identical to those obtained from a real test.
measurement.time_s = excitation.time;
measurement.excitationFrequency_Hz = ...
    excitationFrequency_Hz.signals.values(:);
measurement.excitation = excitation.signals.values(:);
measurement.responses = measured_responses.signals.values;
measurement.flagSettling = squeeze(flagSettling.signals.values);
measurement.excitationPhase_rad = excitationPhase_rad.signals.values(:);
measurement.phaseError_deg = phaseError_deg.signals.values;
measurement.relativeAmplitudeError = relativeAmplitudeError.signals.values;

%% Estimate Fourier coefficients

% Specify options for function 'estimateFourierCoefficientsFromSteppedSine'
postProcessingOptions.computeFourierCoefficients = 'adaptiveFilter'; 
postProcessingOptions.target_number_of_last_periods_of_hold_phase_to_process = 1;
postProcessingOptions.H = settings.H;
postProcessingOptions.omLP = settings.omLP;

% Call function 'estimateFourierCoefficientsFromSteppedSine' in SRC folder.
% This outputs complex Fourier coefficients from the given input in the
% settled time spans (indicated by 'flagSettling').
% ATTENTION: If you adjust the function call, please make sure that the
% units remain consistent concerning 'Hz', 'rad', and 's'.
[frequency_Hz,Excitation,Response] = ...
    estimateFourierCoefficientsFromSteppedSine(measurement.time_s,...
    measurement.excitationFrequency_Hz,...
    measurement.excitation,measurement.responses,measurement.flagSettling,...
    postProcessingOptions,measurement.excitationPhase_rad);

% Determine order of highest estimated harmonic (required if order is not
% set by user in case of FFT processing).
H = postProcessingOptions.H;


% Convert excitation to displacement unit ('_m' refers to meter) in the
% frequency domain
switch excitationQuantity
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
        error(['Unknown excitation quantity specifier ' ...
            excitaitonQuantity]);
end


% Convert all responses to displacement unit ('_m' refers to meter) in the
% frequency domain.
Response_m = zeros(size(Response));
for iResponse = 1:size(Response,3)
    switch responseQuantity{iResponse}
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
            error(['Unknown response quantity specifier ' ...
                responseQuantity{iResponse}]);
    end
end

%% Identify modal parameters and plot vs. response amplitude

% Use poor man's estimate of modal damping ratio, which uses only one 
% response location for damping estimation.
idxEndOfHoldPhase = find(diff(flagSettling.signals.values) == -1) + 1;
responseAmplitude_m_ONLINE = squeeze(...
    NMA_referenceAmplitude.signals.values(idxEndOfHoldPhase));
modalFrequency_Hz_ONLINE = squeeze(NMA_frequency_Hz.signals.values(...
    idxEndOfHoldPhase));
modalDampingRatio_ONLINE = squeeze(...
    NMA_modalDampingRatio.signals.values(idxEndOfHoldPhase));

% Use sophisticated estimate based on multiple response sensors, and in
% general accounting for a potential change of the modal deflection shape.
% ATTENTION: Ensure consistent units are used (e.g. SI system). E.g., 
% displacement response is in m, linearPhi in 1/sqrt(kg), frequency in Hz,
% excitation in m.
weightsTrapezoidal = 1/2*(diff(sensorLocations(1:end-1)) + ...
    diff(sensorLocations(2:end)))';
[modalFrequency_Hz,modalDampingRatio,Phi,modalAmplitude] = ...
    estimateModalPropertiesFromBackbone(frequency_Hz,Response_m,...
    Excitation_m,indexSensorControl,LMA_SUT.linearPhi,'base','modelFree',...
    weightsTrapezoidal);

% Determine magnitude of fundamental harmonic of reference response.
responseLevelReferenceSensor_m = abs(Response_m(:,2,indexSensorControl));

% Plot results.
figure; tiledlayout(2,1);
nexttile(1);
hold on;box on;grid on;
plot(abs(NM.Resp1(indexSensorControl,:)),NM.frequency_Hz,'k-',...
    'LineWidth',1,'DisplayName','reference (EPMC)');
plot(responseAmplitude_m_ONLINE,modalFrequency_Hz_ONLINE,...
    'bo','LineWidth',1,'DisplayName','backbone test (online)');
plot(responseLevelReferenceSensor_m,modalFrequency_Hz,...
    'gx','LineWidth',1,'DisplayName','backbone test (post-processed)');
xlabel('response amplitude in m');
ylabel('modal frequency in Hz');
legend('location','nw');
nexttile(2);
hold on;box on;grid on;
plot(abs(NM.Resp1(indexSensorControl,:)),NM.modalDampingRatio*1e2,'k-',...
    'LineWidth',1,'DisplayName','reference (EPMC)');
plot(responseAmplitude_m_ONLINE,modalDampingRatio_ONLINE*1e2,...
    'bo','LineWidth',1,'DisplayName','backbone test (online)');
plot(responseLevelReferenceSensor_m,modalDampingRatio*1e2,...
    'gx','DisplayName','backbone test (post-processed)');
xlabel('response amplitude in m');
ylabel('modal damping ratio in %');