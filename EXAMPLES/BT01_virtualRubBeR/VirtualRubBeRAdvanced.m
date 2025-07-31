%========================================================================
% DESCRIPTION: 
% This script does a virtual backbone test using a mathematical model of 
% a cantilevered beam with an elastic dry friction element, inspired by 
% the RubBeR test rig [Scheel.2020].
% 
% The given example configuration is instrumented with 7 response 
% sensors, and is driven by a Brüel&Kjaer Type 4809 shaker applied via 
% stinger at the beam's tip, which is collocated with 7th sensor 
% location (cf. Fig. 5 in [Scheel.2020]).
% 
% In contrast to the simple version in 'VirtualRubBeR.m', this script
% features:
%   - An adjustable plant model
%   - A non-ideal exciter described by a common electro-mechanical model
%   - Multiple response sensors
%   - Illustration of the use of some custom settings
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
% [Scheel.2020]	M. Scheel, T. Weigele, M. Krack (2020): Challenging an 
%       experimental nonlinear modal analysis method with a new strongly 
%       friction-damped structure, Journal of Sound and Vibration. 
%       http://doi.org/10.1016/j.jsv.2020.115580
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
[virtualRubber, responseQuantity, idxDrivePoint, ...
    LMA_SUT, LMA_plant, exciterMass_kg, ...
    a_NM, Resp1_NM, om_NM, del_NM] = ...
    defineRubberAdvancedAndDoNumericalNMA(settings.indexTargetMode);

% Adopt responseQuantity and drive point index
% If you wish to vary the number of sensors, their response quantitiy
% and/or the location of the drive point, you may do so in the section
% "Specify instrumentation" of the function
% "defineRubberAdvancedAndDoNumericalNMA".
settings.responseQuantity = responseQuantity;
settings.indexSensorDrivePoint = idxDrivePoint;

% Extract parameters of target mode.
linearModalFrequency_Hz = LMA_SUT.linearModalFrequency_Hz(idxTargetMode);
linearPlantFrequency_Hz = LMA_plant.linearPlantFrequency_Hz(idxTargetMode);
linearModalDampingRatio = LMA_SUT.linearModalDampingRatio(idxTargetMode);
linearPlantDampingRatio = LMA_plant.linearPlantDampingRatio(idxTargetMode);
linearPhiEx = LMA_SUT.linearPhi(idxDrivePoint,idxTargetMode);
linearPlantPhiEx = LMA_plant.linearPlantPhi(idxDrivePoint,idxTargetMode);

%% Specify backbone stepping

% Test type ['steppedVoltage'|'steppedAmplitude']
settings.testType = 'steppedVoltage';

% Lowest and highest excitation level
switch settings.testType 
    case 'steppedVoltage'
        % For this test type, levels are voltage amplitudes in V. Since an
        % ideal exciter is with unity gain is used, this equals the force
        % level at the drive point in N.
        lowestLevel = 0.05;
        highestLevel = 5;
    case 'steppedAmplitude'
        % For this test type, levels are displacement amplitudes at the
        % drive point in m.
        lowestLevel = 3.5e-6;
        highestLevel = 1e-4;
end

% Number of excitation levels
numberOfLevels = 10;

% Excitation profile (equidistant here)
settings.levels = linspace(lowestLevel,highestLevel,numberOfLevels);

%% Specify adaptive filter parameters
% NOTE: Since all signals are noise free in this virtual test, the 
% cutoff frequency can be varied in a wide range. A typical value is used
% here.
settings.omLP = linearModalFrequency_Hz*2*pi / 10;
% NOTE: In experimental practice, the harmonic order can be limited by the
% hardware running the controller. Here, the same value as for the Harmonic
% Balance compuutation of the numerical reference is used.
settings.H = 7;

%% Check settings (and add any unspecified ones by default values)
settings = checkSettingsBackboneTracker(settings, linearPlantFrequency_Hz, ...
    linearPlantDampingRatio, linearPlantPhiEx, exciterMass_kg, ...
    linearModalFrequency_Hz, linearPhiEx);
%% Select control gains in a theory-driven way
[settings.gainsPhaseController, settings.gainsAmplitudeController] = ...
    controllerDesignBackboneTracker(settings.omLP,...
    linearModalFrequency_Hz, linearModalDampingRatio, linearPhiEx, ...
    linearPlantFrequency_Hz, linearPlantDampingRatio, linearPlantPhiEx, ...
    exciterMass_kg);
% If you want to adjust the control gains, do it here.
% settings.gainsPhaseController.kp = ...
% settings.gainsPhaseController.ki = ...
% settings.gainsAmplitudeController.kp = ...
% settings.gainsAmplitudeController.ki = ...
%% Simulate virtual test
fprintf('---------------------------------------------------\n');
fprintf('Simulation of experiment started\n');
tic;
sim('modelRubBeRAdvanced');
simulationComputationalTime_s = toc;
fprintf('Simulation of experiment succeeded\n')
fprintf('The computational time of the simulation %.2f s.\n',...
    simulationComputationalTime_s);
fprintf('---------------------------------------------------\n');
fprintf('The duration of the virtual experiment is %.2f s.\n',...
    excitation.time(end));

% Store data in variables identical (or at least similar) to those obtained
% from a real test.
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

% Determine order of highest estimated harmonic.
H = size(Excitation,2)-1;

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
            error(['Unknowns response quantity specifier ' ...
                responseQuantity{iResponse}]);
    end
end

%% Identify modal parameters and plot vs. response amplitude

% Use poor man's estimate of modal damping ratio, which assumes that the 
% modal deflection shape remains the same as in the linear case.
idxEndOfHoldPhase = find(diff(flagSettling.signals.values) == -1) + 1;
responseAmplitude_m_ONLINE = squeeze(...
    NMA_drivePointAmplitude.signals.values(idxEndOfHoldPhase));
modalFrequency_Hz_ONLINE = squeeze(NMA_frequency_Hz.signals.values(...
    idxEndOfHoldPhase));
modalDampingRatio_ONLINE = squeeze(...
    NMA_modalDampingRatio.signals.values(idxEndOfHoldPhase));

% Use sophisticated estimate based on multiple response sensors, and in
% general accounting for a potential change of the modal deflection shape.
% ATTENTION: Ensure consistent units are used (e.g. SI system). E.g., 
% displacement response is in m, linearPhi in 1/sqrt(kg), frequency in Hz,
% excitation in N.
[modalFrequency_Hz,modalDampingRatio,Phi,modalAmplitude] = ...
    estimateModalPropertiesFromBackbone(frequency_Hz,Response_m,...
    Excitation,'force',idxDrivePoint,LMA_SUT.linearPhi);

% Determine magnitude of fundamental harmonic of drive point response.
responseLevelDrivePoint_m = abs(Response_m(:,2,idxDrivePoint));

% Plot results.
figure; tiledlayout(2,1);
nexttile(1);
hold on;box on;grid on;
plot(abs(Resp1_NM(idxDrivePoint,:)),om_NM/(2*pi),'k-',...
    'LineWidth',1,'DisplayName','reference (EPMC)');
plot(responseAmplitude_m_ONLINE,modalFrequency_Hz_ONLINE,...
    'bo','LineWidth',1,'DisplayName','backbone test (online)');
plot(responseLevelDrivePoint_m,modalFrequency_Hz,...
    'gx','LineWidth',1,'DisplayName','backbone test (post-processed)');
set(gca,'xlim',...
    [min(responseLevelDrivePoint_m) max(responseLevelDrivePoint_m)],...
    'xscale','log');
xlabel('response amplitude in m');
ylabel('modal frequency in Hz');
legend;
nexttile(2);
hold on;box on;grid on;
plot(abs(Resp1_NM(idxDrivePoint,:)),del_NM*1e2,'k-',...
    'LineWidth',1,'DisplayName','reference (EPMC)');
plot(responseAmplitude_m_ONLINE,modalDampingRatio_ONLINE*1e2,...
    'bo','LineWidth',1,'DisplayName','backbone test (online)');
plot(responseLevelDrivePoint_m,modalDampingRatio*1e2,...
    'gx','DisplayName','backbone test (post-processed)');
set(gca,'xlim',...
    [min(responseLevelDrivePoint_m) max(responseLevelDrivePoint_m)],...
    'xscale','log');
xlabel('response amplitude in m');
ylabel('modal damping ratio in %');