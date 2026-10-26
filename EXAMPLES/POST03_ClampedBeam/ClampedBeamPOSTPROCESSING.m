%========================================================================
% DESCRIPTION: 
% This script post-proccesses data recorded during backbone tracking of a
% physical experiment.
% 
% The provided data is from the doubly-clamped beam test rig under force 
% excitaiton (as used e.g in [Hippold.2024]). The only measured response is
% the velocity at the center of the beam. The example contains two data
% sets, obtianed with different settings of the Backbone Tracker:
%       1. Upwards and downwards tracking of the backbone using equidistant
%          voltage levels.
%       2. Upwards and downwards tracking of the backbone using response
%          amplitude control and equidistant amplitude levels.
% 
% The following post-processing tasks are done for both data sets:
%       1. The Fourier coefficients of the recorded response and the
%          applied force are estimated for each backbone point.
%       2. The amplitude-dependent modal frequency is extracted. Note that
%          since the response was measured on the beam and not at the drive
%          point, a quatification of amplitude-dependent modal damping is 
%          not possible.
%       3. Evaluation and comparison of the number of cycles required per
%          backbone point for both stepping variants.
% 
% See README.md and DOC/BackboneTracking.md for further information.
% 
% REFERENCES
% [Hippold.2024] P. Hippold, M. Scheel, L. Renson, M. Krack (2024): Robust 
%       and fast backbone tracking via phase-locked loops, MSSP. 
%       http://doi.org/10.1016/j.ymssp.2024.111670
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

%% Load measurement data and modal data of SUT
% File names of the two data sets are specified here.
BASEFOLDER = 'DATA';
fileNameMeasurementData = { ...
    'measurement_steppedVoltage', ...
    'measurement_steppedAmplitude'};

% Load linear modal data. The lienar modal frequency serves as a reference
% for the adaptive filter cutoff frequency and is used to normalize the 
% amplitude-dependent modal frequency.
load([BASEFOLDER filesep 'linearModes_ClampedBeam'], 'linearModalFrequency_Hz', ...
    'linearModalDampingRatio','linearPhi');

% Specify method for estimating Fourier coefficients
% ATTENTION: The recorded measurement data only includes one settled period
% per backbone point which is not sufficient for FFT-based post-processing.
% Therefore, the 'adaptiveFilter' option has to be used.
postProcessingOptions.computeFourierCoefficients = 'adaptiveFilter';
postProcessingOptions.target_number_of_last_periods_of_hold_phase_to_process = 1;

% Let user know what is currently being done. This comment applies to all
% other occurrences of 'fprintf' in this section.
fprintf('*INFO on post-processing:\n');
fprintf('- The last %d period(s) of each recording phase are evaluated.\n', ...
    postProcessingOptions.target_number_of_last_periods_of_hold_phase_to_process);

% Specify adaptive filter cutoff frequency (as used during the tests).
postProcessingOptions.omLP = linearModalFrequency_Hz*2*pi/20;
fprintf('- The parameter omLP/omlin is %d.\n', ...
    postProcessingOptions.omLP / (2*pi*linearModalFrequency_Hz));

%% Loop over measurement data sets
backboneTests = cell(size(fileNameMeasurementData));
for RUN = 1:length(fileNameMeasurementData)
    % Load measurement data
    load([BASEFOLDER filesep fileNameMeasurementData{RUN}],'measurement');

    %% Estimate Fourier coefficients
    fprintf('- Post-processing measurement %s\n',fileNameMeasurementData{RUN});
    
    % Call function 'estimateFourierCoefficientsFromSteppedSine' in SRC folder.
    % This outputs complex Fourier coefficients from the given input in the
    % settled time spans (indicated by 'flagSettling').
    % ATTENTION: If you adjust the function call, please make sure that the
    % units remain consistent concerning 'Hz', 'rad', and 's'.
    [frequency_Hz,Excitation,Response,indPROCESS,~,~,~,~,intExcitationPhase_rad] = ...
        estimateFourierCoefficientsFromSteppedSine(measurement.time_s,...
        measurement.excitationFrequency_Hz,...
        measurement.excitation,measurement.response,measurement.flagSettling,...
        postProcessingOptions,measurement.excitationPhase_rad);
    
    % Determine order of highest estimated harmonic.
    H = size(Excitation,2)-1;
    
    % Convert the response to displacement unit ('_m' refers to meter) in 
    % the frequency domain. Recall that the response was measured in terms 
    % of velocity.
    Response_m = Response./(1i*2*pi*frequency_Hz*(0:H));

    %% Evaluate test duration
    % Here, we count the total number of cycles required per backbone
    % point, including the ramp and hold phase.
    fprintf('Number of cycles per backbone point:\n')
    cycleCounter = zeros(size(frequency_Hz));
    % Loop over backbone points
    for idxBackbonePoint = 1:length(cycleCounter)
        if idxBackbonePoint == 1
            % The beginning time frame of the first backbone point is the 
            % beginning of the of the recording. 
            tauStart = intExcitationPhase_rad(1);
        else
            % The beginning of the time frame of each other backbone
            % point is the end of the recording phase of the previous
            % backbone point.
            tauStart = intExcitationPhase_rad(find(indPROCESS(idxBackbonePoint-1,:),1,'last'));
        end
        % The end of the time frame of each backbone point is the end of
        % therespoective recording phase.
        tauEnd = intExcitationPhase_rad(find(indPROCESS(idxBackbonePoint,:),1,'last'));
        % Compute and display number of cycles for the current point.
        cycleCounter(idxBackbonePoint) = round((tauEnd - tauStart)/(2*pi));
        fprintf('- Point %2d: %3d\n',idxBackbonePoint,cycleCounter(idxBackbonePoint));
    end
    % Average cycle count over the whole backbone
    meanCycles = mean(cycleCounter);
    fprintf('-----------------\nAverage: %3d\n',round(meanCycles));

    %% Store results
    % The estimeated frequency for each backbone point is the corresponding 
    % amplitude-dependent modal frequency.
    results = struct('frequency_Hz',frequency_Hz,...
        'Excitation',Excitation,...
        'Response_m',Response_m);
    backboneTests{RUN} = results;
end

%% Plot backbone
beamThickness_m = 0.8e-3; % for amplitude normalization

figure;
hold on;box on;grid on;
plot(abs(backboneTests{1}.Response_m(:,2))/beamThickness_m,backboneTests{1}.frequency_Hz/linearModalFrequency_Hz,...
    'bx-','LineWidth',1,'DisplayName','voltage levels');
plot(abs(backboneTests{2}.Response_m(:,2))/beamThickness_m,backboneTests{2}.frequency_Hz/linearModalFrequency_Hz,...
    'r.','LineWidth',1,'DisplayName','amplitude control');
xlabel('normalized amplitude');
ylabel('normalized modal frequency');
legend('Location','northwest');
