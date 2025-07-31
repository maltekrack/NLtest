%========================================================================
% DESCRIPTION: 
% This script does a virtual backbone test using a mathematical model of 
% a cantilevered beam with an elastic dry friction element, inspired by 
% the RubBeR test rig [Scheel.2020].
% 
% This version of the script is deliberately kept simple and short, among 
% others, by using default settings where possible. Still, you may use this
% script to study the effect of customized settings. Specifically, you may 
% want to vary the following parameters to get a feeling of how these
% affect the performance of the Backbone Tracker:
%       - Reduce the ramp times (fields firstRampTime_s, rampTime_s of 
%         settings structure) from their defaults.
%       - Loosen/tighten the tolerances (fields tolerancePhaseShift_degree,
%         toleranceRelativeAmplitude, periodsInTolerance ...).
%       - Switch from voltage to amplitude stepping (activating the
%         additional amplitude controller).
%       - Reduce/increase the number of excitation levels, and/or the
%         range.
% For poor settings, the test will not complete successfully. The most 
% important performance metrics of a successful test are (a) duration, (b)
% accuracy of extracted modal parameters. (a) is reported in the command
% line upon successful completion of the virtual test. (b) can be assessed
% by using the results obtained with the initial settings as reference, see
% HINTs in the last section.
% 
% NOTE: The plant model is loaded and cannot be modified, and an idealized 
% exciter with unity gain is considered in this script. If you wish to 
% modify the plant parameters and/or consider a non-ideal exciter, use the 
% advanced version of this script, 'VirtualRubBeRAdvanced.m' in the same 
% folder.
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
addpath('../../SRC');
%% Load virtual plant model 

load('virtualRubBeR.mat','virtualRubber',...
    'linearModalFrequency_Hz','linearModalDampingRatio','linearPhiEx',...
    'responseQuantity');

% Adopt responseQuantity for the drive point sensor, which is the only 
% response sensor in this script.
settings.responseQuantity = responseQuantity;
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
% hardware running the controller. A typical value is used
% here.
settings.H = 7;
%% Check settings (and add any unspecified ones by default values)
settings = checkSettingsBackboneTracker(settings,...
    linearModalFrequency_Hz,linearModalDampingRatio,linearPhiEx);
%% Select control gains in a theory-driven way
[settings.gainsPhaseController, settings.gainsAmplitudeController] = ...
    controllerDesignBackboneTracker(settings.omLP,...
    linearModalFrequency_Hz,linearModalDampingRatio,linearPhiEx);
% If you want to adjust the control gains, do it here.
% settings.gainsPhaseController.kp = ...
% settings.gainsPhaseController.ki = ...
% settings.gainsAmplitudeController.kp = ...
% settings.gainsAmplitudeController.ki = ...
%% Simulate virtual test
sim('modelRubBeR');
fprintf('The duration of the virtual test was %.2f s.\n', ...
    excitation.time(end));
%% Extract and illustrate modal parameters vs. response amplitude

% Determine index to end of hold phase ('flagSettling' goes from 1 to 0).
idxEndOfHoldPhase = find(diff(flagSettling.signals.values) == -1) + 1;

% Extract data from online estimation of modal parameters.
% NOTE: The online data is a poor man's estimate of the modal damping 
% ratio, which assumes that the modal deflection shape remains the same as 
% in the linear case. The advanced version of this script, 
% 'VirtualRubBeRAdvanced.m' in the same folder, illustrates the deficiency 
% of this estimate, and how a more accurate modal damping estimate can be 
% obtained.
responseAmplitude_m = squeeze(...
    NMA_drivePointAmplitude.signals.values(idxEndOfHoldPhase));
modalFrequency_Hz = squeeze(NMA_frequency_Hz.signals.values(...
    idxEndOfHoldPhase));
modalDampingRatio = squeeze(...
    NMA_modalDampingRatio.signals.values(idxEndOfHoldPhase));
% HINT: Uncomment the following lines to store the results obtained with 
% the present settings as reference. Then comment again before running with
% different settings.
% aref = responseAmplitude_m;
% omref = modalFrequency_Hz;
% Dref = modalDampingRatio;
% save('reference','aref','omref','Dref');

% Plot results.
figure; tiledlayout(2,1);
% HINT: Uncomment the following lines to load and plot the reference
% results stored earlier.
% load('reference','aref','omref','Dref');
% nexttile(1);
% plot(aref,omref,'ko--','DisplayName','reference');
% nexttile(2);
% plot(aref,Dref*100,'ko--','DisplayName','reference');
nexttile(1);
hold on;box on;
plot(responseAmplitude_m, ...
    ones(size(responseAmplitude_m))*linearModalFrequency_Hz,...
    'color',.5*[1 1 1],'LineWidth',2,'DisplayName','linear');
plot(responseAmplitude_m,modalFrequency_Hz,...
    'gx:','LineWidth',1,'DisplayName','nonlinear');
set(gca,'xlim',[min(responseAmplitude_m) max(responseAmplitude_m)]);
xlabel('response amplitude in m');
ylabel('modal frequency in Hz');
legend;
nexttile(2);
hold on;box on;
plot(responseAmplitude_m,...
    ones(size(responseAmplitude_m))*linearModalDampingRatio*100,...
    'color',.5*[1 1 1],'LineWidth',2,'DisplayName','linear');
plot(responseAmplitude_m,modalDampingRatio*100,'gx:',...
    'LineWidth',1,'DisplayName','nonlinear');
set(gca,'xlim',[min(responseAmplitude_m) max(responseAmplitude_m)]);
xlabel('response amplitude in m');
ylabel('modal damping ratio in %');