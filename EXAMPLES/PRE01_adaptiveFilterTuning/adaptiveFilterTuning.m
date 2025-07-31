%========================================================================
% DESCRIPTION: 
% This script illustrates the systematic tuning of the cutoff frequency of 
% the adaptive filter. As recommended in [Hippold.2024], data from an
% open-loop test under sinusoidal voltage with a frequency equal to the
% linear natural frequency of the target mode is used. Filters with
% different cutoff frequencies are applied. The resulting phase
% fluctuations are illustrated together with the specified tolerance
% criterion, similar to Fig. 10 of [Hippold.2024].
% 
% See DOC/BackboneTracking.md for the role of adaptive filters in general 
% and the cutoff frequency in particular. Specifically, the section 
% "Tuning the adaptive filter based on provided real measurement data"
% refers to this example, and the section "Tune the adaptive filter"
% explains how to run an open-loop test with your test rig.
% 
% To apply this script to your own data, some adjustments are needed. In
% particular, you need to specify the measurement data file. 
% 
% The cutoff frequencies to be tested are specified relative to the angular 
% excitation frequency (equal to natural angular frequency of the target 
% mode in [Hippold.2024]). Typical values are in the range from 1/100 to 
% 1/10. Higher values are expected to lead to poor control robustness. 
% If you seem to need lower values to reach acceptable phase distortion, 
% this might indicate relatively high noise present in your data. To avoid 
% unacceptably long test duration, consider starting the backbone test at 
% a higher exictation level and/or adjusting the instrumentation to reduce
% noise. If you can double the cutoff frequency, you can expect roughly 
% half the test duration.
% 
% On the tolerance criterion for the phase distortion produced by the
% adaptive filter: The number of periods for which the phase distortion is 
% to be below a given tolerance, along with that tolerance, should match 
% the parameters of the settling detection specified in the initialization 
% of the backbone test. More specifically, 'periodsInTolerance' specified 
% in this script should be >= 'settings.periodsInTolerance', and 
% 'phaseTolerance_deg' is recommended to be 1/2 of 
% 'settings.tolerancePhaseShift_degree' in [Hippold.2024]. The tolerance 
% criterion may have to be adjusted for your specific test.
% 
% It is crucial that transients have decayed before assessing the level of 
% phase distortion produced by the adaptive filter. This applies to the 
% transient of the plant, the transient of the adaptive filters, and
% potential transients of sensors and signal conditioners. One can estimate 
% the transient of the plant analogous to Eq. (44) in [Hippold.2024]. 
% The 0.1 % settling time of an adaptive filter with cutoff frequency 
% omLP is -log(0.001)/omLP. In experimental practice, the transients of
% other devices in the measurement chain may dominate. This is actually the
% case for the provided measurement, which is why a much larger time is
% recorded and not used for assessing the phase distortion. To help you
% decide whether or not transients have decayed, the recorded time series
% is illustrated along with its Fourier decomposition obtained with the
% respective adaptive filter. If you find that the transients have not
% decayed in the time frame highighted in green color, specify a higher
% 'nPeriodsSettling' (you might have to repeat the open-loop test to obtain
% a sufficiently long recording).
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
clear;
clc;
close all;
filepath = fileparts(mfilename("fullpath"));
cd(filepath);
addpath('../../SRC/');
%% Specify trial adaptive filters and phase tolerance criterion

% Trial cutoff frequencies relative to the excitation frequency
% HINT: Extend the example range and/or refine, keeping in mind what was 
% expained above in the header.
omLP_relative = [1/20, 1/10, 1/8];

% Hrmonic order of the filter
% NOTE: The open-loop test is at low excitation level. Hence, no 
% substantial higher harmonics are expected. Still, it is recommended to
% specify the same value here as the one to be used later during the
% backbone test.
H = 10;

% Tolerance of phase in °
% NOTE: In [Hippold.2024] it is recommended to set the phase fluctuation 
% tolerance to 50 % of the tolerance used in settling detection which 
% results in +/-0.5° for backbone tracking since there the default 
% tolerance for settling detection is +/-1°.
phaseTolerance_deg = .5; 

% Number of periods to be within tolerance band
periodsInTolerance = 20;

% Store adaptive filter settings in options structure compatible with
% function 'estimateFourierCoefficientsFromSteppedSine'. The individual
% cutoff frequency is specified later in a loop.
options = struct('computeFourierCoefficients', 'adaptiveFilter', ...
    'H', H, ...
    'target_number_of_last_periods_of_hold_phase_to_process', ...
    periodsInTolerance);

%% Load measurement data and specify initial part to be ignored
timeStampData = '2025_07_16-13_35';
BASEFOLDER = 'DATA';
fileNameMeasurementData = sprintf('measurement_%s',timeStampData);
load([BASEFOLDER filesep fileNameMeasurementData],'measurement');

% Specify number of initial periods to be ignored before checking the phase
% fluctuation tolerance.
nPeriodsSettling = 500;

%% Check user input, select required section of time series

% Check if the transient of the slowest adaptive filter is exected to have
% decayed in the ignored initial time frame.
nPeriodsTransientAF = ceil(-log(0.001)/(min(omLP_relative)*2*pi));
if nPeriodsSettling < nPeriodsTransientAF
    warning(['The chosen number of settling periods is smaller than the ' ...
        'estimated 0.1\% settling time of the adaptive filter. ' ...
        'Consider increasing nPeriodsSettling to at least %d.'], ...
        nPeriodsTransientAF);
end

% Check for constant excitation frequency.
if max(abs(diff(measurement.excitationFrequency_Hz))) ~= 0
    warning(['Excitation frequency is not constant. Use test with ' ...
        'constant excitation frequency for adaptive filter tuning.']);
end

% Proceed with initial excitation frequency.
nominalFrequency_Hz = measurement.excitationFrequency_Hz(1);
options.nominalFreq = nominalFrequency_Hz;

% Determine required section of time series.
timeToProcess = (nPeriodsSettling+periodsInTolerance)/nominalFrequency_Hz;
idxEnd = find(measurement.time_s - measurement.time_s(1) >= ...
    timeToProcess, 1, 'first');
if isempty(idxEnd)
    error(['Open-loop test recording too short for specified ' ...
        'analysis. Consider reducing nPeriodsSettling or extend ' ...
        'test duration']);
end

% Truncate time series to required section.
time_s                  = measurement.time_s(1:idxEnd+1);
excitation              = measurement.excitation(1:idxEnd+1);
response                = measurement.response(1:idxEnd+1);
excitationPhase_rad     = measurement.excitationPhase_rad(1:idxEnd+1);
excitationFrequency_Hz  = measurement.excitationFrequency_Hz(1:idxEnd+1);

% Determine section in which the phase fluctuation criterion is to be
% checked.
excitationPeriod = ( excitationPhase_rad(1:idxEnd) - ...
    excitationPhase_rad(1) )/(2*pi);
flagSettling = false(size(time_s)); 
flagSettling( excitationPeriod >= nPeriodsSettling ) = true;

%% Apply adaptive filters with different cutoff frequencies
for ii = 1:length(omLP_relative)

    % Set cutoff frequency.
    options.omLP = omLP_relative(ii) * nominalFrequency_Hz*2*pi;
    fprintf('Applying adaptive filter with relative cutoff frequency %.2e...\n', ...
        omLP_relative(ii));

    % Estimate Fourier coefficients with current adaptive filter options.
    [~,~,~,idxToBeInTolerance,~, ~, ...
        excitationInterpolated, responseInterpolated, ...
        excitationPhaseInterpolated_rad, ...
        FourierCoefficientsExcitation, FourierCoefficientsResponse] = ...
        estimateFourierCoefficientsFromSteppedSine( time_s, ...
        excitationFrequency_Hz,excitation,response,flagSettling,options, ...
        excitationPhase_rad);

    % Synthesize signals estimated by adaptive filter.
    signalExcitation = real( sum(exp(repmat(1i*(0:H), ...
        length(excitationPhaseInterpolated_rad),1).*...
        excitationPhaseInterpolated_rad) .* ...
        FourierCoefficientsExcitation,2));
    signalResponse = real( sum(exp(repmat(1i*(0:H), ...
        length(excitationPhaseInterpolated_rad),1).*...
        excitationPhaseInterpolated_rad) .* ...
        FourierCoefficientsResponse,2));

    % Convert from instantaneous phase to excitation periods for plotting,
    % starting at zero.
    excitationPeriod = ( excitationPhaseInterpolated_rad - ...
        excitationPhaseInterpolated_rad(1) )/(2*pi);

    % Plot measured time series vs. estimated signals.
    figure;
    tlSignals = tiledlayout(2,1);
    title(tlSignals,sprintf('\\omega_{LP} = %.2e \\Omega',...
        omLP_relative(ii)));
    nexttile(1);
    hold on;
    plot(excitationPeriod,excitationInterpolated,'k-','LineWidth',1, ...
        'DisplayName','measured');
    plot(excitationPeriod(idxToBeInTolerance),signalExcitation(idxToBeInTolerance), ...
        'g-','LineWidth',1,'DisplayName','estimated signal (assessed)');
    xlabel('excitaiton periods');
    ylabel('excitation');
    box on;
    legend('location','nw');
    nexttile(2);
    hold on;
    plot(excitationPeriod,responseInterpolated,'k-','LineWidth',1, ...
        'DisplayName','measured');
    plot(excitationPeriod(idxToBeInTolerance),signalResponse(idxToBeInTolerance), ...
        'g-','LineWidth',1,'DisplayName','estimated signal (assessed)');
    xlabel('excitation periods');
    ylabel('response');
    box on;

    % Compute phase lag.
    % HINT: If you wish to determine the dominant source for phase
    % distortion (excitation vs. response), simply set
    %       phaseLag_rad = unwrap(excitationPhaseLag_rad);
    %               or
    %       phaseLag_rad = unwrap(responsePhaseLag_rad);
    % respectively.
    excitationPhaseLag_rad = angle(...
        FourierCoefficientsExcitation(idxToBeInTolerance,2));
    responsePhaseLag_rad = angle(...
        FourierCoefficientsResponse(idxToBeInTolerance,2));
    phaseLag_rad = unwrap(responsePhaseLag_rad - excitationPhaseLag_rad);
    
    % Compute mean phase lag to define tolerance band.
    meanPhaseLag_rad = mean(phaseLag_rad);
    
    % Plot phase lag and tolerance band.
    figure;
    title(sprintf('\\omega_{LP} = %.2e \\Omega',omLP_relative(ii)));
    hold on;
    plot(excitationPeriod(idxToBeInTolerance),phaseLag_rad*180/pi,'k-', ...
        'DisplayName','estimated');
    plot(excitationPeriod(idxToBeInTolerance),...
        ones(sum(idxToBeInTolerance),1) * ...
        meanPhaseLag_rad*180/pi,'b-','DisplayName','mean');
    plot(excitationPeriod(idxToBeInTolerance),...
        ones(sum(idxToBeInTolerance),1) * ...
        (meanPhaseLag_rad*180/pi + phaseTolerance_deg),...
        'r-','DisplayName','tolerance');
    plot(excitationPeriod(idxToBeInTolerance),...
        ones(sum(idxToBeInTolerance),1) * ...
        (meanPhaseLag_rad*180/pi - phaseTolerance_deg),...
        'r-','HandleVisibility','off');
    xlabel('excitation periods');
    ylabel('phase shift in degree');
    box on;
    legend;

    fprintf('...done\n');
end