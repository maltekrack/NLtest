%========================================================================
% DESCRIPTION: 
% This script post-proccesses data recorded during backbone tracking of a
% physical experiment.
% 
% The provided data is from the RubBeR test rig [Scheel.2020], in a 
% configuration with 7 acceleration sensors, driven by a Brüel&Kjaer Type 
% 4809 shaker applied via stinger at ~ two-third of the beam's length 
% counted from the clamping, which is collocated with 5th sensor location 
% (cf. Fig. 5 in [Scheel.2020]).
% 
% The following post-processing tasks are done:
%       1. The Fourier coefficients of all recorded responses and the
%         applied force are estimated for each backbone point.
%       2. The amplitude-dependent modal parameters are extracted.
%       3. The contribtion 'E(m,h)' to the mechanical energy of every 
%         available linear mode 'm' and harmonic 'h' is computed; the 
%         most important ones are identified and plotted.
%       4. Using the extracted modal parameters, a frequency-response curve
%         is reconstructed using the Single Nonlinear Mode Theory.
%
% This script shall serve as point of departure for post-processing data
% recorded during backbone tracking of your own test rig. If the above
% post-processing tasks are sufficient for your test objectives, ideally,
% you should only have to specify the 'timeStampSettings' and the
% 'timeStampData' in the section "Load test settings, measurement data, 
% and modal data of SUT". The modal data of the structure under test (SUT)
% is also needed for post-processing tasks 2-4; for variable names and data
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
% [Hippold.2024] P. Hippold, M. Scheel, L. Renson, M. Krack (2024): Robust 
%       and fast backbone tracking via phase-locked loops, MSSP. 
%       http://doi.org/10.1016/j.ymssp.2024.111670
% [Scheel.2020]	M. Scheel, T. Weigele, M. Krack (2020): Challenging an 
%       experimental nonlinear modal analysis method with a new strongly 
%       friction-damped structure, Journal of Sound and Vibration. 
%       http://doi.org/10.1016/j.jsv.2020.115580
% [Schwarz.2020] S. Schwarz, L. Kohlmann, A. Hartung, J. Groß, M. Scheel, 
%       M. Krack (2020): Validation of a Turbine Blade Component Test With 
%       Frictional Contacts by Phase-Locked-Loop and Force-Controlled 
%       Measurements, Journal of Engineering for Gas Turbines and Power.
%       http://doi.org/10.1115/1.4044772
% [Woiwode.2024] L. Woiwode, M. Krack (2024): Experimentally uncovering  
%       isolasvia backbone tracking, Journal of Structural Dynamics.
%       http://doi.org/10.25518/2684-6500.180
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
timeStampSettings = '2025_07_16-13_41';
timeStampData = '2025_07_16-13_41';
BASEFOLDER = 'DATA';
fileNameSettings = sprintf('settings_%s',timeStampSettings);
fileNameMeasurementData = sprintf('measurement_%s',timeStampData);
load([BASEFOLDER filesep fileNameSettings],'settings');
load([BASEFOLDER filesep fileNameMeasurementData],'measurement');
% Concatenate drive point response and additional responses. Here it is
% presumed that all additional signals are responses. The type of response 
% (displacement vs. velocity vs. acceleration) is specified in
% 'settings.responseQuantity' in the initialization script.
response = zeros(size(measurement.response,1),...
    numel(settings.responseQuantity));
idxAddResp = 0;
for idxResp=1:numel(settings.responseQuantity)
    if idxResp==settings.indexSensorDrivePoint
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
load([BASEFOLDER filesep 'linearSUTmodes'], 'linearModalFrequency_Hz', ...
    'linearModalDampingRatio','linearPhi');
%% Estimate Fourier coefficients

% Let user know what is currently being done. This comment applies to all
% other occurrences of 'fprintf' in this section.
fprintf('*INFO on post-processing:\n');
fprintf('- Post-processing measurement recorded at\n');
fprintf('%s\n', string(datetime(timeStampData,...
    'InputFormat','yyyy_MM_dd-HH_mm','format','MMMM d, yyyy - HH:mm')));

% Check for response and/or excitation delay
if isfield(settings,'delayResponse') && settings.delayResponse~=0 || ...
        isfield(settings,'delayExcitation') && settings.delayExcitation~=0
    fprintf(['- Non-zero response/excitation delay specified. You need ' ...
        'to account for this yourself, i.e., there is no automatic ' ...
        'compensation implemented in the publicly released ' ...
        'post-processing script.']);
end

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
[frequency_Hz,Excitation,Response] = ...
    estimateFourierCoefficientsFromSteppedSine(measurement.time_s,...
    measurement.excitationFrequency_Hz,...
    measurement.excitation,response,measurement.flagSettling,...
    postProcessingOptions,measurement.excitationPhase_rad);

% Determine order of highest estimated harmonic.
H = size(Excitation,2)-1;

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
[modalFrequency_Hz,modalDampingRatio,Phi,modalAmplitude] = ...
    estimateModalPropertiesFromBackbone(frequency_Hz,Response_m,...
    Excitation,settings.indexSensorDrivePoint,linearPhi);

% Determine magnitude of fundamental harmonic of drive point response.
responseLevelDrivePoint_m = ...
    abs(Response_m(:,2,settings.indexSensorDrivePoint));

% Plot results.
figure; tiledlayout(2,1);
nexttile(1);
hold on;box on;grid on;
plot(responseLevelDrivePoint_m,modalFrequency_Hz,...
    'g-','LineWidth',1,'DisplayName','nonlinear');
plot([min(responseLevelDrivePoint_m) max(responseLevelDrivePoint_m)],...
    linearModalFrequency_Hz(settings.indexTargetMode)*[1 1],...
    'k-','DisplayName','linear');
set(gca,'xlim',...
    [min(responseLevelDrivePoint_m) max(responseLevelDrivePoint_m)]);
xlabel('fundamental harmonic drive point amplitude in m');
ylabel('frequency in Hz');
legend;
nexttile(2);
hold on;box on;grid on
plot(responseLevelDrivePoint_m,modalDampingRatio*100,...
    'g-','DisplayName','nonlinear');
plot([min(responseLevelDrivePoint_m) max(responseLevelDrivePoint_m)],...
    100*linearModalDampingRatio(settings.indexTargetMode)*[1 1],...
    'k-','DisplayName','linear');
set(gca,'xlim',...
    [min(responseLevelDrivePoint_m) max(responseLevelDrivePoint_m)]);
xlabel('fundamental harmonic drive point amplitude in m');
ylabel('modal damping ratio in %');

%% Evaluate modal-harmonic decomposition of mechanical energy

% Loop over hold phases
omlin = 2*pi*linearModalFrequency_Hz;   % linear modal frequencies in rad/s
E = zeros(length(frequency_Hz),size(linearPhi,2),H);
for iHold=1:size(E,1)
    % Estimate complex modal amplitudes via least-squares regression.
    eta = linearPhi\transpose(squeeze(Response_m(iHold,2:end,:)));
    % Determine contribution to period-average of the mechanical energy, 
    % per mode and harmonic, see e.g. [Woiwode.2024].
    E(iHold,:,:) = 1/4*( ...
        repmat(((1:H)*2*pi*frequency_Hz(iHold)),size(eta,1),1).^2 + ...
        repmat(omlin(:),1,size(eta,2)).^2 ) .* abs(eta).^2;
end

% Identify most important contributions. As reference, the fundamental
% harmonic of the target mode is used (which should be the only relevant
% one in the linear case).
ref = squeeze(E(:,settings.indexTargetMode,1));
% The threshold might have to be adjusted for other measurements.
thrshld = 5e-3;
[row,col] = find(permute(sum(E./repmat(ref,1,size(linearPhi,2),...
    H)>thrshld,1)>=1,[2 3 1]));

% Plot results.
figure; hold on;
legstr = cell(length(row),1);
% Loop over non-resonant modes
for i=1:length(row)
    if row(i)==settings.indexTargetMode && col(i)==1
        % This is the reference mode and harmonic leading to the trivial
        % result of unity. This is not added to the plot.
        continue;
    end
    plot(responseLevelDrivePoint_m,squeeze(E(:,row(i),col(i))./ref),...
        'DisplayName',['m=' num2str(row(i)) ',h=' num2str(col(i))]);
end
set(gca,'xscale','log','yscale','log','ylim',[thrshld/10 1],...
    'xlim',[min(responseLevelDrivePoint_m) ...
    max(responseLevelDrivePoint_m)]);
legend('Location','NW');
xlabel('fundamental harmonic drive point amplitude in m');
ylabel(['E(m,h)/E(' num2str(settings.indexTargetMode) ',1)']);

%% Reconstruct frequency response curve using Single Nonlinear Mode Theory

% Specify magnitude of harmonic forcing and drive point. The drive point 
% may be different from the drive point in the backbone test, but it must 
% be one where the response has been acquired. The force magnitude used 
% here is arbitrary - of course this needs to be adjusted to whatever
% you might be interested in.
Fexc_1H = 1/2*max(abs(Excitation(:,2)));
idxNewDrivePoint = settings.indexSensorDrivePoint;

% Interpolate backbone results for fine resolution of response curve.
interpMethod = 'PCHIP';
interpModalAmplitude = linspace(modalAmplitude(1),modalAmplitude(end),1e3);
interpModalFrequency_rad = 2*pi*interp1(modalAmplitude,...
    modalFrequency_Hz,interpModalAmplitude,interpMethod);
interpModalDampingRatio = interp1(modalAmplitude,...
    modalDampingRatio,interpModalAmplitude,interpMethod);
interpPhiExc = interp1(modalAmplitude,...
    Phi(:,2,idxNewDrivePoint),interpModalAmplitude,interpMethod);

% Use closed-form expression from [Schwarz.2020] for frequency response
% curve.
p2 = (interpModalFrequency_rad).^2 - ...
    2*(interpModalDampingRatio.*interpModalFrequency_rad).^2;
rad = p2.^2 - interpModalFrequency_rad.^4 + ...
    abs(conj(interpPhiExc)*Fexc_1H).^2./interpModalAmplitude.^2;
OmR = sqrt(p2 + sqrt(rad));
OmL = sqrt(p2 - sqrt(rad));
phaseL = -rad2deg(angle(-OmL.^2 + ...
    1i*OmL.*2.*interpModalDampingRatio.*interpModalFrequency_rad + ...
    interpModalFrequency_rad.^2));
phaseR = -rad2deg(angle(-OmR.^2 + ...
    1i*OmR.*2.*interpModalDampingRatio.*interpModalFrequency_rad + ...
    interpModalFrequency_rad.^2));

% Plot results.
figure; tiledlayout(2,1);
nexttile(1);
hold on;
plot(modalFrequency_Hz,responseLevelDrivePoint_m,...
    'g','LineWidth',1,'DisplayName','backbone');
plot(OmL(rad>=0)/2/pi,...
    abs(interpModalAmplitude(rad>=0).*interpPhiExc(rad>=0)),...
    'r.','DisplayName','NM-ROM');
plot(OmR(rad>=0)/2/pi,...
    abs(interpModalAmplitude(rad>=0).*interpPhiExc(rad>=0)),...
    'r.','HandleVisibility','off');
xlabel('frequency in Hz')
ylabel('fundamental harmonic drive point amplitude in m')
legend;
nexttile(2);
hold on
plot(OmL(rad>=0)/2/pi,phaseL(rad>=0),'r.');
plot(OmR(rad>=0)/2/pi,phaseR(rad>=0),'r.');
xlabel('frequency in Hz');
ylabel('phase lag in °');