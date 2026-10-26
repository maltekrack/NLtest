%========================================================================
% DESCRIPTION: 
% The purpose of this function is to output a valid 'settings' structure
% required for backbone tracking. To this end, parameters specified by 
% the user (fields of input 'settings' structure) are checked. Unspecified
% fields are added and set to default values. The settings are also
% reported to the user via command line output.
% 
% Linear modal parameters of the plant and the structure under test (SUT)
% with regard to the target mode are needed, together with the exciter 
% moving mass, to obtain some of the default settings, see 
% DOC/BackboneTracking.md. If no plant properties are provided, an ideal 
% exciter is assumed (zero moving mass; plant modal parameters identical 
% to that of SUT).
% 
% INPUT
%   VARIABLE                    MEANING                         TYPE
%   settings                    structure with settings for     struct
%                               backbone tracking    
%
% OUTPUT
%   VARIABLE                    MEANING                         TYPE
%   settings                    structure with settings for     struct
%                               backbone tracking 
%
% USAGE/INSTRUCTIONS
% Several mandatory fields of the 'settings' structure are expected to be
% populated by the user. These particular settings are specific to the
% system/test that is being conducted. The following list outlines these
% variables. 
% NOTE: The following is not comprehensive of all fields within the settings 
% structure. Please consult the documentation for the full list. 
% 
%   FIELDNAME                     MEANING                         TYPE
%     linearPlantFrequency_Hz     linear modal frequency of       1x1 double
%                                   the plant in Hz
%     linearPlantDampingRatio     linear modal damping ratio      1x1 double
%                                   of the plant
%     linearPlantPhiC             linear mass-normalized mode     1x1 double
%                                   shape of plant at the  
%                                   control point
%     excitationQuantity          physical quantity of            string
%                                   excitation source  
%     responseQuantity            physical quantities of          cell array ...
%                                   response source(s)              of strings
%     testType                    physical quantity of            string
%                                   test stepping scheme
%     levels                      array of test amplitudes        Nx1 double
%     omLP                        cutoff frequency for the        1x1 double
%                                   adaptive filter
% 
% REFERENCES
% [1] P. Hippold, M. Scheel, L. Renson, M. Krack: Robust and fast backbone
%       tracking via phase-locked loops, MSSP 220 (2024), 
%       DOI: 10.1016/j.ymssp.2024.111670
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

function [settings] = checkSettingsBackboneTracker(settings)
%% Handle unspecified plant/SUT modal parameters
disp('**INFO on plant/SUT properties');
% Mandatory inputs: plant modal frequency, plant modal damping ratio, plant
% mode shape at control sensor
if ~isfield(settings,'linearPlantFrequency_Hz') || ...
        ~isfield(settings,'linearPlantDampingRatio') || ...
        ~isfield(settings,'linearPlantPhiC')
    error(['No complete set of modal properties provided. The input must ' ...
        'include att least linearPlantFrequency_Hz, linearPlantDampingRatio,' ...
        ' linearPlantPhiC.']);
else
    if numel(settings.linearPlantFrequency_Hz)~=1 || settings.linearPlantFrequency_Hz<0
        error('Linear modal frequency of the plant has to be a positive scalar.');
    end
    if numel(settings.linearPlantDampingRatio)~=1 || settings.linearPlantDampingRatio<0
        error('Linear modal damping ratio of the plant has to be a positive scalar.');
    end
    if numel(settings.linearPlantPhiC)~=1
        error('Linear plant mode shape at the response sensor has to be scalar.');
    end

end

% Check if SUT modal frequency is provided (optional)
if ~isfield(settings,'linearModalFrequency_Hz')
    settings.linearModalFrequency_Hz = settings.linearPlantFrequency_Hz;
    disp(['- Only one modal frequency provided; this frequency is used for ' ...
        'plant and SUT (as for ideal exciter behavior).']);
else
    if numel(settings.linearModalFrequency_Hz)~=1 || settings.linearModalFrequency_Hz<0
        error('Linear modal frequency of the SUT has to be a positive scalar.');
    end
end

% Check if SUT modal damping ratio is provided (optional)
if ~isfield(settings,'linearModalDampingRatio')
    settings.linearModalDampingRatio = settings.linearPlantDampingRatio;
    disp(['- Only one modal damping ratio provided; this damping ratio is ' ...
        'used for plant and SUT (as for ideal exciter behavior).']);
else
    if numel(settings.linearModalDampingRatio)~=1 || settings.linearModalDampingRatio<0
        error('Linear modal damping ratio of the SUT has to be a positive scalar.');
    end
end

% Check additional mode shape entries - requirements depend on excitation
% type
if strcmp(settings.excitationQuantity,'force')
    % force excitation
    if ~isfield(settings,'linearPhiEx')
        if ~isfield(settings,'linearPlantPhiEx')
            settings.linearPlantPhiEx = settings.linearPlantPhiC;
            if ~isfield(settings,'linearPhiC')
                % ideal exciter, measurement at drive point
                settings.linearPhiEx = settings.linearPlantPhiC;
                settings.linearPhiC = settings.linearPlantPhiC;
                disp(['- Only linearPlantPhiC provided. Assuming ideal ' ...
                    'exciter behavior and response measurement at the drive point.']);
            else
                % (potentionally) non-ideal exciter behavior, measurement 
                % at drive point
                settings.linearPhiEx = settings.linearPhiC;
                disp(['- Only mode shape at the response sensor ' ...
                    'location provided. Assuming response measurement at the drive point.']);
            end
        else
            if ~isfield(settings,'linearPhiC')
                % ideal exciter behavior, measurement (potentially) not at
                % drive point
                settings.linearPhiC = settings.linearPlantPhiC;
                settings.linearPhiEx = settings.linearPlantPhiEx;
                disp(['- Only mode shape entries of the plant provided. ' ...
                    'Assuming ideal exciter behavior.']);
            else
                error(['Inconlusive combination of plant/SUT parameters ' ...
                    'provided. It is not clear if ideal exciter behavior is assumed.'])
            end
        end
    else
        if ~isfield(settings,'linearPlantPhiEx') || ~isfield(settings,'linearPhiC')
            error(['Inconlusive combination of plant/SUT parameters ' ...
                    'provided. It is not clear if ideal exciter behavior ' ...
                    'and/or response measurement at the drive point is assumed.'])
        else
            % (potentionally) non-ideal exciter behavior, measurement
            % (potentially) not at drive point
        end
    end
    % check drive point mode shapes
    if numel(settings.linearPlantPhiEx)~=1
        error('Linear plant mode shape at the drive point has to be scalar.');
    end
    if numel(settings.linearPhiEx)~=1
        error('Linear SUT mode shape at the drive point has to be scalar.');
    end
    % check exciter mass
    if ~isfield(settings,'exciterMass_kg')
        settings.exciterMass_kg = 0;
        disp('- No moving mass of exciter provided. Assuming ideal exciter behavior.');
    end
else
    % base excitation
    if ~isfield(settings,'linearPhiC')
        % ideal exciter behavior
        settings.linearPhiC = settings.linearPlantPhiC;
        disp(['- Only one mode shape entry at the control sensor provided; this ' ...
            'entry is used for plant and SUT (as for ideal exciter behavior).']);
    end
    % check if masses of SUT and exciter are provided
    if ~isfield(settings,'exciterMass_kg') || ~isfield(settings,'SUTMass_kg')
        settings.exciterMass_kg = 1;
        settings.SUTMass_kg = 0;
        disp(['- Moving mass of exciter and/or mass of SUT not provided. ' ...
            'Assuming negligible SUT mass compared to exciter.']);
    end
    % Set linearPhiEx to arbitrary value (referenced in the Simulink model
    % but not used in case of base excitation).
    settings.linearPhiEx = NaN;
end
% check SUT mode shape at control sensor
if numel(settings.linearPhiC)~=1
    error('Linear SUT mode shape at the control sensor has to be scalar.');
end

%% Instrumentation settings
fprintf('**INFO on instrumentation\n');

% Response sensor to be used for control. 
if ~isfield(settings,'indexSensorControl')
    if isscalar(settings.responseQuantity)
        % A single response sensor is specified. Use this as control
        % sensor.
        settings.indexSensorControl = 1;
        fprintf('- Sensor %d is selected for control. [DEFAULT]\n', ...
            settings.indexSensorControl)
    else
        error(['If more than one response sensor is used, ' ...
            'the sensor to be used for control must be specified.']);
    end
else
    fprintf('- Sensor %d is selected for control.\n', ...
        settings.indexSensorControl)
end

if strcmp(settings.excitationQuantity,'force')
    if ~isfield(settings,'indexSensorDrivePoint')
        % Assume that drive point is collocated with sensor used for control.
        disp(['- No drive point index provided. Assuming that the drive ' ...
            'point is collocated with the sensor used for control. [DEFAULT]']);
    else
        fprintf('- Sensor %d is selected as drive point.\n', ...
            settings.indexSensorControl)
    end
end

% Target mode (whose backbone is to be tracked)
if ~isfield(settings,'indexTargetMode')
    settings.indexTargetMode = 1;
    fprintf('- Mode %d is selected as target mode. [DEFAULT]\n', ...
        settings.indexTargetMode);
else
    fprintf('- Mode %d is selected as target mode.\n', ...
        settings.indexTargetMode);
end

% Initial frequency
if ~isfield(settings,'initialFrequency_Hz')
    settings.initialFrequency_Hz = settings.linearModalFrequency_Hz;
        fprintf(['- The inital excitation frequency is set to the ' ...
            'linear modal frequency. [DEFAULT]\n']);
else
    if numel(settings.initalFrequency_Hz)~=1 || settings.intialFrequency_Hz <= 0
        error('The initial frequency has to be a positive scalar.');
    elseif abs(settings.initialFrequency_Hz - settings.linearModalFrequency_Hz)/ ...
            settings.linearModalFrequency_Hz > 0.05
        warning(['Large deviation of the initial frequency from the ' ...
            'linear modal frequency of the target mode detected.'])
    end
end

% Sample time
if ~isfield(settings, 'samplingTime_s')
    settings.samplingTime_s = 1/(100*settings.linearModalFrequency_Hz);
    fprintf('- The sampling frequency is set to %.0f Hz. [DEFAULT]\n', ...
        1/settings.samplingTime_s);
else
    fprintf('- The sampling frequency is set to %.0f Hz.\n', ...
        1/settings.samplingTime_s);
end

% Transducer sensitivities
if ~isfield(settings, 'sensitivityExcitation')
    settings.sensitivityExcitation = 1;
    fprintf('- The sensitivity of the excitation sensor is %.3f <units>/V. [DEFAULT]\n', settings.sensitivityExcitation);
else
    fprintf('- The sensitivity of the excitation sensor is %.3f <units>/V.\n', settings.sensitivityExcitation);
end
if ~isfield(settings, 'sensitivityResponse')
    settings.sensitivityResponse = 1;
    fprintf('- The sensitivity of the response sensor is %.3f <units>/V. [DEFAULT]\n', settings.sensitivityResponse);
else
    fprintf('- The sensitivity of the response sensor is %.3f <units>/V.\n', settings.sensitivityResponse);
end

% Transducer delays
if ~isfield(settings,'delayExcitation')
    settings.delayExcitation = 0;
    fprintf('- The measurement delay of the excitation is %.1f µs. [DEFAULT]\n', settings.delayExcitation*1e6);
else
    fprintf('- The measurement delay of the excitation is %.1f µs.\n', settings.delayExcitation*1e6);
end
if ~isfield(settings, 'delayResponse')
    settings.delayResponse = 0;
    fprintf('- The measurement delay of the response is %.1f µs. [DEFAULT]\n', settings.delayResponse*1e6);
else
    fprintf('- The measurement delay of the response is %.1f µs.\n', settings.delayResponse*1e6);
end
% Compensate effect of delays on phase lag. For this, the less delayed 
% signal is shifted by a certain number of samples.
if settings.delayResponse > settings.delayExcitation
    settings.nDelayExcitation = round((settings.delayResponse - ...
        settings.delayExcitation)/...
        settings.samplingTime_s);
    settings.nDelayResponse = 0;
    fprintf(['- The excitation is delayed by %d samples to compensate the ' ...
        'difference in measurement delays.\n'], ...
        settings.nDelayExcitation);
elseif settings.delayResponse < settings.delayExcitation
    settings.nDelayExcitation = 0;
    settings.nDelayResponse = round((settings.delayExcitation - ...
        settings.delayResponse)/...
        settings.samplingTime_s);
    fprintf(['- The response is delayed by %d samples to compensate the ' ...
        'difference in measurement delays.\n'], ...
        settings.nDelayResponse);
else
    settings.nDelayExcitation = 0;
    settings.nDelayResponse = 0;
    fprintf('- The measurement delays are equal. No compensation is necessary.\n')
end
if mod(abs(settings.delayExcitation-settings.delayResponse),settings.samplingTime_s) ~= 0
    warning(['The relative delay of the sensors is not an integer ' ...
        'multiple of the sampling time. Consider adjusting the sample ' ...
        'time to avoid inaccurate phase estimation.']);
end

% Determine target phase shift for phase resonance
% NOTE:'respQtyNumeric' and 'excQtyNumeric' are needed in Simulink.
switch settings.excitationQuantity
    case 'force'
        switch settings.responseQuantity{settings.indexSensorControl}
            case 'displacement'
                settings.targetPhaseShift_rad = pi/2;
                settings.respQtyNumeric = 1;
            case 'velocity'
                settings.targetPhaseShift_rad = 0;
                settings.respQtyNumeric = 2;
            case 'acceleration'
                settings.targetPhaseShift_rad = -pi/2;
                settings.respQtyNumeric = 3;
            otherwise
                error('Unknown response type.');
        end
        settings.excQtyNumeric = 0;
    case 'displacement'
        switch settings.responseQuantity{settings.indexSensorControl}
            case 'displacement'
                settings.targetPhaseShift_rad = pi/2;
                settings.respQtyNumeric = 1;
            case 'velocity'
                settings.targetPhaseShift_rad = 0;
                settings.respQtyNumeric = 2;
            case 'acceleration'
                settings.targetPhaseShift_rad = -pi/2;
                settings.respQtyNumeric = 3;
            otherwise
                error('Unknown response type.');
        end
        settings.excQtyNumeric = 1;
    case 'velocity'
        switch settings.responseQuantity{settings.indexSensorControl}
            case 'displacement'
                settings.targetPhaseShift_rad = pi;
                settings.respQtyNumeric = 1;
            case 'velocity'
                settings.targetPhaseShift_rad = pi/2;
                settings.respQtyNumeric = 2;
            case 'acceleration'
                settings.targetPhaseShift_rad = 0;
                settings.respQtyNumeric = 3;
            otherwise
                error('Unknown response type.');
        end
        settings.excQtyNumeric = 2;
    case 'acceleration'
        switch settings.responseQuantity{settings.indexSensorControl}
            case 'displacement'
                settings.targetPhaseShift_rad = -pi/2;
                settings.respQtyNumeric = 1;
            case 'velocity'
                settings.targetPhaseShift_rad = pi;
                settings.respQtyNumeric = 2;
            case 'acceleration'
                settings.targetPhaseShift_rad = pi/2;
                settings.respQtyNumeric = 3;
            otherwise
                error('Unknown response type.');
        end
        settings.excQtyNumeric = 3;
    otherwise
        error('Unknown excitation type.');
end

% Command line message according to excitation type (force or base)
switch settings.excitationQuantity
    case 'force'
        % correct phase shift if mode shape at drive point and at control 
        % sensor have different signs.
        if sign(settings.linearPhiEx)*sign(settings.linearPhiC) < 0
            settings.targetPhaseShift_rad = ...
                settings.targetPhaseShift_rad + pi;
            % wrap to [-pi, pi]
            settings.targetPhaseShift_rad = settings.targetPhaseShift_rad - ...
                floor(settings.targetPhaseShift_rad/pi)*2*pi;
        end
        fprintf(['- Force excitation is used. The response at the control ' ...
            'sensor is measured in terms of %s.\n  The resulting target phase ' ...
            'lag (arg(excitation)-arg(response)) is %.0f degrees.\n'], ...
            settings.responseQuantity{settings.indexSensorControl}, ...
            settings.targetPhaseShift_rad*180/pi)
    case {'displacement', 'velocity', 'acceleration'}
        fprintf(['- Base excitation is used. The base %s is measured. The ' ...
            'response used for control is measured in terms of %s.\n  The ' ...
            'resulting target phase lag (arg(excitation)-arg(response)) is ' ...
            '%.0f degrees.\n'], ...
            settings.excitationQuantity, ...
            settings.responseQuantity{settings.indexSensorControl}, ...
            settings.targetPhaseShift_rad*180/pi);
        warning(['***Note that the phase lag at resonance depends on the ' ...
            'sensor location. CHECK IF THE TARGET PHASE LAG MATCHES THE ' ...
            'PHASE LAG AT RESONANCE FOR YOUR SELECTED SENSOR LOCATION. ' ...
            'The target phase lag should be the phase lag at resonance, ' ...
            'using the definition PHASELAG = arg(excitation)-arg(response).'])
end

%% Settling detection and number of settled periods to be recorded
fprintf('**INFO on settling detection and recording\n');
% Threshold for phase settling
if ~isfield(settings,'tolerancePhaseShift_degree')
    settings.tolerancePhaseShift_degree = 1;
    fprintf('- The phase shift tolerance is %.2f degrees. [DEFAULT]\n', ...
        settings.tolerancePhaseShift_degree);
else
    fprintf('- The phase shift tolerance is %.2f degrees.\n', ...
        settings.tolerancePhaseShift_degree);
end

% Threshold for amplitude settling (tolerance relative to setpoint)
if ~isfield(settings,'toleranceRelativeAmplitude')
    settings.toleranceRelativeAmplitude = 0.01;
    fprintf('- The relative amplitude tolerance is %.2f %%. [DEFAULT]\n', ...
        settings.toleranceRelativeAmplitude*100);
else
    fprintf('- The relative amplitude tolerance is %.2f %%.\n', ...
        settings.toleranceRelativeAmplitude*100);
end

% Number of periods in the tolerance corridor are required to meet the
% settling criterion (and start recording)
if ~isfield(settings, 'periodsInTolerance')
    settings.periodsInTolerance = 20;
    fprintf('- The required number of periods in tolerance is %d. [DEFAULT]\n', ...
        settings.periodsInTolerance);
else
    fprintf('- The required number of periods in tolerance is %d.\n', ...
        settings.periodsInTolerance);
end

% Number of periods to be recorded (once settling criterion has been met)
if ~isfield(settings, 'recordedPeriods')
    settings.recordedPeriods = 1;
    fprintf(['- The number of periods to be recorded for each ' ...
        'backbone point is %d. [DEFAULT]\n'], ...
        settings.recordedPeriods);
else
    fprintf(['- The number of periods to be recorded for each ' ...
        'backbone point is %d.\n'], ...
        settings.recordedPeriods);
end

%% Excitation profile
fprintf('**INFO on excitation profile\n');

% Check whether valid test type is specified
if ~isfield(settings,'testType')
    error('No test type specified. Set test type to steppedVoltage or steppedAmplitude.');
elseif ~strcmp(settings.testType, 'steppedVoltage') && ...
        ~strcmp(settings.testType,'steppedAmplitude')
    error('Invalid test type. Set test type to steppedVoltage or steppedAmplitude.')
end

% Check whether excitation levels were specified
if ~isfield(settings,'levels')
    error('No levels specified.')
end

% Voltage limit
if ~isfield(settings,'voltageLimit')
    settings.voltageLimit = inf;
    warning(['No voltage limit specified. Set an appropriate ' ...
        'voltage limit to protect your hardware from possible overload. ' ...
        '(You can ignore this warning in case of a virtual experiment.)']);
else
    fprintf('- The plant input voltage is limited to %.2f V.\n',...
        settings.voltageLimit);
end

% Calculate default ramp time as 5% settling time of the amplitude 
% transient (assuming amplitude-constant plant damping) [1].
if strcmp(settings.excitationQuantity, 'force')
    defaultRampTime_s = -log(0.05)/...
        (settings.linearPlantDampingRatio*settings.linearPlantFrequency_Hz*2*pi*...
        (1+settings.exciterMass_kg*settings.linearPhiEx^2));
else
     defaultRampTime_s = -log(0.05)/...
        (settings.linearPlantDampingRatio*settings.linearPlantFrequency_Hz*2*pi);
end

% Initial ramp time
if ~isfield(settings, 'firstRampTime_s')
    % Use default, but minimum 100 linear periods for first ramp to 
    % guarantee smooth startup. Limit to 1000 periods in case of very 
    % light damping.
    settings.firstRampTime_s = min( ...
        max(defaultRampTime_s,100/settings.linearModalFrequency_Hz), ...
        1000/settings.linearModalFrequency_Hz );
    fprintf(['- The initial voltage ramp takes %.0f linear ', ...
        'periods. [DEFAULT]\n'], ...
        settings.firstRampTime_s*settings.linearModalFrequency_Hz);
else
    fprintf(['- The initial voltage ramp takes %.0f linear ', ...
        'periods.\n'], settings.firstRampTime_s*settings.linearModalFrequency_Hz);
    if settings.firstRampTime_s*settings.linearModalFrequency_Hz < 100 || ...
            settings.firstRampTime_s*settings.linearModalFrequency_Hz > 1000
        warning(['The specified initial ramp time is not within the typical' ...
            'range of 100 to 1000 linear periods.']);
    end
end

% Set initial level, steps and ramps. Estimate minimum test duration.
switch settings.testType
    case 'steppedVoltage'
        fprintf(['- The backbone is tracked using prescribed shaker ' ...
            'input voltage levels.\n']);

        % Set numeric test specifier (used in Simulink)
        settings.testTypeNumeric = 1;

        % Store specified voltage levels (for plotting)
        settings.voltageLevels = settings.levels; 

        % Adopt initial voltage level
        settings.initialVoltage = settings.levels(1);

        % 'initialVoltage' is later added when stepping and must therefore 
        % be substracted here
        settings.levels = settings.levels - settings.initialVoltage;

        % Check whether any of the specified levels exceeds the limit
        if max(settings.voltageLevels) > settings.voltageLimit
            error(['The voltage limit is lower than the highest voltage level.' ...
                ' Adjust either the limit or the levels.']);
        end
        
        % Ramp time (for transition between levels)
        if ~isfield(settings,'rampTime_s')
            % Use default time for all ramps in case of stepped voltage. 
            % Limit to 1000 periods in case of light damping
            if defaultRampTime_s <= 1000/settings.linearModalFrequency_Hz
                settings.rampTime_s = defaultRampTime_s;
                fprintf(['- Each following voltage ramp takes %.0f linear ', ...
                    'periods. [DEFAULT]\n'], ...
                    settings.rampTime_s*settings.linearModalFrequency_Hz)
            else
                settings.rampTime_s = 1000/settings.linearModalFrequency_Hz;
                warning(['The voltage ramp was limited to 1000 linear ' ...
                    'periods although the plant decay rate suggests a higher'...
                    ' value. Consider using response amplitude control to ' ...
                    'reduce the test time and guarantee a steady state.'])
            end
        else
            fprintf(['- Each following voltage ramp takes %.0f linear ', ...
                    'periods.\n'], ...
                    settings.rampTime_s*settings.linearModalFrequency_Hz)
            if settings.rampTime_s*settings.linearModalFrequency_Hz < 100 || ...
                    settings.firstRampTime_s*settings.linearModalFrequency_Hz > 1000
                warning(['The specified ramp time is not within the typical' ...
                    'range of 100 to 1000 linear periods.']);
            end
        end

        % Estimate minimum test duration
        minimumTestDuration = settings.firstRampTime_s*settings.linearModalFrequency_Hz + ...
            (length(settings.levels)-1)*(settings.rampTime_s*settings.linearModalFrequency_Hz) + ...
            length(settings.levels)*(settings.periodsInTolerance+settings.recordedPeriods);

    case 'steppedAmplitude'
        fprintf(['- The backbone is tracked using controlled response ' ...
            'amplitude levels.\n']);

        % Set numeric test specifier (used in Simulink)
        settings.testTypeNumeric = 2;

        % Set initial voltage level
        if ~isfield(settings,'initialVoltage')
            % Use linear estimate of voltage required to reach first 
            % requested response level
            if strcmp(settings.excitationQuantity,'force')
                settings.initialVoltage = 1/(settings.linearPlantPhiEx * settings.linearPlantPhiC) * ...
                    2*settings.linearPlantDampingRatio* ...
                    (settings.linearModalFrequency_Hz*2*pi)^2 * settings.levels(1);
            else
                settings.initialVoltage = 1/settings.linearPlantPhiC^2 * ...
                    2*settings.linearPlantDampingRatio* ...
                    (settings.linearModalFrequency_Hz*2*pi)^2 * settings.levels(1);
            end
            fprintf(['- The initial voltage level estimated from the linear'...
                ' plant parameters is %.2f V. [DEFAULT]\n'], ...
                settings.initialVoltage);
        else
            fprintf('- The initial voltage level is %.2f V.\n', ...
                settings.initialVoltage);
        end

        % Adopt amplitude levels
        settings.amplitudeLevels = settings.levels;

        % Ramp time (for transition between levels)
        if ~isfield(settings, 'rampTime_s')
            % amplitude control: use fixed number of linear cycles (here:
            % 100)
            settings.rampTime_s = 100/settings.linearModalFrequency_Hz;
            fprintf(['- Each response amplitude ramp takes %.0f linear ', ...
                'periods. [DEFAULT]\n'], settings.rampTime_s*settings.linearModalFrequency_Hz)
        else
            fprintf('- Each response amplitude ramp takes %.0f linear ', ...
                'periods.\n', settings.rampTime_s*settings.linearModalFrequency_Hz)
            if settings.RampTime_s*settings.linearModalFrequency_Hz < 100 || ...
                    settings.firstRampTime_s*settings.linearModalFrequency_Hz > 1000
                warning(['The specified ramp time is not within the typical' ...
                    'range of 100 to 1000 linear periods.']);
            end
        end

        % Estimate minimum test duration
        minimumTestDuration = settings.firstRampTime_s*settings.linearModalFrequency_Hz + ...
            settings.periodsInTolerance + ...
            (length(settings.levels)-1)*(settings.rampTime_s*settings.linearModalFrequency_Hz) + ...
            length(settings.levels)*(settings.periodsInTolerance+settings.recordedPeriods);

end

% Total number of levels (may contain several backbone runs)
settings.nLevelsTotal = length(settings.levels);

%% Adaptive filter
fprintf('**INFO on Fourier decomposition\n');

% Check cutoff frequency
if ~isfield(settings, 'omLP')
    error('No adaptive filter cutoff frequency specified.');
elseif ~(settings.omLP > 0)
    error('Adaptive filter cutoff frequency must be a positive number.')
end
fprintf(['- The ratio of adaptive filter cutoff frequency to linear' ...
    ' natural frequency of the target mode is %.3f.\n'],...
    settings.omLP/settings.linearModalFrequency_Hz/2/pi);
if settings.omLP/settings.linearModalFrequency_Hz/2/pi > 1
    warning(['This value is above the ' ...
        'typical range of [0.01, 1].']);
elseif settings.omLP/settings.linearModalFrequency_Hz/2/pi < 0.01
    warning(['This value is below the ' ...
        'typical range of [0.01, 1].']);
end

% Check harmonic order
if ~isfield(settings, 'H')
    settings.H = floor(1/(2.56*settings.samplingTime_s*settings.linearModalFrequency_Hz));
    fprintf(['- The harmonic order is set to %d based on the sampling ' ...
        'frequency. [DEFAULT]\n'], settings.H);
else
    fprintf('- The harmonic order is set to %d.\n', settings.H)
    if settings.H > floor(1/(2.56*settings.samplingTime_s*settings.linearModalFrequency_Hz))
        warning(['Frequency of highest harmonic to be estimated is close to or above maximum according to Nyquist criterion. ' ...
            'Consider increasing the sampling frequency or decreasing the harmonic order.']);
    end
end

%% Print estimated minimum test duration 
fprintf(['- The minimum test duration (assuming constant modal ' ...
    'frequency) is %.0f vibration cycles / %.0f s.\n  %.2f %% of the ' ...
    'minimum test duration are expected to be spent on recording the ' ...
    'steady state.\n'], ...
    minimumTestDuration, minimumTestDuration/settings.linearModalFrequency_Hz, ...
    length(settings.levels)*settings.recordedPeriods/minimumTestDuration);
%% Plot excitation profile
figure;
hold on;
xlabel('level');
switch settings.testType 
    case 'steppedVoltage'
        ylabel('voltage (V)');
        plot(settings.voltageLevels,'-x')
    case 'steppedAmplitude'
        ylabel('amplitude setpoint (displacement unit)')
        plot(settings.amplitudeLevels,'-x')
end
box on;
end
