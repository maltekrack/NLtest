%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function estimates Fourier coefficients from time series obtained 
% during step-type single-frequency nonlinear vibration tests.
% 
% This function is intended primarily for the following test types:
%         Phase Resonance Test (PRT),
%         Response Controlled Test (RCT), and
%         Excitation Controlled Test (ECT).
% 
% In all those tests, one has a single instantaneous frequency. The
% frequency can be imposed or feedback-controlled using a phase controller
% such as a phase-locked loop.
% 
% In a PRT, the phase lag between excitation and response is kept at 
% resonance, while the the excitation level is stepped. In a RCT, the 
% response level is kept fixed, while the frequency or the phase lag is
% stepped. In an ECT, the excitation level is kept fixed, while the
% frequency or the phase lag is stepped.
% The excitation is commonly the voltage input to the shaker amplifier, or 
% a feedback-controlled applied force or base motion. The response is 
% commonly acquired with motion sensors (displacement, velocity, 
% acceleration).
% 
% In this function, it is assumed that the stepping is done along a 
% prescribed profile, consisting of ramp and hold phases. That profile is 
% specified in the 'steps' variable, whose first column are time
% instants and the second column are values (e.g. excitation/response
% level or phase or frequency). Durign the test, the values are typicaly 
% interpolated linearly between time instants. If the value is the same at 
% one time instant and the next, this is called a HOLD PHASE. If the value 
% differs, this is called a RAMP PHASE. This function identifies the hold 
% phases and processes the last section of those phases.
% REMARK: Because the profile is typically piecewise constant (thanks to
% the linear ramps), the name 'steps' is somewhat debatable.
% 
% To deal also with phase controlled tests, where the instantaneous
% frequency is not an imposed piecewise constant but a dynamic quantity, 
% the variable 'instFreq' is to be provided as time series of the same 
% dimension as the discrete time vector ('time'). Further mandatory input 
% is a scalar excitation ('exc') and a scalar response ('resp') time 
% series. Additional time series e.g. of further response sensors can be
% provided as optional input argument. For many tasks, only a single
% response signal is sufficient. Then it is convenient to have a simple
% data structure analogous to the excitation. This is why the scalar
% 'resp' is kept separate from the optional additional time series. Of
% course, the additional time series may contain a copy of 'resp'. After
% all, the same signal processing is applied to all time series (except 
% 'instFreq').
% 
% The steady excitation frequency is estimated as the mean value of 
% 'instFreq' in the last part of the hold phase. The length of this time 
% span is 'nominalFreq' times 
%       'target_number_of_last_periods_of_hold_phase_to_process'. Once the 
% steady excitation frequency is estimated, this time span is updated. 
% 
% The Fourier coefficients of the time series are obtained via FFT.
% 
% INPUT
%   VARIABLE                MEANING                     TYPE
%   steps                   test profile                n x 2 double
%       COLUMNS
%        1                  time instant                n x 1 double
%        2                  (set) value                 n x 1 double
%   time                    Time                        n x 1 double
%   instFreq                Instantaneous frequency     n x 1 double
%   exc                     Excitation time series      n x 1 double
%   resp                    Response time series        n x 1 double
%   varargin{1}             Additional time series      n x nAdd double
%   varargin{2}             Options                     structure
% 
%      FIELD NAMES
% 
%      target_number_...   number of periods to be      positive integer
%       of_last_periods_...  used for estimating
%       of_hold_phase_to_... Fourier coefficients
%       process
% 
%      H                    harmonic order of Fourier   positive integer
%                           coefficients to be estim. 
% 
%       nominalFreq         nominal frequency, used     double
%                           for estimating time span
%                           corresponding to target
%                           number of periods
% 
% OUTPUT
%   VARIABLE        MEANING                              TYPE
%     freq          steady excitation frequency          nHold x 1 double
%     EXC           complex Fourier coefficients of exc  nHold x H+1 double
%     RESP          ... of resp                          nHold x H+1 double
%     ADDSENS       ... of additional time series        nHold x 1 cell of
%                                                        H+1 x nAdd double
%     indPROCESS    selected samples for evaluation      nHold x n Boolean
%     time,         output time series (may be useful    (same as input)
%     instFreq,exc,         if interpolation is done)
%     resp,addSens
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of NLtest available via
% https://github.com/maltekrack/NLtest.
% 
% COPYRIGHT (C) 2024
%   Malte Krack (malte.krack@ila.uni-stuttgart.de) 
%   Maren Scheel (maren.scheel@ila.uni-stuttgart.de)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [freq,EXC,RESP,ADDSENS,indPROCESS,...
    time,instFreq,exc,resp,addSens] = ...
    estimateFourierCoefficientsFromSteppedSine(steps,time,instFreq,...
    exc,resp,varargin)
%% Handle and check user input

% Set default options and adopt from input if specified
opt = struct('H',[],...
    'target_number_of_last_periods_of_hold_phase_to_process',[],...
    'nominalFreq',[]);
if nargin>6
    if isstruct(varargin{2})
        tmp = varargin{2};
        new_fieldnames = fieldnames(tmp);
        for ij=1:length(new_fieldnames)
            opt.(new_fieldnames{ij}) = ...
                tmp.(new_fieldnames{ij});
        end
    else
        error('Expecting input of type structure for options.');
    end
end

% If a target number of periods is not specified for hold phases, use
% default value
if isempty(opt.target_number_of_last_periods_of_hold_phase_to_process)
    opt.target_number_of_last_periods_of_hold_phase_to_process = 100;
    warning(['Target number of last periods of hold phase to process ' ...
        'not provided. Using default value of ' num2str(...
        opt.target_number_of_last_periods_of_hold_phase_to_process)]);
end

% Check if inputs are given as column vectors
n = size(time,1);
if size(time,2)>1||size(instFreq,2)>1||size(exc,2)>1||size(resp,2)>1
    error('Input time series must be column vectors.');
end

% Handle additional time series
if nargin>=5 && ~isempty(varargin{1})
    addSens = varargin{1};
else
    addSens = zeros(n,0);
end

% Check consistency of input dimensions
if size(instFreq,1)~=n||size(exc,1)~=n||size(resp,1)~=n||...
        (~isempty(addSens)&&size(addSens,1)~=n)
    error('Inconsistent input time series dimensions.');
end

% Check time vector
if any(isnan([time;exc;resp]))
    error('NaN values not allowed within input.');
end
% Determine sampling and ensure equidistant sampling
dt = time(2)-time(1);
if any(diff(time)<0)
    error('Forward sampling required.');
elseif max(abs((diff(time)/dt-1)))>1e-6
    % NOTE: With some real-time systems, the time step varies by ca. +/-3%,
    % which can distort the FFT-based estimation of Fourier coefficients.
    % In that case, an interpolation is proposed. The new time vector and
    % interpolated data can then be an important output of this function.
    disp('Non-equidistant samples detected. Interpolating...');
    timetmp = time;
    time = linspace(time(1),time(end),n)';
    dt = time(2)-time(1);
    instFreq = interp1(timetmp,instFreq,time);
    exc = interp1(timetmp,exc,time);
    resp = interp1(timetmp,resp,time);
    if size(addSens,2)>0
        addSens = interp1(timetmp,addSens,time);
    end
end

% Check provided nominal frequency
if ~isempty(opt.nominalFreq)
    if numel(opt.nominalFreq)~=1||opt.nominalFreq<=0
        error('Expecting positive scalar for nominal frequency.')
    end
else
    % None provided, estimate it
    opt.nominalFreq = mean(abs(instFreq(~isnan(instFreq))));
    disp(['No field nominalFreq of options structure provided. ' ...
        'Estimating it from mean absolute value of instFreq as ' ...
        num2str(opt.nominalFreq) ' (same unit as instFreq).']);
end

% Check provided number of harmonics
if ~isempty(opt.H)
    if numel(opt.H)~=1||opt.H<=0||...
            mod(opt.H,1)~=0
        error('Expecting positive integer for number of harmonics.')
    end
else
    % None specified, select maximum. The factor is 2.56 instead of the 
    % Nyquist limit of 2 because anti-aliasing filters commonly restrict
    % the useful frequency bandwidth (see e.g. Allemang and Avitabile
    % (2022): Handbook of experimental structural dynamics).
    opt.H = floor(1/dt/opt.nominalFreq/2.56)-1;
end
%% Interpret step intput
step_times = steps(:,1);
step_values = steps(:,2);
%% Truncate according to available data
data_available = step_times>=time(1) & step_times<=time(end);
step_times = step_times(data_available);
step_values = step_values(data_available);
if isempty(step_times)
    error(['No data available in time frame of test ' ...
        'according to provided step variable.']);
end
%% Identify hold phases and keep those that are long enough
isStartOfHoldPhase = find(diff(step_values)==0);
holdPhase_start = step_times(isStartOfHoldPhase);
holdPhase_end = step_times(isStartOfHoldPhase+1);
% If hold time is too short, scrap hold phase
isLongEnough = (holdPhase_end-holdPhase_start)*opt.nominalFreq > ...
    opt.target_number_of_last_periods_of_hold_phase_to_process;
holdPhase_end = holdPhase_end(isLongEnough);
holdPhase_start = holdPhase_start(isLongEnough);
numHoldPhases = size(holdPhase_end,1);
if numHoldPhases==0
    error('No hold phase long enough for target number of periods.');
end
disp([num2str(numHoldPhases) ' hold phases have been identified.']);
%% Analyze hold phases

% Initialize output
freq = zeros(numHoldPhases,1);
EXC = zeros(numHoldPhases,opt.H+1);
RESP = EXC;
ADDSENS = cell(numHoldPhases,1);
indPROCESS = false(numHoldPhases,n);

% Loop over hold phases
for ihold=1:numHoldPhases

    % Estimate frequency and determine samples to be processed
    holdPhase = [holdPhase_start(ihold) holdPhase_end(ihold)];
    [isToBeProcessed,estimatedFrequency] = ...
        estimateFrequencyAnddetermineSamplesToBeProcessed(time,...
        instFreq,holdPhase,opt);

    % Select and concatenate time series
    timeSeries = [exc(isToBeProcessed) resp(isToBeProcessed) ...
        addSens(isToBeProcessed,:)];

    % Compute Fourier coefficients and update estimated frequency
    [FourierCoefficients,estimatedFrequency] = ...
        computeFourierCoefficients(timeSeries,dt,estimatedFrequency,opt.H);

    % Store indices of selected samples and estimated frequency
    indPROCESS(ihold,:) = isToBeProcessed;
    freq(ihold) = estimatedFrequency;

    % Store Fourier coefficients
    EXC(ihold,:) = FourierCoefficients(:,1);
    RESP(ihold,:) = FourierCoefficients(:,2);
    if size(addSens,2)>0
        ADDSENS{ihold} = ...
            FourierCoefficients(:,2+(1:size(addSens,2)));
    end

    % Diagnostic information on signal processing
    if ihold==1 || ihold==numHoldPhases
        n_samp = sum(indPROCESS(ihold,:));
        np_hold = (holdPhase(2)-holdPhase(1))*freq(ihold);
        np_eval = n_samp*dt*freq(ihold);
        if ihold==1
            disp(['Information on estimation of excitation frequency ' ...
                'and Fourier coefficients in FIRST hold phase:']);
        else
            disp(['Information on estimation of excitation frequency ' ...
                'and Fourier coefficients in LAST hold phase:']);
        end
        disp(['     Duration of hold phase:             ' ...
            num2str(round(np_hold)) ' excitation periods.']);
        disp(['     Portion of hold phase used:         ' ...
            num2str(1e2*np_eval/np_hold) '%.']);
        disp(['     Number of samples used per period:  ' ...
            num2str(round(n_samp/np_eval)) '.']);
    end
end
end
%% NESTED FUNCTION: Est. frequency and determine samples to be processed
function [isToBeProcessed,estimatedFrequency] = ...
    estimateFrequencyAnddetermineSamplesToBeProcessed(time,instFreq,...
        holdPhase,opt)

% Estimate samples based on nominal frequency
isToBeProcessed = time<=holdPhase(2) & ...
    time>(holdPhase(2)-...
    opt.target_number_of_last_periods_of_hold_phase_to_process/...
    opt.nominalFreq);

% Estimate frequency along those samples
estimatedFrequency = mean(instFreq(isToBeProcessed&~isnan(instFreq)));

% Check for validity and consistency
if isnan(estimatedFrequency)
    error('No valid instantaneous frequency at steady state.');
elseif abs(opt.nominalFreq/estimatedFrequency-1)>.2
    warning(['Estimated frequency of ' num2str(estimatedFrequency) ...
        ' is not within +/- 20 percent of opt.nominalFreq '...
        'in hold phase [' num2str(holdPhase(1)) ' '...
        num2str(holdPhase(2)) '].']);
end

% Update selected samples according to updated frequency estimate
isToBeProcessed = time<=holdPhase(2) & ...
    time>(holdPhase(2)-...
    opt.target_number_of_last_periods_of_hold_phase_to_process/...
    estimatedFrequency);
end
%% NESTED FUNCTION: Compute Fourier coefficients and update est. frequency
function [FourierCoefficients,estimatedFrequency] = ...
    computeFourierCoefficients(timeSeries,dt,estimatedFrequency,H)

% Apply FFT to time series
L = size(timeSeries,1);
freqs = (1/(L*dt))*(0:floor(L/2));
ampls = fft(timeSeries)/L;
% Truncate and convert to half-spectrum
ampls(1,:) = real(ampls(1,:));
ampls(2:length(freqs),:) = 2*ampls(2:length(freqs),:);

% Determine index to fundamental harmonic
[~,idx_1H] = min(abs(freqs-estimatedFrequency));

% Check if excitation spectrum has local maximum near identified
% excitation frequency
inds = find((0.85*estimatedFrequency<freqs)&(freqs<1.15*estimatedFrequency));
[~,idx_tmp] = max(abs(ampls(inds,1)));
if idx_tmp==1 || idx_tmp==length(inds)
    warning(['ATTENTION: There is no local maximum of excitation ' ...
        'spectrum within +/-15 percent of' ...
        ' the estimated steady excitation frequency of ' ...
        num2str(estimatedFrequency) '.' ' This usually indicates '...
        'inconsistent data or a poor signal quality/resolution.']);
end

% Update estimated frequency (closest to mean instantaneous one)
estimatedFrequency = freqs(idx_1H);

% Fourier coefficients are the amplitudes corresponding to integer 
% multiples of the fundamental frequency
FourierCoefficients = ampls((idx_1H-1)*(0:H)+1,:);
end
