%========================================================================
% DESCRIPTION: 
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
% In a PRT, the target is a fixed (resonant) phase lag between excitation 
% and response, while the excitation level is stepped. In a RCT, the 
% response level is kept fixed, while the frequency or the phase lag is
% stepped. In an ECT, the excitation level is kept fixed, while the
% frequency or the phase lag is stepped.
% The excitation is commonly the voltage input to the shaker amplifier, or 
% a feedback-controlled applied force or base motion. The response is 
% commonly acquired with motion sensors (displacement, velocity, 
% acceleration).
% 
% A typical step-type single-frequency nonlinear vibration test consists of
% alternating transient phases (ramps intended to achieve a smooth 
% transition plus the time it takes to settle on a steady state) and 
% steady-state phases, denoted "hold phases" in the following. This 
% function selects data from the steady-state phases by means of the 
% Boolean variable flagSettling. If flagSettling is true, the associated 
% time instance is a treated as steady state. In other words, this function 
% does not check for steady state, instead it relies on the specified 
% categorization. The state-of-the-art implementation of the 
% BackboneTracker, for instance, carries out such a categorization
% during the test based on an automatic settling detection in accordance
% with user specified tolerance parameters. In older implementations, a 
% priori specified wait times were used, which were deemed sufficiently
% long for transients to have decayed, inducing a categorization into
% presumed transient and hold phases.
%
% To deal also with phase controlled tests, where the instantaneous
% frequency is not an imposed piecewise constant but a dynamic quantity, 
% the variable 'instFreq' is to be provided as time series of the same 
% dimension as the discrete time vector ('time'). Further mandatory inputs 
% are a scalar excitation ('exc') time series, a scalar or matrix response   
% ('resp') time series and a scalar Boolean time series 'flagSettling'. 
% Additional time series of the instantanoues excitation phase (integral of
% the excitation frequency) can be provided as optional input argument 
% ('instPhase'). If it is not provided, it is obtained by numerical
% integration of 'instFreq', if needed.
% 
% The steady excitation frequency is estimated as the mean value of 
% 'instFreq' in the last part of a given hold phase over an integer number 
% of periods. The length of this time span is determined in terms of 
% excitation phase (if instPhase is provided) as 2*pi times 
%       'target_number_of_last_periods_of_hold_phase_to_process'.
% Otherwise, it is determined as 'nominalFrequency' times 
%       'target_number_of_last_periods_of_hold_phase_to_process'.
% Once the steady excitation frequency is estimated, this time span is 
% updated.
%
% Two options for estimating Fourier coefficients are implemented: The
% first option is to apply an adaptive filter to the full time series of 
% excitation and response signals and to subsequently average the estimated 
% Fourier coefficients over the specified number of last periods of the 
% hold phases. The second option is to apply the FFT to the corresponding
% time span.
%
% INPUT
%   VARIABLE                MEANING                        TYPE
%   time                    time in s                      n x 1 double
%   instFreq                instantaneous frequency in Hz  n x 1 double
%   exc                     excitation time series         n x 1 double
%   resp                    response time series           n x nSens double
%   flagSettling            indicates hold phases          n x 1 double
%   opt		                options                        structure
%      FIELD NAMES
%      computeFourier...    'adaptiveFilter' or            string
%       Coefficients          'FFT'
%
%      target_number_...    number of periods to be        positive integer
%       of_last_periods_...  used for averaging or 
%       of_hold_phase_to_... estimating Fourier 
%       process              coefficients
% 
%      H                    harmonic order of Fourier      positive integer
%                           coefficients to be estim. 
% 
%      nominalFrequency      nominal frequency, used       positive double
%                            for estimating time span
%                            corresponding to target
%                            number of periods (if 
%                            varargin{1} is empty)
%
%      samplingTimeNew      new time increment used for    double
%                            interpolation   
%
%      omLP                 cutoff frequency of            positive double
%                            adaptive filter
%   
%   varargin{1}             instantaneous phase            n x 1 double
% 
% 
% OUTPUT
%   VARIABLE        MEANING                              TYPE
%     freq          steady excitation frequency          nHold x 1 double
%     
%     EXC           complex Fourier coefficients of exc  nHold x H+1 double
%     
%     RESP          ... of response                      (nHold x H+1 x 
%                                                           nSens) double
%     
%     indPROCESS    selected samples for evaluation      nHold x n Boolean
%     
%     time,         interpolated output time series      (same as input)
%     instFreq,exc,  
%     resp, instPhase
%
%     exc_FC        estimated, time-varying Fourier      n x H+1 double
%                    coefficient of the excitation
%     resp_FC        ... of response                     n x H+1 x nSens 
%                                                                   double
% ========================================================================
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

function [freq,EXC,RESP,indPROCESS,time,instFreq,exc,resp,instPhase,...
    exc_FC,resp_FC] = estimateFourierCoefficientsFromSteppedSine(...
    time,instFreq,exc,resp,flagSettling,opt,varargin)
    
    %% Handle and check user input
    
    % Check if inputs are given as column vectors
    n = size(time,1);
    if size(time,2)>1||size(instFreq,2)>1||size(exc,2)>1||...
            size(flagSettling,2)>1
        error('Expecting column vectors.');
    end

    % Check that options are given as struct
    if ~isstruct(opt)
        error('Expecting input of type structure for options.');
    end
    
    % Handle additional time series
    if nargin>=7 && ~isempty(varargin{1})
        instPhase = varargin{1};
    else
        instPhase = zeros(n,0);
    end
    
    % Check consistency of input dimensions
    if size(instFreq,1)~=n||size(instPhase,1)~=n||...
            size(exc,1)~=n||size(resp,1)~=n||size(flagSettling,1)~=n
        error('Inconsistent input time series dimensions.');
    end
    
    % Check for reasonable values
    if any(isnan([time,exc,resp]))
        error('NaN values not allowed within input.');
    end
    if ~all(flagSettling==1|flagSettling==0)
        error('flagSettling must be Boolean');
    end

    % If a target number of periods is not specified for hold phases, 
    % use default value
    if ~isfield(opt,'target_number_of_last_periods_of_hold_phase_to_process')
        opt.target_number_of_last_periods_of_hold_phase_to_process = 1;
        warning(['Target number of last periods of hold phase to process'...
            ' not provided. Using default value of ' num2str(...
            opt.target_number_of_last_periods_of_hold_phase_to_process)]);
    else
        % Check provided number of periods
        if numel(opt.target_number_of_last_periods_of_hold_phase_to_process)~=1||...
                opt.target_number_of_last_periods_of_hold_phase_to_process<=0||...
                mod(opt.target_number_of_last_periods_of_hold_phase_to_process,1)~=0
            error('Expecting positive integer for number of harmonics.')
        end
    end
    
    % Check method for computing Fourier coefficients
    if ~strcmp(opt.computeFourierCoefficients,'adaptiveFilter') && ...
            ~strcmp(opt.computeFourierCoefficients,'FFT')
        error(['Expecting either string "adaptiveFilter" or "FFT" '... 
            'for option how to compute Fourier coefficients.'])
    end
    
    % Check provided nominal frequency
    if isfield(opt,'nominalFrequency')
        if numel(opt.nominalFrequency)~=1||opt.nominalFrequency<=0
            error('Expecting positive scalar for nominal frequency.')
        end
    else
        % None provided, estimate it
        opt.nominalFrequency = mean(abs(instFreq(~isnan(instFreq))));
        fprintf(['No field nominalFrequency of options structure provided. ' ...
            'Estimating it from mean absolute value of instFreq as ' ...
            num2str(opt.nominalFrequency) ' (same unit as instFreq).\n']);
    end
    % Set time increment for interpolation
    if isfield(opt,'samplingTimeNew') 
        if ~isscalar(opt.samplingTimeNew) || ...
                ~isreal(opt.samplingTimeNew) || opt.samplingTimeNew<=0
            error('Provided new sample time must be real positive scalar.')
        end
        dttmp = opt.samplingTimeNew;
    else 
        dttmp = mean(diff(time)); % needed for check of Nyquist criterion
    end
    % Check provided number of harmonics
    if isfield(opt,'H')
        if numel(opt.H)~=1||opt.H<=0||...
                mod(opt.H,1)~=0
            error('Expecting positive integer for number of harmonics.')
        end
        if opt.H > floor(1/dttmp/opt.nominalFrequency/2.56)-1
            warning(['Frequency of highest harmonic to be estimated is '...
                'close to or above maximum according to Nyquist criterion. '...
                'Please verify or decrease harmonic order.'])
        end
    else
        % None specified, select maximum. The factor is 2.56 instead of the 
        % Nyquist limit of 2 because anti-aliasing filters commonly restrict
        % the useful frequency bandwidth (see e.g. Allemang and Avitabile
        % (2022): Handbook of experimental structural dynamics).
        opt.H = floor(1/dttmp/opt.nominalFrequency/2.56)-1;
    end
    
    % Check adaptive filter cutoff frequency
    if isfield(opt,'omLP')&&~isempty(opt.omLP)
        if opt.omLP<=0
            error('Expecting positive number for low-pass filter frequency.')
        end
    end
    
    %% Define hold phases

    % Determine start and end time instants of hold phases
    holdSTART = time(diff(flagSettling)>0);
    holdEND = time(diff(flagSettling)<0);

    % Check if all hold phases have start and end
    if numel(holdSTART)~=numel(holdEND) || holdSTART(1)>holdEND(1)
        error(['Each hold phase (indicated by flagSettling true) ' ...
            'must have a start and an end in the provided time span.']);
    end

    % Store time intervals of hold phases
    holdPhase = [holdSTART holdEND];

    %% Determine sampling and ensure equidistant sampling
    dt = mean(diff(time));
    if any(diff(time)<0)
        error('Forward sampling required.');
    elseif max(abs((diff(time)/dt-1)))>1e-6||...
            isfield(opt,'samplingTimeNew')
        % NOTE: With some real-time systems, the time step varies by about
        % +/-3%, which can distort the estimation of Fourier coefficients.
        % In that case, an interpolation is proposed. This is also relevant
        % in case of simulations with variable time steps. The new time
        % vector and interpolated data can then be an important output of
        % this function.
        fprintf(['Non-equidistant samples detected or new time increment'...
            ' provided. Interpolating...\n If no time '...
            'increment is specified, the mean time increment is used.\n ']);
        timetmp = time;
        if ~isfield(opt,'samplingTimeNew')
            dt = mean(diff(time));
        else 
            dt = opt.samplingTimeNew;
        end
        time = (time(1):dt:time(end)).';
        n = size(time,1);
        
        instFreq = interp1(timetmp,instFreq,time);
        if ~isempty(instPhase)
            instPhase = interp1(timetmp,instPhase,time); 
        end
        exc = interp1(timetmp,exc,time);
        resp = interp1(timetmp,resp,time);
        fprintf('Done.\n');
    else
        fprintf(['Equidistant samples detected. Interpolation is not ' ...
            'conducted.\n']);
    end
    
    %% Determine end of hold phases with resampled time vector
    [~,idxLast] = min(abs(repmat(time,[1 length(holdPhase(:,2))])...
        - holdPhase(:,2).'));
    numHoldPhases = length(idxLast);
    
    %% Determine Fourier coefficients and evaluate in each hold phase

    % Initialize output
    freq = zeros(numHoldPhases,1);
    EXC = zeros(numHoldPhases,opt.H+1);
    RESP = zeros(numHoldPhases,opt.H+1,size(resp,2));
    indPROCESS = false(numHoldPhases,n);

    switch opt.computeFourierCoefficients 
        case 'adaptiveFilter'
            % Determine instantaneous phase if not provided
            if isempty(instPhase)
                instPhase  = cumtrapz(time, 2*pi*instFreq);
            end

            % Apply adaptive filter
            fprintf('Applying adaptive filter to measurement data...\n');
            [~, exc_FC]  = computeFourierCoefficientsWithAF(exc,dt,...
                opt.H,instPhase,opt.omLP);
            resp_FC = NaN([size(exc_FC),size(resp,2)]);
            for jj = 1:size(resp,2)
                [~, resp_FC(:,:,jj)]  = ...
                    computeFourierCoefficientsWithAF(resp(:,jj),dt,...
                    opt.H,instPhase,opt.omLP);
            end
            fprintf('Done.\n');

        case 'FFT'
            fprintf('Applying FFT to measurement data.\n');

    end
            
    % Loop over hold phases
    for ihold=1:numHoldPhases

        % Estimate frequency and determine samples to be processed
        [isToBeProcessed,estimatedFrequency] = ...
            estimateFrequencyAnddetermineSamplesToBeProcessed(time,...
            instFreq,instPhase,holdPhase(ihold,:),opt);
           
        % Determine Fourier coefficients for each hold phase 
        switch opt.computeFourierCoefficients
            case 'adaptiveFilter'
                % Determine Fourier coefficients as mean over adaptive
                % filter output
                EXC(ihold,:) = mean(exc_FC(isToBeProcessed,:));
                if size(resp_FC,3)==1
                    RESP(ihold,:,:) = ...
                        mean(resp_FC(isToBeProcessed,:,:));
                else
                    RESP(ihold,:,:) = ...
                        squeeze(mean(resp_FC(isToBeProcessed,:,:)));
                end
            case 'FFT'
                % Apply FFT to each hold phase
                
                % Select and concatenate time series
                timeSeries = [exc(isToBeProcessed) resp(isToBeProcessed,:)];
            
                % Compute Fourier coefficients and update estimated frequency
                [FourierCoefficients,estimatedFrequency] = ...
                    computeFourierCoefficientsWithFFT(timeSeries,dt,...
                    estimatedFrequency,opt.H);

                % Store Fourier coefficients
                EXC(ihold,:) = FourierCoefficients(:,1);
                RESP(ihold,:,:) = ...
                    FourierCoefficients(:,1+(1:size(resp,2)));

                % In case of FFT, no instantaneous estimate of the Fourier
                % coefficients is available. Return empty arrays.
                if nargout>=10
                    exc_FC = [];
                    resp_FC = [];
                end
        end

        % Store indices of selected samples and estimated frequency
        indPROCESS(ihold,:) = isToBeProcessed;
        freq(ihold) = estimatedFrequency;

        % Diagnostic information on signal processing
        if ihold==1 || ihold==numHoldPhases
            n_samp = sum(indPROCESS(ihold,:));
            np_hold = (holdPhase(ihold,2)-holdPhase(ihold,1))*freq(ihold);
            np_eval = n_samp*dt*freq(ihold);
            if ihold==1
                fprintf(['Information on estimation of excitation frequency'...
                    ' and Fourier coefficients in FIRST hold phase:\n']);
            else
                fprintf(['Information on estimation of excitation frequency'...
                    ' and Fourier coefficients in LAST hold phase:\n']);
            end
            fprintf(['     Portion of hold phase used:         ' ...
                num2str(1e2*np_eval/np_hold) '%%.\n']);
            fprintf(['     Number of samples used per period:  ' ...
                num2str(round(n_samp/np_eval)) '.\n']);
        end

    end
end
%% NESTED FUNCTION: adaptive filter

function [filteredSignal,FourierCoefficient] = ...
    computeFourierCoefficientsWithAF(recordedSignal, dt, H, instPhase, omLP)
    % This function applies the adaptive filter (discrete form) to the 
    % given time series.
    %
    % INPUT
    %   VARIABLE                MEANING                        TYPE
    %   recordedSignal          recorded time series           n x 1 double
    %   dt                      sampling time                  double
    %   H                       number of consecutive          double
    %                            harmonics in the basis
    %   instPhase               instantaneous phase in radians n x 1 double
    %   omLP                    cutoff frequency of            double
    %                            adaptive filter
    % OUTPUT
    %   VARIABLE                MEANING                        TYPE
    %   filteredSignal          reconstructed time series      n x 1 double
    %   FourierCoefficient      estimated, time-varying        n x H+1 double
    %                            Fourier coefficients
    
    % Initialize output
    FourierCoefficient = zeros(H+1, size(recordedSignal,1));

    % Define auxiliary variables
    basisPositive = exp(+1i*(0:H).'.*(instPhase).');
    basisNegative = exp(-1i*(0:H).'.*(instPhase).');

    % Loop over time
    for ii = 1:length(recordedSignal)-1
        FourierCoefficient(:, ii+1) = FourierCoefficient(:, ii) + ...
            2 * omLP * dt * basisNegative(:,ii)*(...
            recordedSignal(ii) - real(...
            FourierCoefficient(:,ii).'*basisPositive(:,ii) ) );
    end

    % Reconstruct filtered signal
    filteredSignal = sum(real(FourierCoefficient.*basisPositive)).';

    % Return Fourier coefficients in expected format
    FourierCoefficient = FourierCoefficient.';
end
%% NESTED FUNCTION: Est. frequency and determine samples to be processed
function [isToBeProcessed,estimatedFrequency] = ...
    estimateFrequencyAnddetermineSamplesToBeProcessed(time,instFreq,instPhase,...
        holdPhase,opt)
    % This function estimates the excitation frequency in the current hold 
    % phase and determines indices of samples to be processed.
    %
    % INPUT
    %   VARIABLE                MEANING                        TYPE
    %   time                    time in s                      n x 1 double
    %   instFreq                instantaneous frequency in Hz  n x 1 double
    %   instPhase               instantaneous phase in radians n x 1 double
    %   holdPhase               start and end time of          1 x 2 double
    %                            hold phase    
    %   opt                     options                        struct
    %      FIELD NAMES
    %       target_number_...    number of periods to be     positive integer
    %       of_last_periods_...  used for averaging or 
    %       of_hold_phase_to_... estimating Fourier 
    %       process              coefficients
    %      nominalFrequency          nominal frequency, used        double
    %                            for estimating time span
    %                            corresponding to target
    %                            number of periods
    % OUTPUT
    %   VARIABLE                MEANING                        TYPE
    %   isToBeProcessed         indicates samples of          n x 1 logcial
    %                            hold phase
    %   estimatedFrequency      averaged frequency in Hz       double

    % Check if instantaneous phase is given
    if ~isempty(instPhase) 
        % instantaneous phase given:
        % periods are determined by determining phase difference
        index_end = find(time<=holdPhase(2),1,'last');
        phase_start = instPhase(index_end)-...
            2*pi*opt.target_number_of_last_periods_of_hold_phase_to_process;
        index_start = find(instPhase>=phase_start,1,'first');
        isToBeProcessed = false(size(time));
        isToBeProcessed(index_start:index_end)=true;
        
        estimatedFrequency = mean(instFreq(isToBeProcessed&~isnan(instFreq)));
    else 
        % no instantaneous phase given: 
        % periods are estimated by averaging instantaneous frequency
        % estimate samples based on nominal frequency
        isToBeProcessed = time<=holdPhase(2) & ...
            time>(holdPhase(2)-...
            opt.target_number_of_last_periods_of_hold_phase_to_process/...
            opt.nominalFrequency);
        
        % Estimate frequency along those samples
        estimatedFrequency = mean(instFreq(isToBeProcessed&~isnan(instFreq)));
        
        % Check for validity and consistency
        if isnan(estimatedFrequency)
            error('No valid instantaneous frequency at steady state.');
        end
        
        % Update selected samples according to updated frequency estimate
        isToBeProcessed = time<=holdPhase(2) & ...
            time>(holdPhase(2)-...
            opt.target_number_of_last_periods_of_hold_phase_to_process/...
            estimatedFrequency);
    
    end

end
%% NESTED FUNCTION: Compute Fourier coefficients and update est. frequency
function [FourierCoefficients,estimatedFrequency] = ...
    computeFourierCoefficientsWithFFT(timeSeries,dt,estimatedFrequency,H)
    % applies FFT to time series and extracts fundamental frequency
    % (close to the estimated frequency), Fourier coefficent of zero-th 
    % harmonic, fundamental harmonic and H-1 higher harmonics. 

    % Apply FFT to time series
    L = size(timeSeries,1);
    freqs = (1/(L*dt))*(0:floor(L/2));
    ampls = fft(timeSeries)/L;
    % Truncate and convert to half-spectrum
    ampls = ampls(1:length(freqs),:);
    ampls(1,:) = real(ampls(1,:));
    ampls(2:length(freqs),:) = 2*ampls(2:length(freqs),:);
    
    % Determine index to fundamental harmonic
    [~,idx_1H] = min(abs(freqs-estimatedFrequency));
    
    % Check if excitation spectrum has local maximum near identified
    % excitation frequency
    inds = find((0.85*estimatedFrequency<freqs)&...
        (freqs<1.15*estimatedFrequency));
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