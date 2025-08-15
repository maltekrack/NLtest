%========================================================================
% DESCRIPTION: 
% Before running this script, adjust and run 'initializeRealExperiment.m'
% to build the Simulink model required for running the physical experiment
% and saving the settings file.
% This script loads the settings file, loads the compiled Simulink model 
% to your hardware, and sets up the communication with it. Specifically, 
% it opens Matlab windows for signal visualization and the GUI to 
% start/stop the backbone test. After completion of the test, the recorded 
% data is saved in a .mat file for post-processing.
% 
% The Simulink model 'realExperiment.slx' and this script have been 
% designed for the dSPACE MicroLabBox (version 1, ds1202). To adjust this 
% script to your own experiment, ideally, you just have to specify the 
% 'timeStampSettings', and the 'dSpaceRelease'. The tool should work with 
% a reasonable range of recent dSPACE and Matlab releases.
% 
% The structure of this script follows the dSpace ASAM XIL API example in 
% the installation folder under "\Demos\MAPort\Matlab". 
% See also
%   https://www.dspace.com/en/inc/home/applicationfields/portfolio/...
%       ...xil_api/xil_api_standard.cfm#179_24041.
% 
% See README.md and DOC/BackboneTracking.md for further information.
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
close all hidden;
clc;
filepath = fileparts(mfilename("fullpath"));
cd(filepath);
%% Import features needed for hardware communication
import ASAM.XIL.Interfaces.Testbench.*;
import ASAM.XIL.Interfaces.Testbench.Common.CaptureResult.*;
import ASAM.XIL.Interfaces.Testbench.Common.Capturing.*;
import ASAM.XIL.Interfaces.Testbench.Common.Capturing.Enum.*;
import ASAM.XIL.Interfaces.Testbench.Common.Error.*;
import ASAM.XIL.Interfaces.Testbench.Common.ValueContainer.*;
import ASAM.XIL.Interfaces.Testbench.Common.WatcherHandling.*;
import ASAM.XIL.Interfaces.Testbench.MAPort.*;
import ASAM.XIL.Interfaces.Testbench.MAPort.Enum.*;
%% Load settings and specify options

% Specify the settings file saved when running the script
% 'InitializeRealExperiment.m'.
timeStampSettings = '2025_07_16-13_41'; 
fileNameSettings = sprintf('settings_%s',timeStampSettings);
BASEFOLDER = 'DATA';
load([BASEFOLDER filesep fileNameSettings],'settings');

%--------------------------------------------------------------------------
% hardware-related options
%--------------------------------------------------------------------------
% Specify the configuration file. Edit this file if you use hardware 
% different from dSPACE MicroLabBox ds1202, or you have renamed the 
% Simulink model.
maPortConfigFile = '.\MAPortConfig_backbone.xml';
% Specify the dSPACE release in the format 'YYYY-A/B'. This is needed to 
% use the appropriate ASAM XIL API assembly commands.
dSpaceRelease = '2022-B'; % specify as 'YYYY-A/B'
add_netAssembly(dSpaceRelease);

%--------------------------------------------------------------------------
% options for data handling and storage
%--------------------------------------------------------------------------
% Set name of temporary file to which data is written during the test.
recordingFileTmp = 'measurementDataTmp.mf4';
% Set name of .mat file to which data is saved after completion of the
% test. To avoid unintentionally overwriting measurement data, the file
% name contains a time stamp.
matFile = sprintf('DATA/measurement_%s', ...
    string(datetime('now','format','yyyy_MM_dd-HH_mm')));

% Plotting every sample live might exceed hardware capacity. If you have 
% issues with the live plot, try a higher value. If the plots appear too 
% undersampled, try a lower value.
% NOTE: The sampling rate of recorded data is equal to the hardware 
% sampling rate, independent of this parameter. Also, when the test is 
% over, the time series plots are refreshed using all recorded samples.
downsamplingFactorPlot = 20; 

%--------------------------------------------------------------------------
% Define the variables to be captured (either online or recorded)
%--------------------------------------------------------------------------
% All variables that are recorded and/or displayed by default are handled
% separately and have appropriate names. The two lists are independent, 
% i.e., each variable can be in only one of the lists or in both. 
%
% The easiest way to add variables to be recorded (e.g. additional
% responses) is to add their names to 'variableName_additional'. The
% signal will then automatically be part of the .mat file which is stored 
% after the test. Variables in that list are not plotted by default but are
% available at runtime so that the user can add them to a live plot.
%
% Please note that if you introduce new variables separately (i.e. not as a
% part of 'variableName_additional'), changes at other locations in the
% code are required to make the variables available online or to
% record them.
% 
% All existing internal variable names can be found in the .trc file 
% created when compiling the Simulink model. 
% The model is configured in such a way that all labelled signals (with an 
% annotation at the arrow connecting two Simulink blocks) are listed as 
% `Labels/<signal label>`.

% Specify names of main backbone tracking variables.
variableName_excitation             = 'Model Root/excitation/excitation';
variableName_response               = 'Model Root/response/response';
variableName_frequency_Hz           = 'Labels/frequency_Hz';
variableName_phaseError_deg         = 'Labels/phaseError_deg';
variableName_relativeAmplitudeError = 'Labels/relativeAmplitudeError';
variableName_displacementAmplitude  = 'Labels/displacementAmplitude';
variableName_settling               = 'Labels/settling';
variableName_phase_rad              = 'Labels/phase_rad';
variableName_phaseCtrlOn            = 'Labels/phaseCtrlOn';
variableName_ampCtrlOn              = 'Labels/ampCtrlOn';
variableName_currentLevel           = ['Model Root/BackboneTracker/' ...
    'BackboneTracker/Model Root/steppingAndAmplitudeController/currentLevel'];
variableName_finished               = 'Labels/finished';

% Specify names of modal parameters.
variableName_backboneAmplitude      = 'Labels/backboneResponseAmplitude';
variableName_modalFrequency         = 'Labels/modalFrequency_Hz';
variableName_modalDampingRatio      = 'Labels/modalDampingRatio';

% Specify names of additional variables to be recorded. In the provided 
% version of this script, all responses for which a 
% 'settings.responseQuantity' field has been set, are accounted for. One of
% these is the drive point response that has already been specified in the
% 'variableName_response'. The others are specified here as automatically, 
% recorded and added to the online list to be available at runtime.
% If only excitation and drive point response are measured, this list will 
% be empty.
variableName_additional = cell(1,numel(settings.responseQuantity)-1);
idxAddResp = 0;
for idxResp=1:numel(settings.responseQuantity)
    if idxResp==settings.indexSensorDrivePoint
        continue;
    else
        idxAddResp = idxAddResp + 1;
        variableName_additional{idxAddResp} = ...
            ['Model Root/response_' num2str(idxResp) ...
            '/response_' num2str(idxResp)];
    end
end

%% Initialize and conduc real-time experiment

% Initialize variables for accumulating online data.
time_s                  = [];
excitation              = [];
response                = [];
frequency_Hz            = [];
phaseError_deg          = [];
relativeAmplitudeError  = [];
displacementAmplitude   = [];
settling                = [];
backboneAmplitude       = [];
modalFrequency_Hz       = [];
modalDampingRatio       = [];
additionalSignals       = cell(size(variableName_additional));

% This variable is required to detect the transition to the next point.
lastSampleSettled = 0; % 

try 
    %----------------------------------------------------------------------
    % Initialize test bench.
    %----------------------------------------------------------------------
    [myMAPortFactory, myCapturingFactory, ~] = ...
        initialize_testbench(dSpaceRelease);

    %----------------------------------------------------------------------
    % Create and configure a MAPort object.
    %----------------------------------------------------------------------
    fprintf('Creating MAPort...\n');
    MAPort = myMAPortFactory.CreateMAPort('DemoMAPort');
    fprintf('...done\n');
    maPortConfig = MAPort.LoadConfiguration(maPortConfigFile);
    fprintf('Configuring MAPort...\n');
    MAPort.Configure(maPortConfig, true);    
    fprintf('...done\n');
    
    %----------------------------------------------------------------------
    % Create and initialize a Capture object.
    %----------------------------------------------------------------------
    masterTask              = 'DataCapture';
    Capture_online          = MAPort.CreateCapture(masterTask);
    CaptureWriter_online    = ...
        myCapturingFactory.CreateCaptureResultMemoryWriter();
    Capture_recorder        = MAPort.CreateCapture(masterTask);
    CaptureWriter_recorder  = ...
        myCapturingFactory.CreateCaptureResultMDFWriterByFileName(...
        recordingFileTmp);

    %----------------------------------------------------------------------
    % Apply lists of measured and recorded signals.
    %----------------------------------------------------------------------
    
    % The online variables are available while the experiment is running.
    % Create online variable list.
    variableList_online = ...
        NET.createGeneric('System.Collections.Generic.List', ...
        {'System.String'});
    % Add variables.
    variableList_online.Add(variableName_excitation);
    variableList_online.Add(variableName_response);
    variableList_online.Add(variableName_frequency_Hz);
    variableList_online.Add(variableName_phaseError_deg);
    variableList_online.Add(variableName_relativeAmplitudeError);
    variableList_online.Add(variableName_displacementAmplitude);
    variableList_online.Add(variableName_settling);
    variableList_online.Add(variableName_phaseCtrlOn);
    variableList_online.Add(variableName_ampCtrlOn);
    variableList_online.Add(variableName_backboneAmplitude);
    variableList_online.Add(variableName_modalFrequency);
    variableList_online.Add(variableName_modalDampingRatio);
    variableList_online.Add(variableName_currentLevel);
    variableList_online.Add(variableName_finished);        
    for ii = 1:length(variableName_additional)
        variableList_online.Add(variableName_additional{ii});
    end
    % Add list to capture object.
    Capture_online.Variables = variableList_online;
    % Specify downsampling rate of data for live plotting.
    Capture_online.Downsampling = downsamplingFactorPlot;
    
    % The recorder variables are written to a temporary file during the 
    % experiment and can be accessed (e.g. saved) after the experiment is
    % over. 
    % Create recorder variable list.
    variableList_recorder = ...
        NET.createGeneric('System.Collections.Generic.List',...
        {'System.String'});
    % Add varibles.
    variableList_recorder.Add(variableName_excitation);
    variableList_recorder.Add(variableName_response);
    variableList_recorder.Add(variableName_frequency_Hz);
    variableList_recorder.Add(variableName_phaseError_deg);
    variableList_recorder.Add(variableName_relativeAmplitudeError);
    variableList_recorder.Add(variableName_displacementAmplitude);
    variableList_recorder.Add(variableName_settling);
    variableList_recorder.Add(variableName_phase_rad);
    for ii = 1:length(variableName_additional)
        variableList_recorder.Add(variableName_additional{ii});
    end
    % Add list to capture object.
    Capture_recorder.Variables = variableList_recorder;

    % Read number of excitation levels.
    finalLevel = settings.nLevelsTotal;
    % Read test type.
    testType = settings.testTypeNumeric;
    
    %----------------------------------------------------------------------
    % Opening of plot and UI Windows
    %----------------------------------------------------------------------
    
    % Create and initialize data plot.
    fig = initialize_plots();
    % Create UI figure.
    [uifig, control_stop, control_start, ...
        lmp_settling, lmp_phase, lmp_amp, lmp_step,...
        nb_exc_level] = create_UI(finalLevel);
    % Define callbacks for UI buttons.
    control_stop.Callback = @(src,event) MAPort.StopSimulation();
    control_start.Callback = @(src,event) MAPort.StartSimulation();

    %----------------------------------------------------------------------
    % Start of experiment
    %----------------------------------------------------------------------
    experimentStarted = 0;
    fprintf('Waiting until experiment is running...\n');
    while (MAPort.State ~= MAPortState.eSIMULATION_RUNNING &&...
            ishghandle(uifig))
        pause(0.02);
    end
    
    if ishghandle(uifig)
        %------------------------------------------------------------------
        % Start of measuring and recording
        %------------------------------------------------------------------
        fprintf('\nStart of experiment.\n');
        experimentStarted = 1;
        experimentFinished = 0;

        % Enable/diable buttons in UI.
        control_start.Enable = 'off';
        control_stop.Enable = 'on';

        % Start recording.
        Capture_online.Start(CaptureWriter_online);
        Capture_recorder.Start(CaptureWriter_recorder);

        fprintf('Waiting until capture is running...\n');
        while (Capture_online.State ~= CaptureState.eRUNNING && ...
                ishghandle(uifig))
            pause(0.02);
        end
        fprintf('Starting to fetch data...\n');
    
        %------------------------------------------------------------------
        % Plotting of measured signals online
        %------------------------------------------------------------------
        % While the experiment is running and the control window is open
        % data is fetched in intervals.
        while (ishghandle(uifig) && ...
                MAPort.State == MAPortState.eSIMULATION_RUNNING && ...
                ~experimentFinished)
            
            % Fetch results from capture object.
            CaptureResult = Capture_online.Fetch(false);
            lastIndex = CaptureResult.ExtractSignalValue(masterTask, ...
                variableName_excitation).FcnValues.Value.Count-1;
            % Read and display index of current excitation level.
            curr_exc_level = MAPort.Read(variableName_currentLevel);
            nb_exc_level.String = sprintf('%2.0f / %2.0f',...
                curr_exc_level.Value,finalLevel);
            % Check if experiment is finished.
            experimentFinished = MAPort.Read(variableName_finished).Value;
            % Append newly fetched data to already available one.
            time_s                  = [time_s, ...
                CaptureResult.ExtractSignalValue(masterTask, ...
                variableName_excitation).XVector.Value.ToArray().double];
            excitation              = [excitation, ...
                CaptureResult.ExtractSignalValue(masterTask, ...
                variableName_excitation).FcnValues.Value.ToArray().double];
            response                = [response, ...
                CaptureResult.ExtractSignalValue(masterTask, ...
                variableName_response).FcnValues.Value.ToArray().double];
            frequency_Hz            = [frequency_Hz, ...
                CaptureResult.ExtractSignalValue(masterTask, ...
                variableName_frequency_Hz).FcnValues.Value.ToArray().double];
            phaseError_deg          = [phaseError_deg, ...
                CaptureResult.ExtractSignalValue(masterTask, ...
                variableName_phaseError_deg).FcnValues.Value.ToArray().double];
            relativeAmplitudeError  = [relativeAmplitudeError, ...
                CaptureResult.ExtractSignalValue(masterTask, ...
                variableName_relativeAmplitudeError).FcnValues.Value.ToArray().double];
            displacementAmplitude   = [displacementAmplitude, ...
                CaptureResult.ExtractSignalValue(masterTask, ...
                variableName_displacementAmplitude).FcnValues.Value.ToArray().double];
            settling                = [settling, ...
                CaptureResult.ExtractSignalValue(masterTask, ...
                variableName_settling).FcnValues.Value.ToArray().double];
            for ii = 1:length(additionalSignals)
                additionalSignals{ii} = [additionalSignals{ii}, ...
                    CaptureResult.ExtractSignalValue(masterTask, ...
                    variableName_additional{ii}).FcnValues.Value.ToArray().double];
            end
            
            % Refresh colors of lamps.
            previouslySettled = lastSampleSettled;
            % Set settling lamp.
            if sum(CaptureResult.ExtractSignalValue(masterTask, ...
                    variableName_settling).FcnValues.Value.ToArray().double) ...
                    ~= 0 
                % If at least one sample within current data frame 
                % corresponds to a settled state, green-lit lamp.
                lmp_settling.Color = 'green';
            else
                lmp_settling.Color = 'red';
            end
            % Set phase control on/off lamp.
            if CaptureResult.ExtractSignalValue(masterTask, ...
                    variableName_phaseCtrlOn).FcnValues.Value.Item(lastIndex) ==1
                lmp_phase.Color = 'green';
            else
                lmp_phase.Color = 'red';
            end
            % Set amplitude control on/off lamp.
            % If this is not an amplitude-controlled test, leave lamp grey.
            if testType == 2
                if CaptureResult.ExtractSignalValue(masterTask, ...
                        variableName_ampCtrlOn).FcnValues.Value.Item(lastIndex) == 1
                    lmp_amp.Color = 'green';
                else
                    lmp_amp.Color = 'red';
                end
            end

            % Set amplitude stepping on/off lamp.
            if CaptureResult.ExtractSignalValue(masterTask, ...
                    variableName_ampCtrlOn).FcnValues.Value.Item(lastIndex) == 1
                lmp_step.Color = 'green';
            else
                lmp_step.Color = 'red';
            end

            % Append new modal data to already available one.
            if ~isnan(CaptureResult.ExtractSignalValue(masterTask, ...
                    variableName_modalDampingRatio).FcnValues.Value.Item(lastIndex))
                backboneAmplitude = [backboneAmplitude, ...
                    CaptureResult.ExtractSignalValue(masterTask, ...
                    variableName_backboneAmplitude).FcnValues.Value.Item(lastIndex)];
                modalFrequency_Hz = [modalFrequency_Hz, ...
                    CaptureResult.ExtractSignalValue(masterTask, ...
                    variableName_modalFrequency).FcnValues.Value.Item(lastIndex)];
                modalDampingRatio = [modalDampingRatio, ...
                    CaptureResult.ExtractSignalValue(masterTask, ...
                    variableName_modalDampingRatio).FcnValues.Value.Item(lastIndex)];
            end 

            % Refreshed plots to show current data.
            if ishghandle(fig)
                refreshdata
                drawnow
            else
                pause(0.1);
            end
        end
        % Inform user that data aquisition is over.
        fprintf('\n...Capturing finished.\n');
        
        %------------------------------------------------------------------
        % Stopping of measuring and experiment
        %------------------------------------------------------------------
        try 
            if Capture_online.State == CaptureState.eRUNNING
                Capture_online.Stop();
                Capture_recorder.Stop(); % stop the capturing
            end
        catch
            % State can't be read if experiment ended automatically
            % no stopping necessary.
        end
        try 
            if MAPort.State == MAPortState.eSIMULATION_RUNNING
                MAPort.StopSimulation(); % stop the experiment
            end
        catch
            % State can't be read if experiment ended automatically
            % no stopping necessary.
        end
        fprintf('Stopping Experiment\n');
        control_stop.Enable = 'off';
    
    else 
        % Control window was closed before start of experiment.
        fprintf('Control window closed.');
    end
    fprintf('\nSucessfully finished.\n');
    
    %----------------------------------------------------------------------
    % Attention: Make sure to dispose the Capture objects and the MAPort
    % object in any case to free system resources like allocated memory and
    % also resources and services on the platform.
    %----------------------------------------------------------------------
    Capture_online.Dispose();
    Capture_recorder.Dispose();
    MAPort.Dispose();
    
    %----------------------------------------------------------------------
    % Read mdf4 file to workspace and save data to mat file
    %----------------------------------------------------------------------
    if experimentStarted

        %------------------------------------------------------------------
        % Create CaptureResultMDFReader
        %------------------------------------------------------------------
        fprintf('\nCreating CaptureResultMDFReader...\n');
        CaptureReader = ...
            myCapturingFactory.CreateCaptureResultMDFReaderByFileName(...
            recordingFileTmp);
        fprintf('...done\n');

        %------------------------------------------------------------------
        % Read CaptureResult using the CaptureResultMDFReader
        %------------------------------------------------------------------
        fprintf('Reading CaptureResult from MDF file...\n');    
        CaptureResult = CaptureReader.Load();
        fprintf('...done\n');
    
        %------------------------------------------------------------------
        % Extract measured data from CaptureResult
        %------------------------------------------------------------------

        % Time can be extracted as the 'XVector' of any captured signal.
        % This yields the actual execution time (nominal time +- jitter) of
        % the sampling while the variable 'currentTime' yields the nominal 
        % time based on the sampling rate.
        time_s = CaptureResult.ExtractSignalValue(masterTask, ...
            variableName_excitation).XVector.Value.ToArray().double;

        % Extract data
        excitation              = CaptureResult.ExtractSignalValue(...
            masterTask, ...
            variableName_excitation).FcnValues.Value.ToArray().double;
        response                = CaptureResult.ExtractSignalValue(...
            masterTask, ...
            variableName_response).FcnValues.Value.ToArray().double;
        frequency_Hz            = CaptureResult.ExtractSignalValue(...
            masterTask, ...
            variableName_frequency_Hz).FcnValues.Value.ToArray().double;
        phaseError_deg          = CaptureResult.ExtractSignalValue(...
            masterTask, ...
            variableName_phaseError_deg).FcnValues.Value.ToArray().double;
        relativeAmplitudeError  = CaptureResult.ExtractSignalValue(...
            masterTask, ...
            variableName_relativeAmplitudeError).FcnValues.Value.ToArray().double;
        displacementAmplitude   = CaptureResult.ExtractSignalValue(...
            masterTask, ...
            variableName_displacementAmplitude).FcnValues.Value.ToArray().double;
        excitationPhase_rad     = CaptureResult.ExtractSignalValue(...
            masterTask, ...
            variableName_phase_rad).FcnValues.Value.ToArray().double;
        settling                = CaptureResult.ExtractSignalValue(...
            masterTask, ...
            variableName_settling).FcnValues.Value.ToArray().double; 
        for ii = 1:length(additionalSignals)
            additionalSignals{ii} = transpose(CaptureResult.ExtractSignalValue(...
                masterTask, ...
                variableName_additional{ii}).FcnValues.Value.ToArray().double);
        end

        % If plot figure is still open, refresh plots to show time series
        % with all acquired samples.
        if ishghandle(fig)
            refreshdata
            drawnow
        end

        % Convert data to format required for post processing.
        % time in s
        measurement.time_s                  = time_s.';      
        % excitation in measurement unit
        measurement.excitation              = excitation.';   
        % drive point response in measurement unit
        measurement.response                = response.';  
        % instantaneous frequency in Hz
        measurement.excitationFrequency_Hz  = frequency_Hz.';     
        % phase error in degree
        measurement.phaseError_deg          = phaseError_deg.';    
        % relative response amplitude error
        measurement.relativeAmplitudeError  = relativeAmplitudeError.';   
        % instantaneous phase in rad
        measurement.excitationPhase_rad     = excitationPhase_rad.';   
        % settling detection
        measurement.flagSettling            = settling.';     
        % additionally recorded signals in respective units
        measurement.additionalSignals       = additionalSignals;        
        % Save in .mat file.
        fprintf('Saving data to .mat file...\n');
        save(matFile, 'measurement');
        fprintf('...done\n');
    end
    
%--------------------------------------------------------------------------
% Exception handling
%--------------------------------------------------------------------------
       
catch e  
    if (exist('MAPort', 'var'))
        if (MAPort.State == MAPortState.eSIMULATION_RUNNING)
            fprintf('Stopping experiment\n');
            MAPort.StopSimulation(); % stop the simulation
        end
        % Make sure to dispose of the MAPort object in case of an error.
        MAPort.Dispose();
    end
    if (exist('Capture_online', 'var'))
		% Make sure to dispose of the Capture object in case of an error.
        Capture_online.Dispose();
    end
    if (exist('Capture_recorder', 'var'))
		% Make sure to dispose of the MAPort object in case of an error.
        Capture_recorder.Dispose();
    end
    
    if (isa(e, 'NET.NetException'))
        if (isa(e.ExceptionObject, ...
                'ASAM.XIL.Interfaces.Testbench.Common.Error.TestbenchPortException'))
            %--------------------------------------------------------------
            % Display the XIL API exception information to identify cause
            % of error
            %--------------------------------------------------------------
            fprintf('XIL API exception occurred:\n');
            fprintf('Code: %d\n', e.ExceptionObject.Code);
            fprintf('CodeDescription: %s\n', ...
                char(e.ExceptionObject.CodeDescription));
            fprintf('VendorCode: %d\n', e.ExceptionObject.VendorCode);
            fprintf('VendorCodeDescription: %s\n', ...
                char(e.ExceptionObject.VendorCodeDescription));
        else
            fprintf('.NET exception occurredd:\n');
            fprintf('Message: %s\n', char(e.message));
        end
    end
    
    rethrow(e);
end

%--------------------------------------------------------------------------
%% Nested functions
%--------------------------------------------------------------------------

function add_netAssembly(hardwareVersion)
    % This function adds the assembly version.
    % For MicroLabBoxes of version 2018-A or older, assemblies of version
    % 2.0.1.0 have to be used.
    versionYear = str2double(hardwareVersion(1:4));
    if versionYear < 2018 || strcmp(hardwareVersion,'2018-A')
        NET.addAssembly(['ASAM.XIL.Interfaces, Version=2.0.1.0, ' ...
            'Culture=neutral,PublicKeyToken=bf471dff114ae984']);
        NET.addAssembly(['ASAM.XIL.Implementation.TestbenchFactory, ' ...
            'Version=2.0.1.0, Culture=neutral, PublicKeyToken=fc9d65855b27d387']);
    else
        NET.addAssembly('ASAM.XIL.Interfaces');
        NET.addAssembly('ASAM.XIL.Implementation.TestbenchFactory');
    end
end

function [fig] = initialize_plots()
    % This function initializes data plots and returns figure handle.
    % Edit this function to modify the live plot.
    plotFontSize = 12;
    fig = figure('Name','Measured Data','NumberTitle','off',...
        'Position',[200, 100, 1000, 600]);
    % Initialize plots in tiled layout.
    tOuter = tiledlayout(fig,2,4, 'Padding','tight');
    
    % First tiled layput is for time data.
    hAx = nexttile(tOuter, [2,3]); 
    hAx.Visible = 'off';
    hP = uipanel(fig, 'Position', hAx.OuterPosition, 'BorderWidth', 0);
    t_Inner_1 = tiledlayout(hP,2,3,'TileSpacing','compact',...
        'Padding','compact');
    title(t_Inner_1, 'Measured Signals')
    xlabel(t_Inner_1, 'time in s','FontSize',plotFontSize);

    % excitation (tile 1)
    nexttile(t_Inner_1,1)
    plot(0,0,'b','XDataSource','time_s','YDataSource','excitation',...
        'LineWidth',1.0);
    ylabel('excitaiton','FontSize',plotFontSize);
    % response (tile 4)
    nexttile(t_Inner_1,4)
    plot(0,0,'b','XDataSource','time_s','YDataSource','response',...
        'LineWidth',1.0);
    ylabel('response','FontSize',plotFontSize);
    % excitation (tile 2)
    nexttile(t_Inner_1,2)
    plot(0,0,'b','XDataSource','time_s','YDataSource','frequency_Hz',...
        'LineWidth',1.0);
    ylabel('frequency (Hz)','FontSize',plotFontSize);
    % phase error (tile 5)
    nexttile(t_Inner_1,5)
    plot(0,0,'b','XDataSource','time_s','YDataSource','phaseError_deg',...
        'LineWidth',1.0);
    ylabel('phase lag error (deg)','FontSize',plotFontSize);
    % response amplitude (tile 3)
    nexttile(t_Inner_1,3)
    plot(0,0,'b','XDataSource','time_s',...
        'YDataSource','displacementAmplitude','LineWidth',1.0);
    ylabel('response amplitude','FontSize',plotFontSize);
    % amplitude control error (tile 6)
    nexttile(t_Inner_1,6)
    plot(0,0,'b','XDataSource','time_s',...
        'YDataSource','relativeAmplitudeError','LineWidth',1.0);
    ylabel('amplitude control error','FontSize',plotFontSize);

    % Second tiled layout is for modal data.
    hAx = nexttile(tOuter, [2,1]);
    hAx.Visible = 'off';
    hP = uipanel(fig, 'Position', hAx.OuterPosition, ...
        'BorderWidth', 0, 'BackgroundColor','w');
    t_Inner_2 = tiledlayout(hP,2,1,'TileSpacing','compact',...
        'Padding','compact');
    title(t_Inner_2,'Modal Properties')
    xlabel(t_Inner_2,'displacement amplitude','FontSize',plotFontSize);
    % modal frequency
    nexttile(t_Inner_2,1)
    plot(nan,nan,'b+', 'XDataSource','backboneAmplitude',...
        'YDataSource','modalFrequency_Hz','LineWidth',1.5);
    ylabel('modal frequency (Hz)','FontSize',plotFontSize);
    % modal damping
    nexttile(t_Inner_2,2)
    plot(nan,nan,'b+', 'XDataSource','backboneAmplitude',...
        'YDataSource','modalDampingRatio','LineWidth',1.5);
    ylabel('modal damping ratio','FontSize',plotFontSize);
end

function [myMAPortFactory, myCapturingFactory, myWatcherFactory] = ...
    initialize_testbench(hardwareVersion)
    % This function initializes the test bench according to the hardware
    % version.
    import ASAM.XIL.Implementation.TestbenchFactory.Testbench.*;
    
    % Create of factories necessary for running experiment and capturing
    % results.
    myTestbenchFactory = TestbenchFactory();
    myTestbench = myTestbenchFactory.CreateVendorSpecificTestbench(...
        'dSPACE GmbH', 'XIL API', hardwareVersion);
    myMAPortFactory = myTestbench.MAPortFactory;
    myCapturingFactory = myTestbench.CapturingFactory;
    myWatcherFactory = myTestbench.WatcherFactory;
end

function [uifig, control_stop, control_start, lmp_settling, lmp_phase, ...
    lmp_amp, lmp_step, nb_exc_level] = create_UI(final_exc_level)
    % This function creates the graphical user interface (GUI).
    % It returns a figure handle and handles to control elements.
    
    % Set text font size.
    ctrlFontSize = 12;

    % Create figure.
    uifig = uifigure('Name','GUI','NumberTitle','off', ...
        'Position',[1220, 280, 260, 420]);

    % Add descriptive text.
    uicontrol(uifig,'Style','text', ...
        'String','Backbone Tracking', ...
        'Position',[20,380,140,20], ...
        'FontSize',ctrlFontSize,'HorizontalAlignment','left');

    % Add buttons for start and stop.
    control_start = uicontrol(uifig,'String','START', ...
        'Position',[170,375,70,30], ...
        'FontSize',ctrlFontSize,'BackgroundColor','w');
    control_stop = uicontrol(uifig,'String','STOP', ...
        'Position',[170,335,70,30], ...
        'FontSize',ctrlFontSize,'BackgroundColor','w','Enable','off');

    % Create phase control on/off lamp.
    uicontrol(uifig,'Style','text','String','Phase Control', ...
        'Position',[20,270,130,20],'FontSize',ctrlFontSize,...
        'HorizontalAlignment','left');
    lmp_phase = uilamp(uifig,'Position',[170,270,20,20],'Color',[0.5, 0.5, 0.5]);
    % Create amplitude amplitude control on/off lamp.
    uicontrol(uifig,'Style','text','String','Amplitude Control', ...
        'Position',[20,230,170,20],'FontSize',ctrlFontSize,...
        'HorizontalAlignment','left');
    lmp_amp = uilamp(uifig,'Position',[170,230,20,20],'Color',[0.5, 0.5, 0.5]);
    % Create stepping on/off lamp.
    uicontrol(uifig,'Style','text','String','Stepping Started', ...
        'Position',[20,190,170,20],'FontSize',ctrlFontSize,...
        'HorizontalAlignment','left');
    lmp_step = uilamp(uifig,'Position',[170,190,20,20],'Color',[0.5, 0.5, 0.5]);

    % Create settling lamp.
    uicontrol(uifig,'Style','text','String','Settled State', ...
        'Position',[20,150,130,20],'FontSize',ctrlFontSize,...
        'HorizontalAlignment','left');
    lmp_settling = uilamp(uifig,'Position',[170,150,20,20],'Color',[0.5, 0.5, 0.5]);

    % Add information on progress.
    uicontrol(uifig,'Style','text','String','Backbone Point', ...
        'Position',[20,110,130,20],'FontSize',ctrlFontSize,...
        'HorizontalAlignment','left');
    nb_exc_level = uicontrol(uifig,'Style','text', ...
        'String',sprintf('%2.0f / %2.0f',0,final_exc_level), ...
        'Position',[170,110,70,20],'FontSize',ctrlFontSize,...
        'HorizontalAlignment','left');

    % Add image.
    if exist('backbone_tracker.svg','file')
        uiimage(uifig, 'ImageSource','backbone_tracker.svg',...
            "Position",[0,20,260,50]);
    else
        uicontrol(uifig,'Style','text','String','BackboneTracker', ...
        'Position',[0,20,260,50],'FontSize',18);
    end
end
