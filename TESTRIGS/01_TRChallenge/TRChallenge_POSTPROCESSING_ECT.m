%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script post-processes data acquired during Excitation Controlled 
% Testing of the benchmark system of the Tribomechadynamics Research
% Challenge. In the tests, the set value of the phase lag between response 
% and excitation was stepped, while the excitation level was kept fixed,
% using feedback control.
% 
% The challenge is described in [1]. The tests were part of Project 3 of 
% the Tribomechadynamics Research Camp 2022 in Stuttgart [2]. The test 
% methods are described in [3]. CAD models, technical drawings and design 
% documentation are available in [4].
% 
% In this script, one selects 1 out of 4 configurations and specifies a
% tested excitation level. This script then post-processes the 
% corresponding data and generates a figure similar to Figure 11 in [3].
% 
% The corresponding data has to be available in the BASEFOLDER specified 
% by the user. The data can be downloaded from [5]. For information on the 
% data file names and content structure, see README in [5].
% 
% REFERENCES
% [1] http://tmd.rice.edu/ tribomechadynamics-research-challenge-2021/
% [2] http://tmd.rice.edu/tribomechadynamics-research-camp/2022-graduate-projects/
% [3] https://arxiv.org/abs/2403.07438
% [4] https://doi.org/10.18419/darus-3147
% [5] https://doi.org/10.18419/darus-4484
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of NLtest available via
% https://github.com/maltekrack/NLtest.
% 
% COPYRIGHT (C) 2024
%   Malte Krack (malte.krack@ila.uni-stuttgart.de) 
%   Maren Scheel (maren.scheel@ila.uni-stuttgart.de)
%   and the co-authors of [3].
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars;
close all;
clc;
addpath('../../SRC');
%% KEY USER INPUT

% Folder with measurements
BASEFOLDER = '';

% Specify configuration [1|2|3|4]
CONFIG = 4;

% Specify base acceleration level in m/s^2. Available values:
%       configuration 1: [1|2|3|4]
%       configuration 2: [2|3|4|5]
%       configuration 3: [1|2|3|4]
%       configuration 4: [1|2|3|4|5]
LEVEL = 2;

% Specify options for post-processing
options.target_number_of_last_periods_of_hold_phase_to_process = 300;
%% LOAD DATA FROM FILES AND PREPARE FOR PROCESSING

% Read representative fundamental frequency for configuration and set as
% nominal
modes = readmatrix([BASEFOLDER '/config' num2str(CONFIG) '/Config' ...
    num2str(CONFIG) '_linear_complex_modes']);
options.nominalFreq = modes(end,1);

% Generate data and step file name
tmp = [BASEFOLDER '/config' num2str(CONFIG) '/Config' ...
    num2str(CONFIG) '_ECT'];
dataFileName = [tmp '_' num2str(LEVEL) 'm_s2'];
stepFileName = [tmp '_phase_steps'];

% Read data file
data = readmatrix(dataFileName);

% Read step file
steps = readmatrix(stepFileName);

% Interpret data
time_s = data(:,1);                         % time in s
velocity_m_s = data(:,2:18);                % velocity in m/s
instantaneousFrequency_Hz = data(:,19);     % instant. frequency in Hz

% Determine base velocity (excitation) and relative velocity at sensor 
% location 8 (response)
excitation_m_s = ( velocity_m_s(:,16) + velocity_m_s(:,17) ) / 2;
relativeVelocity_m_s = velocity_m_s(:,1:15)-repmat(excitation_m_s,1,15);
response_m_s = relativeVelocity_m_s(:,8);
%% ESTIMATE FOURIER COEFFICIENTS FROM ECT (end of hold phases)
[frequency_Hz,Excitation_m_s,Response_m_s] = ...
    estimateFourierCoefficientsFromSteppedSine(steps,time_s,...
    instantaneousFrequency_Hz,...
    excitation_m_s,response_m_s,[],...
    options);
%% ILLUSTRATE FREQUENCY RESPONSE CURVE (Figure 11 in [3])

% Response level: magnitude of fundamental harmonic displacement at sensor
% location 8
response_level_mm = 1e3*abs(Response_m_s(:,2))./(2*pi*frequency_Hz);

% Illustrate results
figure('name',['Excitation level ' num2str(LEVEL) ' m/s^2']);
plot(frequency_Hz,response_level_mm,...
    'mx','markersize',10)
ylabel('displacement amplitude (panel center) in mm');
xlabel('\Omega /(2\pi) in Hz');
