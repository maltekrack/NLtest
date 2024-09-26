%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function implements the well-known Circle Fit method for estimating
% frequency and damping ratio of a well-separated mode from the
% corresponding section of the frequency response function (FRF), see 
% e.g. [1].
% 
% The particular implementation is intended for Response Controlled 
% Testing of nonlinear structures, where the FRF has been acquired within a
% narrow frequency band around the resonance, possibly using
% phase control. In such a case, the common frequency spacing technique for
% estimating the natural frequency might not be useful, because the
% frequency window is so narrow and control imperfections spoil the quality
% of the FRF (as compared to that obtained with a common FRF estimator 
% applied to data acquired under random broadband testing in the linear 
% regime). Instead, the natural frequency is simply obtained as the phase 
% resonant point of the (centralized) FRF. This assumes 
% that the FRF is of appropriate type, e.g. receptance, and the mode is
% sufficiently real!
% 
% % REFERENCES
% [1] Maia; Silva (1997): Theoretical and Experimental Modal Analysis.
% 
% INPUT
%   VARIABLE    MEANING                        TYPE
%   freq        frequency                      L x 1 double
%   FRF         complex frequency              L x 1 complex double
%                response function
%   varargin{1}  flag whether to plot results  Boolean
% 
% OUTPUT
%   VARIABLE                    MEANING                     TYPE
%     modalFrequency            modal frequency             double
%     modalDampingRatio_mean    mean of damping ratio       double
%     modalDampingRatio_std     standard deviation of ...   double
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [modalFrequency,modalDampingRatio_mean,modalDampingRatio_std] = ...
    circleFit(freq,FRF,varargin)
%% Handle and check user input

% Check if inputs are given as column vectors, and data is valid
L = size(freq,1);
if size(freq,2)>1||size(FRF,2)>1
    error('First two input arguments must be column vectors.');
end
if size(FRF,1)~=L
    error('FRF must be of same size as frequency vector.');
end
if any(isnan([freq;FRF]))
    error('NaN values not allowed within input.');
end

% Handle optional flag whether to show results
if nargin>2 && ~isempty(varargin{1}) && isscalar(varargin{1})
    illustrateCircleFit = varargin{1}==1;
else
    illustrateCircleFit = false;
end
%% Fit a circle through the FRF in the complex plane and centralize FRF

% Determine location and radius of circle using least-squares minimization
%   circle equation: (x-xc)^2+(y-yc)^2-R^2 = 0
%   expanded: x^2 + y^2 - R^2 + xc^2 + yc^2 - 2*x*xc - 2*y*yc = 0
%   Unknown are R, xc, yc. Reformulation as linear equation:
%             x*a(1) + y*a(2) + a(3) = -(x^2+y^2) 
%       with a(1) = -2*xc, a(2) = -2*yc, a(3) = xc^2 + yc^2 - R^2,
%       which we can solve easily in a least-squares sense.
x = real(FRF); y = imag(FRF);
a = [x y ones(size(x))] \ -(x.^2+y.^2);
xc = -a(1)/2;
yc = -a(2)/2;
R = sqrt(xc^2+yc^2-a(3));

% Centralize FRF around origin in complex plane
FRF0 = FRF - (xc+1i*yc);

%% Estimate natural frequency using frequency spacing technique
% 
% % Phase angle of centralized FRF
% th = unwrap(angle(FRF0));
% 
% % Define equidistant frequency samples and interpolate
% freq_ = linspace(freq(1),freq(end),max(10*length(freq),1e2));
% th_ = interp1(freq,th,freq_,'PCHIP');
% 
% % Determine point with largest phase spacing and use as natural frequency
% [~,in_] = max(abs(diff(th_)));
% [~,in] = min(abs(th-th_(in_)));
% modalFrequency = freq(in);
%% Estimate natural frequency from FRF point with minimum real part
% [~,in] = min(abs(real(FRF0)));
% modalFrequency = freq(in);
%% Estimate natural frequency from FRF point closed to phase resonance

% Phase angle of centralized FRF
th = unwrap(angle(FRF0));

% Select point closes to phase resonance (-pi/2)
[~,in] = min(abs(th+pi/2));
modalFrequency = freq(in);
%% Estimate modal damping ratio from combinations of frequencies above
% and below resonance
Ia = in+1:length(freq);
Ib = 1:in-1;
combs = combvec(Ia,Ib);
modalDampingRatio = zeros(size(combs,2),1);
for iComb=1:size(combs,2)
    ia = combs(1,iComb);
    ib = combs(2,iComb);

    % Evaluate angular frequencies resonance, above and below
    Omn = 2*pi*modalFrequency;
    Oma = 2*pi*freq(ia);
    Omb = 2*pi*freq(ib);

    % Evaluate phase difference to resonance, above and below
    % %             WHY THE FUCK DOES THIS NOT WORK?!
    %             phia = angle(FRF0(iRes)) - angle(FRF0(iLeft));
    %             phib = angle(FRF0(iRes)) - angle(FRF0(iRight));
    phia = atan(abs(real(FRF0(ia)))/abs(imag(FRF0(ia))));
    phib = atan(abs(real(FRF0(ib)))/abs(imag(FRF0(ib))));

    % Estimate modal damping ratio ...
    %       ... as structural/hysteretic damping
    modalDampingRatio(iComb) = ...
        (Omb^2-Oma^2)/(2*Omn^2)*(1/(tan(phia/2)+tan(phib/2)));
%     %       ... as viscous damping
%     modalDampingRatio(iComb) = ...
%         (Omb^2-Oma^2)/(2*Omn)*(1/(Oma*tan(phia/2)+Omb*tan(phib/2)));
end

% Evaluate mean and standard deviation
modalDampingRatio_mean = mean(modalDampingRatio);
modalDampingRatio_std = std(modalDampingRatio);

% Illustrate FRF and fitted circle
if illustrateCircleFit
    figure;
    hold on; grid on;
    th = linspace(0,2*pi,50)';
    xe = R*cos(th)+xc;
    ye = R*sin(th)+yc;
    plot(FRF,'kx-');
    plot(xe,ye,'g-');
    plot(FRF(in),'ro');
    xlabel('Im(FRF)');
    ylabel('Re(FRF)');
    legend('FRF','fitted circle','identified resonance',...
        'LOCATION','BEST');
    axis equal;
end