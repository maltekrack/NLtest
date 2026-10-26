%========================================================================
% DESCRIPTION: 
% Function setting up modal beam model in the context of the self-adaptive
% system.
% A straight and non-preloaded beam, length L, uniform mass density rho, 
% Young’s modulus E and rectangular cross section (area A, area moment of 
% inertia w.r.t. bending I) is considered. The beam is pinned at both ends 
% but the clamping has a finite rotational stiffness kr, and axial
% stiffness kt. The beam is exposed to imposed base motion w0(t).
% 
% Classical beam theory is used (negligible rotational and longitudinal
% inertia, small rotations, Hooke's law, cross sections remain planar and 
% orthogonal to bending curve ~ Bernoulli). To model bending-stretching
% coupling, a second-order Taylor expansion of the axial strain is made.
% 
% Truncation to the Nmod lowest-frequency bending modes is applied, where
% the scalar elastic transversal displacement and the slope read
% 
%           w = PHI(x/L)*qb,
%           dw/dx = dPHI(x/L)*qb,
% 
% where qb is a column vector containing the modal coordinates, and
% PHI(xi), dPHI(xi) evaluate to a row vector of the same dimensions.
% The reduced equations of motion take the form
% 
%           Mb*\dot ub + hb(ddw0,qb,ub) = 0,
% 
% where ub=\dot qb, and ddw0 is the second-order time derivative of w0(t).
% Modal damping with specified damping ratios Dmod is modeled.
%========================================================================
% This file is part of simSAS.
% 
% If you use simSAS, please refer to either one or mulitple of these
% articles:
%   [1] Müller, F.; Beck, M.; Krack, M.: Experimental validation of a model 
%       for a self-adaptive beam–slider system. MSSP, 2023. 
%       doi: 10.1016/j.ymssp.2022.109551
%       MODEL EXTENDED BY IMPERFECT BOUNDARY CONDITIONS &
%       EXPERIMENTAL VALIDATION
%   [2] Müller, F.; Krack, M.: Explanation of the self-adaptive dynamics of 
%       a harmonically forced beam with a sliding mass. AAM, 2020. 
%       doi: 10.1007/s00419-020-01684-5
%       MODEL EXTENDED BY GEOMETRIC NONLINEARITY OF THE BEAM
%   [3] Krack, M.; et al.: Toward understanding the self-adaptive dynamics 
%       of a harmonically forced beam with a sliding mass. AAM, 2017. 
%       doi: 10.1007/s00419-016-1218-5
%       INITIAL MODEL
% 
% COPYRIGHT AND LICENSING: 
% Copyright (C) 2023  Malte Krack (malte.krack@ila.uni-stuttgart.de) and
%                     Florian Müller (florianm987@gmail.com)
% This program comes with ABSOLUTELY NO WARRANTY. 
% simSAS is free software, you can redistribute and/or modify it under the
% GNU General Public License as published by the Free Software Foundation,
% either version 3 of the License, or (at your option) any later version.
% For details on license and warranty, see http://www.gnu.org/licenses
% or gpl-3.0.txt.
%========================================================================
function [Mb,hb,PHI,dPHI,om,b,gam] = modalBeamModel(L,rho,E,A,I,kr,kt,...
    Nmod,Dmod)

% First, we want to obtain the modal deflection shapes and frequencies of
% the underlying linear beam (no bending-stretching coupling, no effect
% of slider). A finite rotational clamping stiffness kr is considered.

% Auxiliary parameters
krL_EI = kr*L/(E*I);
EI_rhoAL4 = (E*I)/(rho*A*L^4);

% Base functions spanning the modal deflection shapes, and derivatives, 
% where phi(xi) = W(lam,xi) * c.
W = @(lam,xi) [sin(lam*xi) cos(lam*xi) sinh(lam*xi) cosh(lam*xi)];
dW = @(lam,xi) [cos(lam*xi) -sin(lam*xi) cosh(lam*xi) sinh(lam*xi)];
ddW = @(lam,xi) [-sin(lam*xi) -cos(lam*xi) sinh(lam*xi) cosh(lam*xi)];

% Boundary condition matrix and its determinant ( B(lam) * c = 0 )
B = @(lam) [W(lam,0); ...                   % left end: pinned
    lam*ddW(lam,0)-krL_EI*dW(lam,0); ...    % left end: rot. spring
    W(lam,1); ...                           % right end: pinned
    lam*ddW(lam,1)+krL_EI*dW(lam,1)];       % right end: rot. spring

% Estimate eigenvalues (asympt. value for high mode orders, ideal clamping)
lamest = (2*(1:Nmod)+1)*pi/2;

% Determine lowest-frequency modes
lam = zeros(Nmod,1);    % roots of characteristic equation
C = zeros(4,Nmod);      % coefficients of modal deflection shape
om = zeros(Nmod,1);     % modal frequencies
gam = zeros(Nmod,1);    % modal excitation factor (imposed base acc.)
dphi = cell(Nmod,1);    % derivative of mode shape as cell array
for imod=1:Nmod

    % Determine roots of characteristic equation and modal deflection shape
    lam(imod) = fzero(@(lam) det(B(lam)),lamest(imod));
    c = null(B(lam(imod)));
    % Normalize phase of mode shape to sine coefficient
    c = c/c(1);
    % Normalize amplitude of mode shape to have unit mass
    mst = integral(@(xi) (c'*W(lam(imod),xi')').^2,0,1)*rho*A*L;
    c = c/sqrt(mst);
    % Store result
    C(:,imod) = c;
    dphi{imod} = @(xi) lam(imod)*(c'*dW(lam(imod),xi')');

    % Modal excitation factor
    gam(imod) = integral(@(xi) (c'*W(lam(imod),xi')'),0,1)*rho*A*L;

    % Modal frequency
    om(imod) = sqrt(EI_rhoAL4*lam(imod).^4);
end

% Define modal deflection shape as continuous function of xi
phistr = '@(xi,W,lam,C) [';
dphistr = '@(xi,dW,lam,C) [';
for imod=1:Nmod
    phistr = [phistr ' W(xi,lam(' num2str(imod) ...
        '))*C(:,' num2str(imod) ') '];
    dphistr = [dphistr ' lam(' num2str(imod) ...
        ')/L*dW(xi,lam(' num2str(imod) '))*C(:,' num2str(imod) ') '];
end
phistr = [phistr ']']; 
dphistr = [dphistr ']']; 
PHItmp = eval(phistr);
dPHItmp = eval(dphistr);
PHI = @(xi) PHItmp(xi(:),W,lam,C);
dPHI = @(xi) dPHItmp(xi(:),dW,lam,C);

% Determine coefficients of geometric nonlinearity
int_dphidphi = zeros(Nmod,Nmod);
for i=1:Nmod
    for j=i:Nmod
        int_dphidphi(i,j) = quadgk(...
            @(xi) dphi{i}(xi).*dphi{j}(xi), 0, 1,...
            'AbsTol',1e-6);
    end
end
int_dphidphi = triu(int_dphidphi) + triu(int_dphidphi)' ...
    - diag(diag(int_dphidphi));
b = zeros(Nmod,Nmod,Nmod,Nmod);
for n=1:Nmod
    for i=1:Nmod
        for j=1:Nmod
            for k=1:Nmod
                % The sorting summarizes redundant polynomial terms,
                % reducing the number of terms to be evaluated. This has to
                % be in accordance with the definition of the nonlinear
                % restoring force!
                srtd_idx = sort([i j k]);
                b(srtd_idx(1),srtd_idx(2),srtd_idx(3),n) = ...
                    b(srtd_idx(1),srtd_idx(2),srtd_idx(3),n) + ...
                    int_dphidphi(i,j)*int_dphidphi(k,n);
            end
        end
    end
end
kax = 1/( 1/(E*A/L) + 1/kt );     % effective axial stiffness
b = kax/(2*L^2)*b;

% Define polynomial N(qb) evaluating the geometric nonlinearity. qb is the
% column vector of the beam's modal coordinates. N(qb) returns a column 
% vector of the modal forces.
N_str = '';
for rr = 1:Nmod
    for jj = 1:Nmod
        for kk = jj:Nmod
            for ll = kk:Nmod
                N_str = [N_str '+(' num2str(b(jj,kk,ll,rr)) ')' ...
                    '*qb(' num2str(jj) ')' ...
                    '*qb(' num2str(kk) ')' ...
                    '*qb(' num2str(ll) ')' ];
            end
        end
    end
    N_save{rr}=str2func(['@(qb)' N_str]);
    N_str = [];
end
% from cell array to one function handle, where output is column-vector
N_save = strcat(regexprep(cellfun(@func2str, N_save, 'uni', 0), ...
    '^@\(qb\)', ''), ';');
N = str2func(strcat('@(qb) [', N_save{:}, ']'));

% Set up parts of equation system (for form, see above)
Mb = eye(Nmod);
hb = @(qb,ub,ddw0) -gam*ddw0 - diag(om.^2)*qb - 2*Dmod.*diag(om)*ub  ...
    - N(qb);