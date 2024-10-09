%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function sets up a linear modal model for interpreting the
% measurement results from [1]. The model involves three Euler-Bernoulli 
% beams, the two cantilevers, plus the mini-beam which forms the unilateral
% spring. The added mass due to the capt nut + bushing assembly, as well
% as the tuning plate are considered. Details of the model are described 
% in Section 3.1 and on page 137, paragraph 3.
% 
% All properties are in SI units.
% 
% REFERENCES
% [1] https://doi.org/10.25518/2684-6500.180
% [2] https://doi.org/10.18419/darus-4501
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of NLtest available via
% https://github.com/maltekrack/NLtest.
% 
% COPYRIGHT (C) 2024
%   Malte Krack (malte.krack@ila.uni-stuttgart.de) 
%   Maren Scheel (maren.scheel@ila.uni-stuttgart.de)
%   Lukas Woiwode
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [MODSPEC,om_lin,PhiMb_lin,Phibar_lin] = ...
    setUpLinearModalBeamModel(MPVset)
%% SPECIFY PROPERTIES

% PROPERTIES OF LEFT/RIGHT BEAM
% main section
E = 210e9;              % Young's modulus
rho = 7720;             % density
len = 392.5*1e-3;       % length (15 mm radius at clamping neglected)
height = 6e-3;          % height in the bending direction
thickness = 60e-3;      % thickness in the third dimension
z = [0 len];            % z range of section
% tuning plate section
mass_tun = (10.8+31.9)*1e-3; % contains screws and washers mass 31.9 gram
len_tun = 40e-3;
height_tun = 1e-3;
thickness_tun = len_tun;
z_tun = z(end)-(11.5-6+57)*1e-3 + [-len_tun 0];
% cap nut + bushing assembly
mass_tip = 13.3e-3; % cap nut, 2 bushings, 2 washers, screw
z_tip = len-11.5e-3;
% mini-beam clamping section
%   NOTE: thickness, len and location are as in tuning plate section
%   masses:
%       mini beam 30.7 g * 40/(40+57); screws&washers&nuts 60.6 g; 
%       left plate 54.3 g; right plate 81.9 g
mass_mbcl = (30.7*40/(40+57)+60.6+54.3+81.9)*1e-3;
height_mbcl = (7.5+5+1)*1e-3;

% PROPERTIES OF MINI-BEAM (free length)
len_MB = 57e-3;
height_MB = 1e-3;
thickness_MB = 40e-3;
mass_MB = (30.7*57/(40+57))*1e-3;
rho_MB = mass_MB/(len_MB*height_MB*thickness_MB);
z_MB = len - (11.5-6)*1e-3 + [-len_MB 0];

% MEASUREMENT LOCATIONS (z coordinate, see PDF indicating the measurement 
% points in [2])
zSens_LB = len - [12 12 62 108 161 206]*1e-3; % 1-6
zSens_MB = len - (11.5-6)*1e-3 - [4 4 21 36]*1e-3; % 7-10
zSens_RB = len - [12 12 62 111 162 210]*1e-3; % 11-16

% SET UP FE MODEL OF LEFT BEAM
% Mesh length ensuring that important z-positions are retained as node
% locations
n_nodes_target = 234; % target number of nodes
zkey = unique(sort([z z_tun z_tip zSens_LB])); % key nodes
zn = cell(size(zkey));
zn{1} = zkey(1);
for ikey=2:length(zkey)
    ztmp = linspace(zkey(ikey-1),zkey(ikey),...
        max(round((zkey(ikey)-zkey(ikey-1))/len*n_nodes_target),2));
%         round((zkey(ikey)-zkey(ikey-1))/len*n_nodes_target));
    zn{ikey} = ztmp(2:end);
end
zn_LB = horzcat(zn{:});
% Define element-specific properties
heighte = height*ones(length(zn_LB)-1,1);
rhoe = rho*ones(size(heighte));
% Account for tuning mass section
isinTunSec = zn_LB(1:end-1)>=z_tun(1) & zn_LB(2:end)<=z_tun(2);
heighteff = height + height_tun*thickness_tun/thickness;
heighte(isinTunSec) = heighteff;
rhoe(isinTunSec) = (rho*len_tun*thickness*height+mass_tun)/...
    (len_tun*thickness*heighteff);
EIe = E*heighte.^3*thickness/12;
% Slightly reduce bending stiffness to account for bolted connection
% (factor .75 set by trial and error to match linear modal frequencies)
EIe(isinTunSec) = EIe(isinTunSec)*.75;
rhoAe = rhoe.*heighte*thickness;
[K_LB,M_LB] = finiteElementBeam(zn_LB,EIe,rhoAe,'clamped-free');
% Add tip mass
inode_tip = find(zn_LB==z_tip);
itrans_tip = 2*inode_tip-3;
M_LB(itrans_tip,itrans_tip) = M_LB(itrans_tip,itrans_tip) + mass_tip;

% SET UP FE MODEL OF RIGHT BEAM
% Mesh length ensuring that important z-positions are retained as node
% locations
zkey = unique(sort([z z_tun zSens_RB])); % key nodes
zn = cell(size(zkey));
zn{1} = zkey(1);
for ikey=2:length(zkey)
    ztmp = linspace(zkey(ikey-1),zkey(ikey),...
        max(round((zkey(ikey)-zkey(ikey-1))/len*n_nodes_target),2));
%         round((zkey(ikey)-zkey(ikey-1))/len*n_nodes_target));
    zn{ikey} = ztmp(2:end);
end
zn_RB = horzcat(zn{:});
% Define element-specific properties
heighte = height*ones(length(zn_RB)-1,1);
rhoe = rho*ones(size(heighte));
% Account for mini-beam clamping section (slightly increased bending 
% stiffness, factor 1.08, set by trial and error to match linear modal 
% frequencies)
isinMBCLSec = zn_RB(1:end-1)>=z_tun(1) & zn_RB(2:end)<=z_tun(2);
heighteff = height + height_mbcl*thickness_tun/thickness;
heighte(isinMBCLSec) = heighteff;
rhoe(isinMBCLSec) = (rho*len_tun*thickness*height+mass_mbcl)/...
    (len_tun*thickness*heighteff);
EIe = 1.08*E*heighte.^3*thickness/12;
% Slightly reduce bending stiffness to account for bolted connection 
% (reduced bending stiffness, factor .1, set by trial and error to match 
% linear modal frequencies)
EIe(isinMBCLSec) = EIe(isinMBCLSec)*.1;
rhoAe = rhoe.*heighte*thickness;
[K_RB,M_RB] = finiteElementBeam(zn_RB,EIe,rhoAe,'clamped-free');

% SET UP FE MODEL OF MINI-BEAM
% Mesh length ensuring that important z-positions are retained as node
% locations
zkey = unique(sort([z_MB zSens_MB])); % key nodes
zn = cell(size(zkey));
zn{1} = zkey(1);
for ikey=2:length(zkey)
    ztmp = linspace(zkey(ikey-1),zkey(ikey),...
        max(round((zkey(ikey)-zkey(ikey-1))/len*n_nodes_target),2));
%         round((zkey(ikey)-zkey(ikey-1))/len*n_nodes_target));
    zn{ikey} = ztmp(2:end);
end
zn_MB = horzcat(zn{:});
% Define element-specific properties (slightly increased bending 
% stiffness, factor 1.07, set by trial and error to match linear modal 
% frequencies)
heighte = height_MB*ones(length(zn_MB)-1,1);
rhoe = rho_MB*ones(size(heighte));
EIe = 1.07*E*heighte.^3*thickness/12;
rhoAe = rhoe.*heighte*thickness;
[K_MB,M_MB] = finiteElementBeam(zn_MB,EIe,rhoAe,'free-free');

% Assemble RB+MB via localization matrix 'L' and primal assembly (null
% space formalism)
loc_connection_MB_on_RB = len-(11.5-6+57)*1e-3;
M_MBRB = blkdiag(M_MB,M_RB);
K_MBRB = blkdiag(K_MB,K_RB);
L = eye(length(M_MBRB));
n_MB = length(M_MB);
% n_RB = length(M_RB);
inode_con = find(zn_RB==loc_connection_MB_on_RB);
idof_con = 2*inode_con-3+(0:1);
L(n_MB+idof_con(1),1) = 1;
L(n_MB+idof_con(2),2) = 1;
L(:,n_MB+idof_con) = [];
M_MBRB = L'*M_MBRB*L;
K_MBRB = L'*K_MBRB*L;

% Determine frequencies and mass-normalized deflection shapes of
% 4 lowest-frequency modes of LB
Nmod = 4;
[PHI_LB,OM2] = eigs(K_LB,M_LB,Nmod,'smallestabs');
[om_LB,ind] = sort(sqrt(diag(OM2)));
PHI_LB = PHI_LB(:,ind);
PHI_LB = PHI_LB./repmat(sqrt(diag(PHI_LB'*M_LB*PHI_LB))',...
    size(PHI_LB,1),1);

% Determine frequencies and mass-normalized deflection shapes of
% 6 lowest-frequency modes of MBRB
Nmod = 6;
[PHI_MBRB,OM2] = eigs(K_MBRB,M_MBRB,Nmod,'smallestabs');
[om_MBRB,ind] = sort(sqrt(diag(OM2)));
PHI_MBRB = PHI_MBRB(:,ind);
PHI_MBRB = PHI_MBRB./repmat(sqrt(diag(PHI_MBRB'*M_MBRB*PHI_MBRB))',...
    size(PHI_MBRB,1),1);
% Re-sort to have 4 RB and 2 MB modes
resort = [1 2 4 5 3 6];
om_MBRB = om_MBRB(resort); PHI_MBRB = PHI_MBRB(:,resort);

% Collect all modal frequencies
om_lin = [om_LB(:);om_MBRB(:)];
MODSPEC = {'LB,m=1','LB,m=2','LB,m=3','LB,m=4',...
    'RB,m=1','RB,m=2','RB,m=3','RB,m=4',...
    'MB,m=1','MB,m=2'};
%%%%%%%%%%%%%%%%%% CHECK/ILLUSTRATE LINEAR MODAL RESULTS
% % om_LB(1:4)/(2*pi)
% om_LB_ref = [30.95 199.04 556.82 1085.93]'*2*pi;
% omErr = (om_LB(1:length(om_LB_ref))./om_LB_ref-1)
% 
% % om_MBRB(1:5)/(2*pi)
% om_MBRB_ref = [28.2 205.54 277.74 594.98 1168.62]'*2*pi;
% omErr = (om_MBRB(1:length(om_MBRB_ref))./om_MBRB_ref(resort(1:end-1))-1)
% 
% figure; hold on;
% imodplot = 1;
% plot(zn_LB(2:end),PHI_LB(1:2:end,imodplot),'k-x');
% 
% figure; hold on;
% imodplot = 4;
% plot(zn_MB,PHI_MBRB(1:2:n_MB,imodplot),'g-x');
% PHI_ex = L*PHI_MBRB;
% plot(zn_RB(2:end),PHI_ex(n_MB+(1:2:n_RB),imodplot),'k-x');
%%%%%%%%%%%%%%%%%%

% Evaluate modal forcing term for base excitation
PhiMb_lin = [sum(PHI_LB'*M_LB(:,1:2:end),2);...
    sum(PHI_MBRB'*M_MBRB(:,1:2:end),2)];

% Evaluate mode shapes at measurement locations
[isIn,iNodeSensLB] = ismember(zSens_LB,zn_LB);
iDOFSensLB = 2*iNodeSensLB-3;
if ~all(isIn); error('Could not find sensor node.'); end
[isIn,iNodeSensMB] = ismember(zSens_MB,zn_MB);
iDOFSensMB = 2*iNodeSensMB-1;
if ~all(isIn); error('Could not find sensor node.'); end
[isIn,iNodeSensRB] = ismember(zSens_RB,zn_RB);
iDOFSensRB = n_MB + 2*iNodeSensRB-3;
if ~all(isIn); error('Could not find sensor node.'); end
PHI_ex = L*PHI_MBRB;
Phibar_lin = blkdiag(PHI_LB(iDOFSensLB,:),...
    [PHI_ex(iDOFSensMB,:);PHI_ex(iDOFSensRB,:)]);

% Restrict to actually used set of sensors
Phibar_lin = Phibar_lin(MPVset,:);

%% 1D Finite Element Model of an Euler-Bernoulli beam with node locations 
% specified by 'xn', and element-specific bending stiffness EIe and mass
% per length rhoAe. We have two nodes per element, with two degrees of 
% freedom each, one transverse displacement in bending direction, and one 
% rotation. Cubic Hermitian polynomials are used as shape functions.
function [K,M,L] = finiteElementBeam(xn,EIe,rhoAe,BCs)
n_nodes = length(xn);
Ne = n_nodes-1;
LEN_E = diff(xn);

% Element matrices
Nde = 4;
Elements(1:Ne) = struct('m',zeros(Nde,Nde),'k',zeros(Nde,Nde),'le',[]);
for e=1:Ne
    Elements(e).le = [e;e+1];
    len_e = LEN_E(e);
    Elements(e).m = rhoAe(e)*len_e/420*[...
        156 22*len_e 54 -13*len_e;...
        22*len_e 4*len_e^2 13*len_e -3*len_e^2;...
        54 13*len_e 156 -22*len_e;...
        -13*len_e -3*len_e^2 -22*len_e 4*len_e^2];
    Elements(e).k = EIe(e)/len_e^3*[...
        12 6*len_e -12 6*len_e;...
        6*len_e 4*len_e^2 -6*len_e 2*len_e^2;...
        -12 -6*len_e 12 -6*len_e;...
        6*len_e 2*len_e^2 -6*len_e 4*len_e^2];
end

% Assemble system M,K
K = zeros(Nde/2*n_nodes,Nde/2*n_nodes); M = K;
for e=1:Ne
    ILE = Nde/2*(kron(Elements(e).le(:),ones(Nde/2,1))-1) + ...
        repmat((1:Nde/2)',2,1);
    K(ILE,ILE) = K(ILE,ILE) + Elements(e).k;
    M(ILE,ILE) = M(ILE,ILE) + Elements(e).m;
end

% Remove rows and colums in accordance with constraints B*q=0,
% so that we take the null space, L, of B (i.e. B*L=0) as
% reduced set of basis vectors, q = L*q_full where q_full
% contains all DOFs of the free beam.
L = eye(size(K,1));
switch BCs
    case 'clamped-clamped'
        L = L(:,3:end-2);
        K = K(3:end-2,3:end-2);
        M = M(3:end-2,3:end-2);
    case 'clamped-pinned'
        L = L(:,[3:end-2 end]);
        K = K([3:end-2 end],[3:end-2 end]);
        M = M([3:end-2 end],[3:end-2 end]);
    case 'clamped-free'
        L = L(:,3:end);
        K = K(3:end,3:end);
        M = M(3:end,3:end);
    case 'pinned-clamped'
        L = L(:,2:end-2);
        K = K(2:end-2,2:end-2);
        M = M(2:end-2,2:end-2);
    case 'pinned-pinned'
        L = L(:,[2:end-2 end]);
        K = K([2:end-2 end],[2:end-2 end]);
        M = M([2:end-2 end],[2:end-2 end]);
    case 'pinned-free'
        L = L(:,2:end);
        K = K(2:end,2:end);
        M = M(2:end,2:end);
    case 'free-clamped'
        L = L(:,1:end-2);
        K = K(1:end-2,1:end-2);
        M = M(1:end-2,1:end-2);
    case 'free-pinned'
        L = L(:,[1:end-2 end]);
        K = K([1:end-2 end],[1:end-2 end]);
        M = M([1:end-2 end],[1:end-2 end]);
    case 'free-free'
        % Nothing to constrain
    otherwise
        error('Not implemented yet.');
end