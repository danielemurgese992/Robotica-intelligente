%% MAIN - Pianificazione e controllo di un robot mobile differenziale
%
%  Punto d'ingresso del progetto. Uso:
%    1) aprire MATLAB nella cartella del progetto;
%    2) modificare la sezione "PARAMETRI UTENTE" qui sotto;
%    3) eseguire:  >> main
%
%  L'elenco completo dei parametri, con unita' di misura e significato, e'
%  in config/default_params.m. Ogni campo non sovrascritto qui mantiene il
%  valore di default.

clear; close all; clc;
root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end
addpath(genpath(root));

p = default_params();

%% ===================== PARAMETRI UTENTE =====================
% --- scenario e configurazioni
p.env.scenario = 'base';        % 'base' | 'narrow' | 'trap' | 'custom'
p.q0 = [];                      % es. [0.8 0.8 0]       ([] = default scenario)
p.qf = [];                      % es. [9.2 9.2 pi/2]

% --- robot
p.robot.r         = 0.05;       % raggio ruote [m]
p.robot.d         = 0.30;       % carreggiata [m]
p.robot.radius    = 0.20;       % ingombro [m]
p.robot.wheel_max = 20;         % [rad/s]

% --- pianificatore
p.planner = 'prm';              % 'cell' | 'prm' | 'rrt' | 'apf'
p.seed    = 1;

% --- controllore
p.ctrl.k_rho    = 1.0;
p.ctrl.k_alpha  = 3.0;
p.ctrl.ref_mode = 'waypoints';  % 'waypoints' | 'continuous'

% --- simulazione / grafica
p.sim.dt         = 0.02;
p.sim.integrator = 'rk4';       % 'euler' | 'rk4' | 'exact'
p.viz.animate    = true;
p.viz.save_gif   = false;

% --- esempio di scenario personalizzato (decommentare per usarlo)
% p.env.scenario = 'custom';
% p.env.bounds   = [0 8 0 6];
% p.env.custom_obstacles = { make_obstacle('circle', [3 3], 0.8), ...
%                            make_obstacle('polygon', [5 1; 6 1; 6 4; 5 4]) };
% p.q0 = [0.7 0.7 0];  p.qf = [7.3 5.3 pi/2];
%% =============================================================

results = run_project(p);

%% (opzionale) confronto quantitativo fra i quattro pianificatori
% T = compare_planners(p, {'cell','prm','rrt','apf'}, 5);
