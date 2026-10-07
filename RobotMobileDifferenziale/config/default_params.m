function p = default_params()
%DEFAULT_PARAMS  Restituisce la struttura con TUTTI i parametri del progetto.
%
%   p = default_params()
%
%   Nessun valore numerico e' cablato nel codice: ogni grandezza usata da
%   modello, pianificatori, controllore, simulazione e grafica e' definita
%   qui. Per provare valori diversi NON modificare questo file: sovrascrivi
%   i campi in main.m (sezione "PARAMETRI UTENTE") oppure da Command Window:
%
%       p = default_params();
%       p.planner = 'rrt';
%       p.ctrl.k_rho = 1.2;
%       results = run_project(p);
%
%   Unita' di misura: metri, secondi, radianti.

%% ---------------------------------------------------------------- ROBOT
p.robot.r         = 0.05;   % raggio delle ruote r [m]
p.robot.d         = 0.30;   % distanza tra le ruote (carreggiata) d [m]
p.robot.radius    = 0.20;   % raggio del cerchio d'ingombro del robot [m]
p.robot.wheel_max = 20;     % |omega_R|,|omega_L| massime [rad/s]
p.robot.v_max     = 0.8;    % velocita' lineare massima [m/s]
p.robot.w_max     = 2.5;    % velocita' angolare massima [rad/s]

%% ------------------------------------------------------------- AMBIENTE
p.env.scenario         = 'base';  % 'base' | 'narrow' | 'trap' | 'custom'
p.env.bounds           = [];      % [xmin xmax ymin ymax]; [] = quelli dello scenario
p.env.custom_obstacles = {};      % ostacoli se scenario = 'custom' (vedi scenario_library)
p.env.safety_margin    = 0.05;    % margine aggiunto al raggio del robot [m]

%% ------------------------------------------------ CONFIGURAZIONI q0 / qf
p.q0 = [];   % [x0 y0 theta0]; [] = configurazione iniziale dello scenario
p.qf = [];   % [xf yf thetaf]; [] = configurazione finale dello scenario

%% -------------------------------------------------------- PIANIFICAZIONE
p.planner = 'prm';          % 'cell' | 'prm' | 'rrt' | 'apf'
p.seed    = 1;              % seme casuale (riproducibilita' di PRM/RRT/APF)

% 4.1 Cell decomposition con retraction
p.cell.resolution       = 0.20;      % lato della cella [m]
p.cell.connectivity     = 8;         % 4 oppure 8 vicini
p.cell.conservative     = true;      % true: cella libera solo se lo e' tutta
p.cell.clearance_weight = 0.30;      % peso [m] che allontana il cammino dagli ostacoli (0 = piu' corto)
p.cell.search           = 'dijkstra';% 'dijkstra' | 'astar'

% 4.2a PRM
p.prm.n_samples         = 400;       % campioni per tentativo
p.prm.k_neighbors       = 10;        % vicini massimi tentati per nodo
p.prm.connection_radius = 2.5;       % raggio massimo di connessione [m]
p.prm.max_attempts      = 4;         % tentativi se il grafo non connette q0-qf
p.prm.growth            = 1.5;       % fattore di crescita dei campioni per tentativo
p.prm.search            = 'dijkstra';% 'dijkstra' | 'astar'

% 4.2b RRT
p.rrt.max_iter  = 6000;     % iterazioni massime
p.rrt.step      = 0.30;     % passo di espansione [m]
p.rrt.goal_bias = 0.10;     % probabilita' di campionare il goal
p.rrt.goal_tol  = 0.30;     % distanza per tentare la connessione al goal [m]

% 4.3 Artificial Potential Fields
p.apf.k_att        = 1.0;   % guadagno attrattivo
p.apf.d_star       = 2.0;   % soglia quadratico/conico del potenziale attrattivo [m]
p.apf.k_rep        = 1.0;   % guadagno repulsivo
p.apf.rho0         = 0.8;   % raggio d'influenza degli ostacoli [m]
p.apf.goal_rep_n      = 2;  % correzione GNRON: esponente n (0 = Khatib classico)
p.apf.goal_rep_radius = 1.0;% entro questa distanza dal goal la repulsione e' scalata (d/R)^n [m]
p.apf.step         = 0.05;  % passo della discesa del gradiente [m]
p.apf.max_iter     = 4000;  % passi massimi
p.apf.goal_tol     = 0.08;  % tolleranza di arrivo [m]
p.apf.stall_window = 40;    % finestra (passi) per rilevare lo stallo
p.apf.stall_dist   = 0.10;  % spostamento minimo nella finestra [m]
p.apf.escape       = true;  % true: fuga dai minimi locali con random walk
p.apf.escape_steps = 40;    % passi del primo random walk (raddoppiano a ogni fuga)
p.apf.escape_growth_max = 16; % fattore massimo di crescita del random walk
p.apf.escape_step  = 0.10;  % lunghezza di un passo di random walk [m]
p.apf.max_escapes  = 25;    % tentativi massimi di fuga
p.apf.k_virtual    = 0.5;   % guadagno degli ostacoli virtuali posti sui minimi (0 = disattivi)
p.apf.rho_virtual  = 1.0;   % raggio d'influenza degli ostacoli virtuali [m]

% Post-elaborazione comune del percorso
p.post.shortcut         = true;   % semplificazione "string pulling" collision-free
p.post.waypoint_spacing = 0.25;   % distanza tra waypoint dopo il ricampionamento [m]
p.post.collision_step   = 0.02;   % passo di campionamento nel test dei segmenti [m]

%% -------------------------------------------------------------- CONTROLLO
p.ctrl.k_rho               = 1.0;     % v = k_rho * rho
p.ctrl.k_alpha             = 3.0;     % omega = k_alpha * alpha
p.ctrl.ref_mode            = 'waypoints'; % 'waypoints' | 'continuous'
p.ctrl.switch_radius       = 0.25;    % [waypoints] raggio di commutazione [m]
p.ctrl.v_ref               = 0.40;    % [continuous] velocita' del riferimento [m/s]
p.ctrl.max_lag             = 0.50;    % [continuous] ritardo max prima di fermare il riferimento [m]
p.ctrl.cos_alpha_scaling   = true;    % v moltiplicata per max(cos(alpha),0)
p.ctrl.goal_tol            = 0.05;    % tolleranza di posizione finale [m]
p.ctrl.align_final_heading = true;    % ruota sul posto fino a theta_f
p.ctrl.k_theta             = 2.0;     % guadagno dell'allineamento finale
p.ctrl.heading_tol         = 2*pi/180;% tolleranza di orientamento finale [rad]

%% ------------------------------------------------------------ SIMULAZIONE
p.sim.dt              = 0.02;     % passo di integrazione [s]
p.sim.t_max           = 150;      % durata massima [s]
p.sim.integrator      = 'rk4';    % 'euler' | 'rk4' | 'exact'
p.sim.wheel_noise_std = 0.0;      % rumore gaussiano sulle ruote [rad/s] (0 = ideale)

%% ------------------------------------------------------- VISUALIZZAZIONE
p.viz.show_plots         = true;  % grafici dei risultati
p.viz.show_planner_debug = true;  % grafico della struttura del pianificatore
p.viz.animate            = true;  % animazione del moto
p.viz.frame_skip         = 5;     % disegna 1 frame ogni N passi
p.viz.pause              = 0.001; % pausa tra frame [s]
p.viz.save_gif           = false; % salva l'animazione come GIF
p.viz.save_figures       = false; % salva i grafici come PNG
p.viz.output_dir         = 'output';
p.viz.gif_file           = 'robot_motion.gif';

p.verbose = true;                 % stampa a console
end
