function p = params_4R()
% PARAMS_4R  Parametri configurabili del progetto "Robot manipolatore planare 4R".
%
%   p = params_4R() restituisce una struttura con TUTTI i valori numerici
%   usati dal progetto. Per provare configurazioni diverse e' sufficiente
%   modificare i valori qui sotto, oppure sovrascrivere i campi della
%   struttura nello script principale (main_4R.m) prima di lanciarlo, es.:
%
%       p = params_4R();
%       p.robot.L = [0.5 0.4 0.3 0.15];
%       p.traj.type = 'line';
%
%   Unita' di misura: SI (m, kg, kg*m^2, s, rad, N*m).

%% ===================== PARAMETRI FISICI DEL ROBOT =======================
p.robot.L  = [0.40 0.35 0.25 0.15];   % lunghezze dei link L1..L4            [m]
p.robot.m  = [3.0  2.5  1.5  0.8 ];   % masse dei link m1..m4                [kg]
% distanza del baricentro dal giunto a monte (default: meta' link)        [m]
p.robot.lc = p.robot.L / 2;
% momenti di inerzia baricentrici (default: asta sottile m*L^2/12)        [kg m^2]
p.robot.I  = p.robot.m .* p.robot.L.^2 / 12;
p.robot.g  = 9.81;                    % accelerazione di gravita' (verso -y) [m/s^2]

%% ===================== CINEMATICA INVERSA ===============================
p.ik.elbow = 'up';      % configurazione del gomito: 'up' (alto) | 'down' (basso)
% Il 4R planare e' ridondante rispetto al compito (x, y, phi): serve un
% parametro aggiuntivo. Si usa psi = theta1+theta2+theta3 (orientamento
% assoluto del link 3). Durante la traiettoria: psi(t) = phi(t) + psiOffset.
p.ik.psiOffset = deg2rad(-40);        % [rad] (0 => theta4 = 0)

%% ===================== TRAIETTORIA DI RIFERIMENTO =======================
p.traj.type     = 'circle';          % 'circle' | 'line'
p.traj.T        = 6;                  % durata del moto                       [s]
p.traj.tHold    = 1;                  % tempo di permanenza finale (fermo)    [s]
% --- cerchio ---
p.traj.center   = [0.45 0.35];        % centro [xc yc]                        [m]
p.traj.radius   = 0.15;               % raggio                                [m]
p.traj.theta0   = 0;                  % angolo polare iniziale sul cerchio    [rad]
p.traj.nTurns   = 1;                  % numero di giri
% --- retta ---
p.traj.pStart   = [0.60 -0.10];       % punto iniziale [x y]                  [m]
p.traj.pEnd     = [0.25  0.55];       % punto finale   [x y]                  [m]
% --- orientamento dell'end-effector (interpolato con la stessa legge oraria)
p.traj.phiStart = deg2rad(0);         % [rad]
p.traj.phiEnd   = deg2rad(30);        % [rad]

%% ===================== CONTROLLORE ======================================
p.ctrl.type = 'computed_torque';      % 'computed_torque' | 'pd_gravity'
% --- guadagni della coppia calcolata (agiscono sulle ACCELERAZIONI, 1/s^2 e 1/s)
%     dinamica dell'errore: dde + Kd de + Kp e (+ Ki int(e)) = 0
wn   = 10;                            % pulsazione naturale desiderata        [rad/s]
zeta = 1;                             % smorzamento (1 = critico)
p.ctrl.Kp = diag(wn^2 * ones(1,4));   % [1/s^2]
p.ctrl.Kd = diag(2*zeta*wn * ones(1,4)); % [1/s]
p.ctrl.Ki = zeros(4);                 % [1/s^3] azione integrale (0 = disattivata)
%     es. poli tripli in -wn: Kp = 3wn^2, Kd = 3wn, Ki = wn^3
% --- guadagni del PD + compensazione di gravita' (agiscono sulle COPPIE)
%     tarati giunto per giunto in base all'inerzia vista da ciascun giunto
p.ctrl.pd.Kp = diag([300 200 60 10]); % [N m/rad]
p.ctrl.pd.Kd = diag([ 40  25  6 0.5]);% [N m s/rad]
p.ctrl.pd.Ki = zeros(4);              % [N m/(rad s)] azione integrale
% Fattore di stima del modello usato dal controllore (1 = modello perfetto).
% Es. 1.2 => il controllore crede che masse/inerzie siano +20% (test robustezza).
p.ctrl.modelScale = 1.0;
p.ctrl.tauMax     = Inf;              % saturazione coppie (scalare o 1x4)   [N m]

%% ===================== SIMULAZIONE ======================================
% Errore iniziale aggiunto alla configurazione desiderata q_d(0)           [rad]
p.sim.q0Offset  = deg2rad([5 -5 5 -5]);
p.sim.dq0       = [0 0 0 0];          % velocita' iniziali                    [rad/s]
p.sim.dtOut     = 0.01;               % passo di campionamento dei risultati  [s]
p.sim.RelTol    = 1e-6;               % tolleranze ode45
p.sim.AbsTol    = 1e-8;

%% ===================== VISUALIZZAZIONE ==================================
p.viz.frameScale = 0.10;              % lunghezza assi delle terne DH        [m]
p.viz.animStep   = 3;                 % disegna 1 campione ogni animStep
p.viz.pause      = 0;                 % pausa aggiuntiva tra i frame          [s]
p.viz.saveVideo  = false;             % true => salva animazione (VideoWriter)
p.viz.videoFile  = 'robot4R_animazione.mp4';
p.viz.qDemo      = deg2rad([30 45 -30 20]); % configurazione per il grafico FK

%% ===================== FLAG DI ESECUZIONE (main_4R) =====================
p.run.fkDemo     = true;   % grafico del robot in una configurazione
p.run.ikDemo     = true;   % verifica FK(IK(p)) e soluzioni gomito alto/basso
p.run.simulate   = true;   % simulazione ode45 con controllore
p.run.plots      = true;   % grafici q(t), e(t), tau(t)
p.run.animate    = true;   % animazione con drawnow
end
