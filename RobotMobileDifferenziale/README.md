# Pianificazione e controllo di un robot mobile differenziale (MATLAB)

Avvio rapido (MATLAB R2016b o successivo, nessun toolbox richiesto; compatibile anche con GNU Octave >= 6):

    >> cd RobotMobileDifferenziale
    >> main                        % esegue pianificazione + controllo + grafici + animazione

Parametri: tutti in `config/default_params.m`; si sovrascrivono nella sezione
"PARAMETRI UTENTE" di `main.m` oppure da Command Window:

    >> addpath(genpath(pwd));
    >> p = default_params();
    >> p.planner = 'rrt';  p.env.scenario = 'narrow';  p.ctrl.k_rho = 1.5;
    >> out = run_project(p);

Test automatici:          >> addpath(genpath(pwd)); run_all_tests
Confronto pianificatori:  >> T = compare_planners(default_params(), {'cell','prm','rrt','apf'}, 5);

Cartelle: config (parametri, scenari), model (cinematica differenziale, integrazione),
environment (ostacoli, distanze, collisioni), planning (cell decomposition, PRM, RRT, APF),
control (controllore rho-alpha, simulazione), analysis (metriche, confronto),
visualization (grafici, animazione), utils, tests.
