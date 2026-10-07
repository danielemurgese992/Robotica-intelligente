# Robot manipolatore planare 4R con gravità (MATLAB)

Modellazione cinematica e dinamica, cinematica inversa, controllo a coppia calcolata,
simulazione con `ode45` e animazione con `drawnow`.

## Avvio rapido
1. Aprire MATLAB (R2019b o successivo; compatibile anche con GNU Octave 8+).
2. Impostare come cartella corrente quella che contiene questo file.
3. Lanciare `main_4R` → grafici FK/IK, valori di M, C, G, simulazione, grafici e animazione.
4. Lanciare `test_4R` → batteria di test numerici (PASS/FAIL).
5. (Opzionale, Symbolic Math Toolbox) `S = derive_dynamics_4R_sym(params_4R(), true)`
   → derivazione simbolica di M, C, G con verifica e generazione dei file in `generated/`.

## Parametri
Tutti i valori numerici sono in `params_4R.m` (robot, IK, traiettoria, controllore,
simulazione, grafica, flag di esecuzione). Si possono modificare lì oppure nella sezione
"PERSONALIZZAZIONE" di `main_4R.m`.

## File
| File | Ruolo |
|---|---|
| `main_4R.m` | script principale |
| `params_4R.m` | parametri configurabili |
| `dh_table_4R.m`, `dh_matrix.m` | tabella DH e matrici A_i |
| `fk_4R.m` | cinematica diretta `[T,x,y,phi] = fk_4R(theta,L)` |
| `ik_4R.m` | cinematica inversa `theta = ik_4R(x,y,phi,L)` |
| `jacobian_4R.m` | Jacobiano e sua derivata |
| `dynamics_4R.m` | modello dinamico `[M,C,G] = dynamics_4R(theta,dtheta,params)` |
| `energy_4R.m` | energia cinetica e potenziale (verifiche) |
| `derive_dynamics_4R_sym.m` | derivazione simbolica Eulero–Lagrange |
| `task_reference_4R.m`, `build_reference_4R.m`, `reference_4R.m` | traiettoria |
| `controller_4R.m` | coppia calcolata / PD + gravità |
| `robot_ode_4R.m`, `simulate_4R.m` | simulazione ode45 |
| `plot_robot_4R.m`, `plot_results_4R.m`, `animate_4R.m` | grafica e animazione |
| `test_4R.m` | test numerici |

La documentazione completa è in `Documentazione_Robot_4R.docx`.
