function print_metrics(m)
%PRINT_METRICS  Stampa a console le metriche di una simulazione.
fprintf('\n---------------- RISULTATI (%s) ----------------\n', upper(m.planner));
fprintf('Tempo di pianificazione       : %8.3f s\n', m.plan_time);
fprintf('Lunghezza percorso pianificato: %8.3f m\n', m.planned_length);
fprintf('Lunghezza percorso eseguito   : %8.3f m\n', m.executed_length);
fprintf('Goal raggiunto                : %8s\n', tf2str(m.reached));
fprintf('Tempo di esecuzione           : %8.2f s\n', m.time_to_goal);
fprintf('Errore finale di posizione    : %8.4f m\n', m.final_pos_error);
fprintf('Errore finale di orientamento : %8.3f deg\n', m.final_head_error * 180/pi);
fprintf('Errore di tracking RMS / max  : %8.4f / %.4f m\n', m.ct_rms, m.ct_max);
fprintf('Distanza min. dagli ostacoli  : %8.3f m (pianificato) / %.3f m (eseguito)\n', ...
        m.min_clear_plan, m.min_clear_exec);
fprintf('Collisioni                    : %8s\n', tf2str(m.collision));
fprintf('|omega ruota| massima         : %8.2f rad/s (saturazione %.1f%% del tempo)\n', ...
        m.max_wheel, 100 * m.saturated_frac);
fprintf('----------------------------------------------------\n');
end
function s = tf2str(b)
if b, s = 'SI'; else, s = 'NO'; end
end
