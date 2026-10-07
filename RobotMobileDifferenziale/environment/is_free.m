function tf = is_free(P, env)
%IS_FREE  true se il robot centrato in P non collide (con margine di sicurezza).
tf = clearance(P, env) > env.inflation;
end
