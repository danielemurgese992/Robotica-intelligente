function c = clearance(P, env)
%CLEARANCE  Distanza con segno dall'ostacolo (o bordo) piu' vicino.
c = min(obstacle_distances(P, env), [], 2);
end
