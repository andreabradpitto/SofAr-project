dt = 0.001;
t = 0:dt:1;
u = t;
area_rect = 0;

for k = 1 : length(t)
    area_rect = area_rect + dt * u(k);
end

% Simpson
area_simps = simps(t,u);

simps([0,0.1],[0,0.1])