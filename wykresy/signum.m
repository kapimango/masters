clear; clc; close all;

s = -5:0.001:5;

rho = 2;      % mnożnik
eps = 0.3;    % parametr wygładzenia

u_sign = -rho * sign(s);
u_tanh = -rho * tanh(s / eps);

figure;
plot(s, u_sign, 'LineWidth', 2); hold on;
plot(s, u_tanh, '--', 'LineWidth', 2);
grid on;

xlabel('s');
ylabel('u(s)');
title('Funkcja signum i jej wygładzone przybliżenie');

legend('-\rho sign(s)', '-\rho tanh(s/\epsilon)', 'Location', 'best');

xline(0, '--');
yline(0, '--');
ylim([-rho-1, rho+1]);

saveas(gcf,"signum",'png');