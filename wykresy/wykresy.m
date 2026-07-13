clc; clear; close all hidden;

%% Wczytanie danych
rodzaj = "pid0";
dane = readtable(sprintf("%s.csv", rodzaj));

t = dane.t;
x = dane.x;
y = dane.y;

T_left  = dane.T_left;
T_right = dane.T_right;

base_cmd = dane.base_cmd;
diff_cmd = dane.diff_cmd;

J_total = dane.J_total;

I_roll = dane.I_roll;
I_pitch = dane.I_pitch;
I_roll_rate = dane.I_roll_rate;
I_pitch_rate = dane.I_pitch_rate;
I_control = dane.I_control;
I_no_progress = dane.I_no_progress;
I_goal_heading = dane.I_goal_heading;

roll_deg = dane.roll_deg;

%% 1. Wykres trajektorii z waypointami i okręgami zaliczenia

WP = [
    -482.0, 190.0;   % WP1
    -482.0, 212.0;   % WP2
    -532.0, 190.0    % WP3
];

START = [-532.0, 190.0];
WAYPOINT_RADIUS = 4.0;   % [m]

figure;
hold on;
grid on;
axis equal;

hTraj = plot(x, y, 'LineWidth', 1.8);

hStart = scatter(x(1), y(1), 80, 'filled');
hEnd   = scatter(x(end), y(end), 80, 'filled');

hWP = scatter(WP(:,1), WP(:,2), 90, 'filled');

theta = linspace(0, 2*pi, 300);

for i = 1:size(WP,1)
    xc = WP(i,1) + WAYPOINT_RADIUS * cos(theta);
    yc = WP(i,2) + WAYPOINT_RADIUS * sin(theta);

    if i == 1
        hCircle = plot(xc, yc, '--', 'LineWidth', 1.4);
    else
        plot(xc, yc, '--', 'LineWidth', 1.4);
    end

    text(WP(i,1) + 0.8, WP(i,2) + 0.8, sprintf('WP %d', i), ...
        'FontSize', 11, ...
        'FontWeight', 'bold');
end

scatter(START(1), START(2), 110, 'p', 'filled');
text(START(1) + 0.8, START(2) - 1.8, 'START', ...
    'FontSize', 10, ...
    'FontWeight', 'bold');

xlabel('x [m]');
ylabel('y [m]');
title('Trajektoria ruchu katamaranu z waypointami i promieniami zaliczenia');

legend([hTraj, hStart, hEnd, hWP, hCircle], ...
    {'Trajektoria', 'Start pomiaru', 'Koniec pomiaru', ...
     'Waypointy nominalne', 'Promień zaliczenia WP'}, ...
    'Location', 'best');

saveas(gcf, sprintf("waypoint_%s", rodzaj), 'png');

%% 2. Wykres sygnału sterującego

figure;
plot(t, base_cmd, 'LineWidth', 1.5);
hold on;
plot(t, diff_cmd, 'LineWidth', 1.5);
grid on;

xlabel('Czas [s]');
ylabel('Sygnał sterujący');
title('Sygnały sterujące regulatora');
legend('base\_cmd', 'diff\_cmd', 'Location', 'best');
axis tight;

saveas(gcf, sprintf("sterowanie_%s", rodzaj), 'png');

%% 3. Wykres ciągu silników - lewy i prawy razem

figure;
plot(t, T_left, 'LineWidth', 1.5);
hold on;
plot(t, T_right, 'LineWidth', 1.5);
grid on;

xlabel('Czas [s]');
ylabel('Ciąg silnika');
title('Ciąg lewego i prawego silnika');
legend('T\_left', 'T\_right', 'Location', 'best');
axis tight;

saveas(gcf, sprintf("ciag_%s", rodzaj), 'png');

%% 4. Wykres wskaźnika jakości J_total w czasie

figure;
plot(t, J_total, 'LineWidth', 1.8);
grid on;

xlabel('Czas [s]');
ylabel('J_{total}');
title('Całkowity wskaźnik jakości w czasie');
axis tight;

saveas(gcf, sprintf("wskaznik_total_%s", rodzaj), 'png');

%% 5. Wykres wartości przechyłu roll

figure;
plot(t, roll_deg, 'LineWidth', 1.5);
grid on;

xlabel('Czas [s]');
ylabel('Roll [deg]');
title('Przechył poprzeczny jednostki w czasie');
yline(0, '--');
axis tight;

saveas(gcf, sprintf("roll_%s", rodzaj), 'png');

%% 6. Wartości całkowite wskaźników - tabela

nazwy = {
    'I_roll'
    'I_pitch'
    'I_roll_rate'
    'I_pitch_rate'
    'I_control'
    'I_no_progress'
    'I_goal_heading'
    'J_total_koniec'
    };

wartosci = [
    I_roll(end)
    I_pitch(end)
    I_roll_rate(end)
    I_pitch_rate(end)
    I_control(end)
    I_no_progress(end)
    I_goal_heading(end)
    J_total(end)
    ];

TabelaWskaznikow = table(nazwy, wartosci, ...
    'VariableNames', {'Wskaznik', 'Wartosc'});

disp(' ');
disp('===== TABELA WSKAŹNIKÓW CAŁKOWITYCH =====');
disp(TabelaWskaznikow);

%% 7. Dodatkowe statystyki sygnałów sterujących i ciągu

stat_nazwy = {
    'T_left mean'
    'T_left max'
    'T_left min'
    'T_right mean'
    'T_right max'
    'T_right min'
    'base_cmd mean'
    'base_cmd max'
    'base_cmd min'
    'diff_cmd mean'
    'diff_cmd max'
    'diff_cmd min'
    'roll mean'
    'roll max'
    'roll min'
    };

stat_wartosci = [
    mean(T_left)
    max(T_left)
    min(T_left)
    mean(T_right)
    max(T_right)
    min(T_right)
    mean(base_cmd)
    max(base_cmd)
    min(base_cmd)
    mean(diff_cmd)
    max(diff_cmd)
    min(diff_cmd)
    mean(roll_deg)
    max(roll_deg)
    min(roll_deg)
    ];

TabelaStatystyk = table(stat_nazwy, stat_wartosci, ...
    'VariableNames', {'Wielkosc', 'Wartosc'});

disp(' ');
disp('===== STATYSTYKI STEROWANIA, CIĄGU I ROLL =====');
disp(TabelaStatystyk);