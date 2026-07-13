clear; clc; close all;

%% ========================================================
%  KONFIGURACJA
% =========================================================

KATALOG_DANYCH   = "dane";
KATALOG_WYKRESOW = "wykresy_png";

ZAPISZ_XLSX = true;

WAYPOINT_RADIUS = 4.0;

START = [-532, 190];

WP = [
    -482, 190;
    -482, 212;
    -532, 190
];

% Kolor pionowych linii przejścia waypointów
KOLOR_PRZEJSCIA_WP = [0.00 1.00 0.00];
SZEROKOSC_LINII_WP = 1.8;

%% ========================================================
%  WYSZUKIWANIE PLIKÓW CSV
% =========================================================

pliki = dir(fullfile(KATALOG_DANYCH, "**", "*.csv"));

if isempty(pliki)
    error("Nie znaleziono plików CSV w katalogu: %s", KATALOG_DANYCH);
end

if ~exist(KATALOG_WYKRESOW, "dir")
    mkdir(KATALOG_WYKRESOW);
end

podsumowanie = table();

fprintf("Znaleziono %d plików CSV.\n", numel(pliki));

%% ========================================================
%  PĘTLA PO WSZYSTKICH PLIKACH CSV
% =========================================================

for k = 1:numel(pliki)

    sciezka_csv = fullfile(pliki(k).folder, pliki(k).name);
    [~, nazwa_pliku_char, ~] = fileparts(pliki(k).name);
    nazwa_pliku = string(nazwa_pliku_char);

    fprintf("\n[%d/%d] Przetwarzam: %s\n", k, numel(pliki), sciezka_csv);

    %% ----------------------------------------------------
    %  Wykrywanie regulatora na podstawie ścieżki/nazwy pliku
    % -----------------------------------------------------

    tekst_do_wykrycia = lower(string(fullfile(pliki(k).folder, pliki(k).name)));

    if contains(tekst_do_wykrycia, "pid")
        folder_regulatora = "PID";
        nazwa_na_wykresie = "PID";
    elseif contains(tekst_do_wykrycia, "mpc")
        folder_regulatora = "MPC";
        nazwa_na_wykresie = "MPC";
    elseif contains(tekst_do_wykrycia, "smc")
        folder_regulatora = "SMC";
        nazwa_na_wykresie = "SMC";
    else
        folder_regulatora = "INNE";
        nazwa_na_wykresie = "INNE";
    end

    folder_wyj = fullfile(KATALOG_WYKRESOW, folder_regulatora);

    if ~exist(folder_wyj, "dir")
        mkdir(folder_wyj);
    end

    wariant = wykryjWariant(nazwa_pliku);

    %% ----------------------------------------------------
    %  Wczytanie danych
    % -----------------------------------------------------

    try
        dane = readtable(sciezka_csv);
    catch ME
        warning("Nie udało się wczytać pliku: %s\n%s", sciezka_csv, ME.message);
        continue;
    end

    n = height(dane);

    if n < 2
        warning("Plik ma za mało próbek: %s", sciezka_csv);
        continue;
    end

    t = pobierzKolumne(dane, "t", n);
    x = pobierzKolumne(dane, "x", n);
    y = pobierzKolumne(dane, "y", n);

    wp_idx = pobierzKolumne(dane, "wp_idx", n);

    T_left  = pobierzKolumne(dane, "T_left", n);
    T_right = pobierzKolumne(dane, "T_right", n);

    base_cmd = pobierzKolumne(dane, "base_cmd", n);
    diff_cmd = pobierzKolumne(dane, "diff_cmd", n);

    J_total = pobierzKolumne(dane, "J_total", n);

    I_roll         = pobierzKolumne(dane, "I_roll", n);
    I_pitch        = pobierzKolumne(dane, "I_pitch", n);
    I_roll_rate    = pobierzKolumne(dane, "I_roll_rate", n);
    I_pitch_rate   = pobierzKolumne(dane, "I_pitch_rate", n);
    I_control      = pobierzKolumne(dane, "I_control", n);
    I_no_progress  = pobierzKolumne(dane, "I_no_progress", n);
    I_goal_heading = pobierzKolumne(dane, "I_goal_heading", n);

    roll_deg = pobierzKolumne(dane, "roll_deg", n);

    %% ----------------------------------------------------
    %  Wyznaczenie momentów przejścia między waypointami
    % -----------------------------------------------------

    [czy_jest_wp12, idx_wp12, t_wp12] = znajdzPrzejscieWP(wp_idx, t, 1, 2);
    [czy_jest_wp23, idx_wp23, t_wp23] = znajdzPrzejscieWP(wp_idx, t, 2, 3);

    czasy_przejsc = [];

    if czy_jest_wp12
        czasy_przejsc(end+1) = t_wp12;
    end

    if czy_jest_wp23
        czasy_przejsc(end+1) = t_wp23;
    end

    %% ====================================================
    %  1. WYKRES TRAJEKTORII
    % =====================================================

    fig = figure("Visible", "off", "Color", "w");

    plot(x, y, "LineWidth", 1.8);
    hold on;
    grid on;
    axis equal;

    plot(START(1), START(2), "ks", ...
        "MarkerSize", 8, ...
        "MarkerFaceColor", "k");

    plot(WP(:,1), WP(:,2), "ro", ...
        "MarkerSize", 8, ...
        "LineWidth", 1.5);

    for i = 1:size(WP,1)
        rysujOkrag(WP(i,1), WP(i,2), WAYPOINT_RADIUS);
        text(WP(i,1) + 1.2, WP(i,2) + 1.2, sprintf("WP%d", i), ...
            "FontWeight", "bold", ...
            "FontSize", 9);
    end

    text(START(1) + 1.2, START(2) - 2.0, "START", ...
        "FontWeight", "bold", ...
        "FontSize", 9);

    xlabel("x [m]");
    ylabel("y [m]");
    title(sprintf("Trajektoria ruchu katamaranu - %s", nazwa_na_wykresie), ...
        "Interpreter", "none");

    legend("trajektoria", "start", "waypointy", ...
        "Location", "best");

    sciezka_png = fullfile(folder_wyj, nazwa_pliku + "_trajektoria.png");
    exportgraphics(fig, sciezka_png, "Resolution", 300);
    close(fig);

    %% ====================================================
    %  2. WYKRES SYGNAŁÓW STERUJĄCYCH
    % =====================================================

    fig = figure("Visible", "off", "Color", "w");

    h1 = plot(t, base_cmd, "LineWidth", 1.5);
    hold on;
    h2 = plot(t, diff_cmd, "LineWidth", 1.5);
    grid on;

    dodajLiniePrzejscWP(czasy_przejsc, KOLOR_PRZEJSCIA_WP, SZEROKOSC_LINII_WP);

    xlabel("Czas [s]");
    ylabel("Sygnał sterujący");
    title(sprintf("Sygnały sterujące regulatora - %s", nazwa_na_wykresie), ...
        "Interpreter", "none");

    legend([h1, h2], ...
        {"base\_cmd", "diff\_cmd"}, ...
        "Location", "best");

    axis tight;

    sciezka_png = fullfile(folder_wyj, nazwa_pliku + "_sterowanie.png");
    exportgraphics(fig, sciezka_png, "Resolution", 300);
    close(fig);

    %% ====================================================
    %  3. WYKRES CIĄGU SILNIKÓW
    % =====================================================

    fig = figure("Visible", "off", "Color", "w");

    h1 = plot(t, T_left, "LineWidth", 1.5);
    hold on;
    h2 = plot(t, T_right, "LineWidth", 1.5);
    grid on;

    dodajLiniePrzejscWP(czasy_przejsc, KOLOR_PRZEJSCIA_WP, SZEROKOSC_LINII_WP);

    xlabel("Czas [s]");
    ylabel("Ciąg silnika [N]");
    title(sprintf("Ciąg lewego i prawego silnika - %s", nazwa_na_wykresie), ...
        "Interpreter", "none");

    legend([h1, h2], ...
        {"T\_left", "T\_right"}, ...
        "Location", "best");

    axis tight;

    sciezka_png = fullfile(folder_wyj, nazwa_pliku + "_ciag.png");
    exportgraphics(fig, sciezka_png, "Resolution", 300);
    close(fig);

    %% ====================================================
    %  4. WYKRES J_TOTAL
    % =====================================================

    fig = figure("Visible", "off", "Color", "w");

    h1 = plot(t, J_total, "LineWidth", 1.8);
    hold on;
    grid on;

    dodajLiniePrzejscWP(czasy_przejsc, KOLOR_PRZEJSCIA_WP, SZEROKOSC_LINII_WP);

    xlabel("Czas [s]");
    ylabel("J_{total}");
    title(sprintf("Całkowity wskaźnik jakości w czasie - %s", nazwa_na_wykresie), ...
        "Interpreter", "none");

    legend(h1, {"J\_total"}, ...
        "Location", "best");

    axis tight;

    sciezka_png = fullfile(folder_wyj, nazwa_pliku + "_J_total.png");
    exportgraphics(fig, sciezka_png, "Resolution", 300);
    close(fig);

    %% ====================================================
    %  5. WYKRES ROLL
    % =====================================================

    fig = figure("Visible", "off", "Color", "w");

    h1 = plot(t, roll_deg, "LineWidth", 1.5);
    hold on;
    grid on;

    yline(0, "--", ...
        "HandleVisibility", "off");

    dodajLiniePrzejscWP(czasy_przejsc, KOLOR_PRZEJSCIA_WP, SZEROKOSC_LINII_WP);

    xlabel("Czas [s]");
    ylabel("Roll [deg]");
    title(sprintf("Przechył poprzeczny jednostki - %s", nazwa_na_wykresie), ...
        "Interpreter", "none");

    legend(h1, {"roll"}, ...
        "Location", "best");

    axis tight;

    sciezka_png = fullfile(folder_wyj, nazwa_pliku + "_roll.png");
    exportgraphics(fig, sciezka_png, "Resolution", 300);
    close(fig);

    %% ====================================================
    %  TABELE WSKAŹNIKÓW I STATYSTYK
    % =====================================================

    wskazniki = table( ...
        ostatniaWartosc(I_roll), ...
        ostatniaWartosc(I_pitch), ...
        ostatniaWartosc(I_roll_rate), ...
        ostatniaWartosc(I_pitch_rate), ...
        ostatniaWartosc(I_control), ...
        ostatniaWartosc(I_no_progress), ...
        ostatniaWartosc(I_goal_heading), ...
        ostatniaWartosc(J_total), ...
        'VariableNames', { ...
            'I_roll', ...
            'I_pitch', ...
            'I_roll_rate', ...
            'I_pitch_rate', ...
            'I_control', ...
            'I_no_progress', ...
            'I_goal_heading', ...
            'J_total' ...
        } ...
    );

    statystyki = table( ...
        mean(T_left, "omitnan"), ...
        max(T_left, [], "omitnan"), ...
        min(T_left, [], "omitnan"), ...
        mean(T_right, "omitnan"), ...
        max(T_right, [], "omitnan"), ...
        min(T_right, [], "omitnan"), ...
        mean(base_cmd, "omitnan"), ...
        max(base_cmd, [], "omitnan"), ...
        min(base_cmd, [], "omitnan"), ...
        mean(diff_cmd, "omitnan"), ...
        max(diff_cmd, [], "omitnan"), ...
        min(diff_cmd, [], "omitnan"), ...
        'VariableNames', { ...
            'T_left_mean', ...
            'T_left_max', ...
            'T_left_min', ...
            'T_right_mean', ...
            'T_right_max', ...
            'T_right_min', ...
            'base_cmd_mean', ...
            'base_cmd_max', ...
            'base_cmd_min', ...
            'diff_cmd_mean', ...
            'diff_cmd_max', ...
            'diff_cmd_min' ...
        } ...
    );

    tabela_przejsc = table( ...
        ["WP1_WP2"; "WP2_WP3"], ...
        [t_wp12; t_wp23], ...
        [idx_wp12; idx_wp23], ...
        'VariableNames', { ...
            'przejscie', ...
            'czas_s', ...
            'indeks_probki' ...
        } ...
    );

    writetable(wskazniki, fullfile(folder_wyj, nazwa_pliku + "_wskazniki.csv"), ...
        "Delimiter", ";");

    writetable(statystyki, fullfile(folder_wyj, nazwa_pliku + "_statystyki.csv"), ...
        "Delimiter", ";");

    writetable(tabela_przejsc, fullfile(folder_wyj, nazwa_pliku + "_przejscia_WP.csv"), ...
        "Delimiter", ";");

    if ZAPISZ_XLSX
        sciezka_xlsx = fullfile(folder_wyj, nazwa_pliku + "_tabele.xlsx");

        try
            writetable(wskazniki, sciezka_xlsx, "Sheet", "Wskazniki");
            writetable(statystyki, sciezka_xlsx, "Sheet", "Statystyki");
            writetable(tabela_przejsc, sciezka_xlsx, "Sheet", "Przejscia_WP");
        catch ME
            warning("Nie udało się zapisać XLSX dla pliku %s: %s", nazwa_pliku, ME.message);
        end
    end

    %% ----------------------------------------------------
    %  Dopisanie do tabeli zbiorczej
    % -----------------------------------------------------

    nowy_wiersz = table( ...
        nazwa_pliku, ...
        folder_regulatora, ...
        wariant, ...
        ostatniaWartosc(I_roll), ...
        ostatniaWartosc(I_pitch), ...
        ostatniaWartosc(I_roll_rate), ...
        ostatniaWartosc(I_pitch_rate), ...
        ostatniaWartosc(I_control), ...
        ostatniaWartosc(I_no_progress), ...
        ostatniaWartosc(I_goal_heading), ...
        ostatniaWartosc(J_total), ...
        mean(T_left, "omitnan"), ...
        max(T_left, [], "omitnan"), ...
        min(T_left, [], "omitnan"), ...
        mean(T_right, "omitnan"), ...
        max(T_right, [], "omitnan"), ...
        min(T_right, [], "omitnan"), ...
        t_wp12, ...
        t_wp23, ...
        'VariableNames', { ...
            'plik', ...
            'regulator', ...
            'wariant', ...
            'I_roll', ...
            'I_pitch', ...
            'I_roll_rate', ...
            'I_pitch_rate', ...
            'I_control', ...
            'I_no_progress', ...
            'I_goal_heading', ...
            'J_total', ...
            'T_left_mean', ...
            'T_left_max', ...
            'T_left_min', ...
            'T_right_mean', ...
            'T_right_max', ...
            'T_right_min', ...
            'czas_przejscia_WP1_WP2_s', ...
            'czas_przejscia_WP2_WP3_s' ...
        } ...
    );

    podsumowanie = [podsumowanie; nowy_wiersz];

end

%% ========================================================
%  ZAPIS TABELI ZBIORCZEJ
% =========================================================

if ~isempty(podsumowanie)
    writetable(podsumowanie, fullfile(KATALOG_WYKRESOW, "podsumowanie_wszystkich_przebiegow.csv"), ...
        "Delimiter", ";");

    if ZAPISZ_XLSX
        try
            writetable(podsumowanie, fullfile(KATALOG_WYKRESOW, "podsumowanie_wszystkich_przebiegow.xlsx"));
        catch ME
            warning("Nie udało się zapisać zbiorczego XLSX: %s", ME.message);
        end
    end
end

fprintf("\nGotowe. Wykresy i tabele zapisano w katalogu: %s\n", KATALOG_WYKRESOW);

%% ========================================================
%  FUNKCJE POMOCNICZE
% =========================================================

function col = pobierzKolumne(dane, nazwa, n)

    nazwy = string(dane.Properties.VariableNames);

    if any(nazwy == nazwa)
        col = dane.(nazwa);
    else
        warning("Brak kolumny: %s. Wstawiam NaN.", nazwa);
        col = NaN(n, 1);
    end

    if iscell(col)
        col = str2double(string(col));
    end

    if isstring(col) || ischar(col)
        col = str2double(col);
    end

end

function val = ostatniaWartosc(x)

    idx = find(~isnan(x), 1, "last");

    if isempty(idx)
        val = NaN;
    else
        val = x(idx);
    end

end

function wariant = wykryjWariant(nazwa_pliku)

    token = regexp(char(nazwa_pliku), "\d+_\d+", "match", "once");

    if isempty(token)
        wariant = "brak";
    else
        wariant = string(token);
    end

end

function [czy_jest, idx_zmiany, t_zmiany] = znajdzPrzejscieWP(wp_idx, t, wp_from, wp_to)

    czy_jest = false;
    idx_zmiany = NaN;
    t_zmiany = NaN;

    if all(isnan(wp_idx))
        return;
    end

    % Podstawowy przypadek: numeracja WP jako 1, 2, 3.
    idx = find(wp_idx(1:end-1) == wp_from & wp_idx(2:end) == wp_to, 1, "first");

    % Awaryjnie: gdyby numeracja była od zera, czyli 0, 1, 2.
    if isempty(idx)
        idx = find(wp_idx(1:end-1) == (wp_from - 1) & wp_idx(2:end) == (wp_to - 1), 1, "first");
    end

    if ~isempty(idx)
        idx_zmiany = idx + 1;
        t_zmiany = t(idx_zmiany);
        czy_jest = true;
    end

end

function dodajLiniePrzejscWP(czasy_przejsc, kolor, szerokosc)

    if isempty(czasy_przejsc)
        return;
    end

    for i = 1:numel(czasy_przejsc)
        if ~isnan(czasy_przejsc(i))
            xline(czasy_przejsc(i), "-", ...
                "Color", kolor, ...
                "LineWidth", szerokosc, ...
                "HandleVisibility", "off");
        end
    end

end

function rysujOkrag(x0, y0, r)

    theta = linspace(0, 2*pi, 200);

    x = x0 + r * cos(theta);
    y = y0 + r * sin(theta);

    plot(x, y, "r--", ...
        "LineWidth", 1.0, ...
        "HandleVisibility", "off");

end