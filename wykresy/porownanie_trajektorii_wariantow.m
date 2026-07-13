clear; clc; close all;

%% ========================================================
%  PORÓWNANIE TRAJEKTORII PID / MPC / SMC
%
%  Dla każdego wariantu, np. 0_0, 0_1, ..., 5_2,
%  skrypt nakłada na jeden wykres trajektorie wszystkich regulatorów.
% =========================================================

KATALOG_DANYCH   = "dane";
KATALOG_WYKRESOW = "wykresy_trajektorie_porownanie";

WAYPOINT_RADIUS = 4.0;
START = [-532, 190];
WP = [
    -482, 190;
    -482, 212;
    -532, 190
];

RYSUJ_TRASE_ZADANA = true;
ZAPISZ_FIG = false;

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

%% ========================================================
%  ZEBRANIE INFORMACJI O PLIKACH
% =========================================================

rekordy = struct("sciezka", {}, "nazwa", {}, "wariant", {}, "regulator", {});

for k = 1:numel(pliki)
    sciezka_csv = fullfile(pliki(k).folder, pliki(k).name);
    [~, nazwa_char, ~] = fileparts(pliki(k).name);
    nazwa = string(nazwa_char);

    wariant = wykryjWariant(nazwa);
    regulator = wykryjRegulator(sciezka_csv);

    if wariant == "brak" || regulator == "INNE"
        fprintf("Pomijam: %s\n", sciezka_csv);
        continue;
    end

    rekordy(end+1).sciezka = sciezka_csv; %#ok<SAGROW>
    rekordy(end).nazwa = nazwa;
    rekordy(end).wariant = wariant;
    rekordy(end).regulator = regulator;
end

if isempty(rekordy)
    error("Nie znaleziono plików możliwych do przypisania do PID, MPC lub SMC.");
end

warianty_znalezione = unique(string({rekordy.wariant}), "stable");

kolejnosc_wariantow = [
    "0_0", "0_1", "0_2", ...
    "2_0", "2_1", "2_2", ...
    "5_0", "5_1", "5_2"
];

warianty = kolejnosc_wariantow(ismember(kolejnosc_wariantow, warianty_znalezione));
warianty_dodatkowe = warianty_znalezione(~ismember(warianty_znalezione, warianty));
warianty = [warianty, warianty_dodatkowe];

fprintf("Znaleziono %d plików i %d wariantów.\n", numel(rekordy), numel(warianty));

%% ========================================================
%  JEDEN WYKRES TRAJEKTORII DLA KAŻDEGO WARIANTU
% =========================================================

kolejnosc_regulatorow = ["PID", "MPC", "SMC"];

for i = 1:numel(warianty)
    wariant = warianty(i);
    idx_wariantu = find(string({rekordy.wariant}) == wariant);

    if isempty(idx_wariantu)
        continue;
    end

    fig = figure("Visible", "off", "Color", "w", ...
        "Position", [100, 100, 1000, 700]);

    hold on;
    grid on;
    axis equal;

    uchwyty_trajektorii = gobjects(0);
    etykiety_trajektorii = strings(0);

    for r = 1:numel(kolejnosc_regulatorow)
        regulator = kolejnosc_regulatorow(r);

        idx_regulatora = idx_wariantu( ...
            string({rekordy(idx_wariantu).regulator}) == regulator ...
        );

        for j = 1:numel(idx_regulatora)
            rekord = rekordy(idx_regulatora(j));

            try
                dane = readtable(rekord.sciezka);
            catch ME
                warning("Nie udało się wczytać pliku %s: %s", ...
                    rekord.sciezka, ME.message);
                continue;
            end

            if ~all(ismember(["x", "y"], string(dane.Properties.VariableNames)))
                warning("Plik nie zawiera kolumn x i y: %s", rekord.sciezka);
                continue;
            end

            x = konwertujNaLiczby(dane.x);
            y = konwertujNaLiczby(dane.y);

            poprawne = isfinite(x) & isfinite(y);
            x = x(poprawne);
            y = y(poprawne);

            if numel(x) < 2
                warning("Za mało poprawnych próbek w pliku: %s", rekord.sciezka);
                continue;
            end

            kolor = kolorRegulatora(regulator);

            h = plot(x, y, "LineWidth", 1.8, "Color", kolor);
            uchwyty_trajektorii(end+1) = h; %#ok<SAGROW>

            if numel(idx_regulatora) == 1
                etykieta = regulator;
            else
                etykieta = sprintf("%s - przebieg %d", regulator, j);
            end

            etykiety_trajektorii(end+1) = string(etykieta); %#ok<SAGROW>
        end
    end

    if isempty(uchwyty_trajektorii)
        warning("Brak poprawnych trajektorii dla wariantu %s.", wariant);
        close(fig);
        continue;
    end

    %% Trasa zadana, start i waypointy

    uchwyty_dodatkowe = gobjects(0);
    etykiety_dodatkowe = strings(0);

    if RYSUJ_TRASE_ZADANA
        trasa_zadana = [START; WP];
        h_ref = plot(trasa_zadana(:,1), trasa_zadana(:,2), "k--", ...
            "LineWidth", 1.2);

        uchwyty_dodatkowe(end+1) = h_ref;
        etykiety_dodatkowe(end+1) = "trasa zadana";
    end

    h_start = plot(START(1), START(2), "ks", ...
        "MarkerSize", 8, "MarkerFaceColor", "k", "LineWidth", 1.2);

    h_wp = plot(WP(:,1), WP(:,2), "ro", ...
        "MarkerSize", 8, "LineWidth", 1.5);

    uchwyty_dodatkowe(end+1) = h_start;
    etykiety_dodatkowe(end+1) = "start";

    uchwyty_dodatkowe(end+1) = h_wp;
    etykiety_dodatkowe(end+1) = "waypointy";

    for w = 1:size(WP,1)
        rysujOkrag(WP(w,1), WP(w,2), WAYPOINT_RADIUS);

        text(WP(w,1) + 1.2, WP(w,2) + 1.2, sprintf("WP%d", w), ...
            "FontWeight", "bold", "FontSize", 9);
    end

    text(START(1) + 1.2, START(2) - 2.0, "START", ...
        "FontWeight", "bold", "FontSize", 9);

    xlabel("x [m]");
    ylabel("y [m]");

    title(sprintf("Porównanie trajektorii regulatorów - wariant %s", wariant), ...
        "Interpreter", "none");

    legend([uchwyty_trajektorii, uchwyty_dodatkowe], ...
        [cellstr(etykiety_trajektorii), cellstr(etykiety_dodatkowe)], ...
        "Location", "best", "Interpreter", "none");

    axis padded;

    %% Zapis wykresu

    sciezka_png = fullfile(KATALOG_WYKRESOW, ...
        "trajektorie_" + wariant + ".png");

    exportgraphics(fig, sciezka_png, "Resolution", 300);

    if ZAPISZ_FIG
        savefig(fig, fullfile(KATALOG_WYKRESOW, ...
            "trajektorie_" + wariant + ".fig"));
    end

    close(fig);
    fprintf("Zapisano: %s\n", sciezka_png);
end

fprintf("\nGotowe. Wykresy zapisano w katalogu: %s\n", KATALOG_WYKRESOW);

%% ========================================================
%  FUNKCJE POMOCNICZE
% =========================================================

function wariant = wykryjWariant(nazwa_pliku)
    token = regexp(char(nazwa_pliku), "\d+_\d+", "match", "once");

    if isempty(token)
        wariant = "brak";
    else
        wariant = string(token);
    end
end

function regulator = wykryjRegulator(sciezka)
    tekst = lower(string(sciezka));

    if contains(tekst, "pid")
        regulator = "PID";
    elseif contains(tekst, "mpc")
        regulator = "MPC";
    elseif contains(tekst, "smc")
        regulator = "SMC";
    else
        regulator = "INNE";
    end
end

function dane_num = konwertujNaLiczby(dane_we)
    if isnumeric(dane_we)
        dane_num = double(dane_we);
    elseif iscell(dane_we)
        dane_num = str2double(string(dane_we));
    elseif isstring(dane_we) || ischar(dane_we)
        dane_num = str2double(string(dane_we));
    else
        dane_num = double(dane_we);
    end

    dane_num = dane_num(:);
end

function kolor = kolorRegulatora(regulator)
    switch regulator
        case "PID"
            kolor = [0.0000, 0.4470, 0.7410];
        case "MPC"
            kolor = [0.8500, 0.3250, 0.0980];
        case "SMC"
            kolor = [0.4660, 0.6740, 0.1880];
        otherwise
            kolor = [0.3500, 0.3500, 0.3500];
    end
end

function rysujOkrag(x0, y0, r)
    theta = linspace(0, 2*pi, 200);
    x = x0 + r * cos(theta);
    y = y0 + r * sin(theta);

    plot(x, y, "r--", "LineWidth", 1.0, ...
        "HandleVisibility", "off");
end
