clear; clc; close all;

%% ========================================================
%  GĘSTOŚĆ ROZKŁADU PRZECHYŁU ROLL + WSKAŹNIKI
%
%  Dla każdego wariantu, np. 0_0, 0_1, ..., 5_2:
%  - nakłada funkcje gęstości roll dla PID, MPC i SMC,
%  - wyznacza wskaźniki opisujące przechył,
%  - zapisuje wykres PNG oraz tabelę CSV/XLSX.
%
%  Wymagana kolumna w CSV:
%      roll_deg
%
%  Opcjonalna kolumna:
%      t
%
%  Jeżeli dostępna jest funkcja ksdensity, wykorzystywana jest estymacja
%  jądrowa. W przeciwnym razie skrypt używa wygładzonego histogramu PDF.
%% ========================================================

KATALOG_DANYCH   = "dane";
KATALOG_WYKRESOW = "wykresy_gestosc_roll";
KATALOG_TABEL    = fullfile(KATALOG_WYKRESOW, "tabele");

ZAPISZ_XLSX = true;

PROG_ROLL_1_DEG = 3.0;
PROG_ROLL_2_DEG = 5.0;

LICZBA_PUNKTOW_GESTOSCI = 600;
LICZBA_PRZEDZIALOW_HIST = 80;

KOLOR_PID = [0.0000, 0.4470, 0.7410];
KOLOR_MPC = [0.8500, 0.3250, 0.0980];
KOLOR_SMC = [0.4660, 0.6740, 0.1880];

%% ========================================================
%  PRZYGOTOWANIE KATALOGÓW
%% ========================================================

if ~exist(KATALOG_WYKRESOW, "dir")
    mkdir(KATALOG_WYKRESOW);
end

if ~exist(KATALOG_TABEL, "dir")
    mkdir(KATALOG_TABEL);
end

%% ========================================================
%  WYSZUKIWANIE PLIKÓW CSV
%% ========================================================

pliki = dir(fullfile(KATALOG_DANYCH, "**", "*.csv"));

if isempty(pliki)
    error("Nie znaleziono plików CSV w katalogu: %s", KATALOG_DANYCH);
end

rekordy = struct( ...
    "sciezka", {}, ...
    "nazwa", {}, ...
    "wariant", {}, ...
    "regulator", {} ...
);

for k = 1:numel(pliki)

    sciezka_csv = fullfile(pliki(k).folder, pliki(k).name);
    [~, nazwa_char, ~] = fileparts(pliki(k).name);
    nazwa = string(nazwa_char);

    wariant = wykryjWariant(nazwa);
    regulator = wykryjRegulator(sciezka_csv);

    if wariant == "brak" || regulator == "INNE"
        fprintf("Pomijam plik bez rozpoznanego wariantu/regulatora: %s\n", ...
            sciezka_csv);
        continue;
    end

    rekordy(end+1).sciezka = sciezka_csv; %#ok<SAGROW>
    rekordy(end).nazwa = nazwa;
    rekordy(end).wariant = wariant;
    rekordy(end).regulator = regulator;
end

if isempty(rekordy)
    error("Nie znaleziono plików przypisanych do PID, MPC lub SMC.");
end

%% ========================================================
%  KOLEJNOŚĆ WARIANTÓW
%% ========================================================

warianty_znalezione = unique(string({rekordy.wariant}), "stable");

kolejnosc_wariantow = [
    "0_0", "0_1", "0_2", ...
    "2_0", "2_1", "2_2", ...
    "5_0", "5_1", "5_2"
];

warianty = kolejnosc_wariantow( ...
    ismember(kolejnosc_wariantow, warianty_znalezione) ...
);

warianty_dodatkowe = warianty_znalezione( ...
    ~ismember(warianty_znalezione, warianty) ...
);

warianty = [warianty, warianty_dodatkowe];
kolejnosc_regulatorow = ["PID", "MPC", "SMC"];

fprintf("Znaleziono %d plików i %d wariantów.\n", ...
    numel(rekordy), numel(warianty));

%% ========================================================
%  TABELA ZBIORCZA
%% ========================================================

podsumowanie = table();

%% ========================================================
%  PĘTLA PO WARIANTACH
%% ========================================================

for i = 1:numel(warianty)

    wariant = warianty(i);
    idx_wariantu = find(string({rekordy.wariant}) == wariant);

    if isempty(idx_wariantu)
        continue;
    end

    przebiegi = struct( ...
        "regulator", {}, ...
        "nazwa", {}, ...
        "t", {}, ...
        "roll", {}, ...
        "kolor", {} ...
    );

    wszystkie_roll = [];

    %% ----------------------------------------------------
    %  Wczytanie przebiegów dla danego wariantu
    %% ----------------------------------------------------

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

            nazwy_kolumn = string(dane.Properties.VariableNames);

            if ~any(nazwy_kolumn == "roll_deg")
                warning("Brak kolumny roll_deg w pliku: %s", rekord.sciezka);
                continue;
            end

            roll_deg = konwertujNaLiczby(dane.roll_deg);

            if any(nazwy_kolumn == "t")
                t = konwertujNaLiczby(dane.t);
            else
                t = (0:numel(roll_deg)-1).';
            end

            n = min(numel(t), numel(roll_deg));
            t = t(1:n);
            roll_deg = roll_deg(1:n);

            poprawne = isfinite(t) & isfinite(roll_deg);
            t = t(poprawne);
            roll_deg = roll_deg(poprawne);

            if numel(roll_deg) < 3
                warning("Za mało poprawnych próbek roll w pliku: %s", ...
                    rekord.sciezka);
                continue;
            end

            [t, idx_sort] = sort(t);
            roll_deg = roll_deg(idx_sort);

            przebiegi(end+1).regulator = regulator; %#ok<SAGROW>
            przebiegi(end).nazwa = rekord.nazwa;
            przebiegi(end).t = t;
            przebiegi(end).roll = roll_deg;
            przebiegi(end).kolor = kolorRegulatora( ...
                regulator, KOLOR_PID, KOLOR_MPC, KOLOR_SMC);

            wszystkie_roll = [wszystkie_roll; roll_deg]; %#ok<AGROW>
        end
    end

    if isempty(przebiegi)
        warning("Brak poprawnych danych roll dla wariantu %s.", wariant);
        continue;
    end

    %% ----------------------------------------------------
    %  Wspólna oś funkcji gęstości
    %% ----------------------------------------------------

    min_roll = min(wszystkie_roll);
    max_roll = max(wszystkie_roll);

    if min_roll == max_roll
        min_roll = min_roll - 1;
        max_roll = max_roll + 1;
    end

    margines = 0.08 * (max_roll - min_roll);

    x_gestosc = linspace( ...
        min_roll - margines, ...
        max_roll + margines, ...
        LICZBA_PUNKTOW_GESTOSCI ...
    );

    %% ----------------------------------------------------
    %  Wykres funkcji gęstości
    %% ----------------------------------------------------

    fig = figure( ...
        "Visible", "off", ...
        "Color", "w", ...
        "Position", [100, 100, 1000, 700] ...
    );

    hold on;
    grid on;

    uchwyty = gobjects(0);
    etykiety = strings(0);
    tabela_wariantu = table();

    for p = 1:numel(przebiegi)

        regulator = przebiegi(p).regulator;
        roll_deg = przebiegi(p).roll;
        t = przebiegi(p).t;
        kolor = przebiegi(p).kolor;

        gestosc = obliczGestosc( ...
            roll_deg, x_gestosc, LICZBA_PRZEDZIALOW_HIST);

        h = plot(x_gestosc, gestosc, ...
            "LineWidth", 2.0, ...
            "Color", kolor);

        uchwyty(end+1) = h; %#ok<SAGROW>

        liczba_tego_regulatora = sum( ...
            string({przebiegi.regulator}) == regulator ...
        );

        if liczba_tego_regulatora == 1
            etykieta = regulator;
        else
            nr_przebiegu = sum( ...
                string({przebiegi(1:p).regulator}) == regulator ...
            );
            etykieta = sprintf("%s - przebieg %d", regulator, nr_przebiegu);
        end

        etykiety(end+1) = string(etykieta); %#ok<SAGROW>

        %% Wskaźniki przechyłu

        wsk = obliczWskaznikiRoll( ...
            t, roll_deg, PROG_ROLL_1_DEG, PROG_ROLL_2_DEG);

        nowy_wiersz = table( ...
            wariant, ...
            regulator, ...
            przebiegi(p).nazwa, ...
            wsk.mean_deg, ...
            wsk.mean_abs_deg, ...
            wsk.rms_deg, ...
            wsk.std_deg, ...
            wsk.max_abs_deg, ...
            wsk.p95_abs_deg, ...
            wsk.iae_deg_s, ...
            wsk.ise_deg2_s, ...
            wsk.czas_powyzej_progu1_proc, ...
            wsk.czas_powyzej_progu2_proc, ...
            'VariableNames', { ...
                'wariant', ...
                'regulator', ...
                'plik', ...
                'mean_roll_deg', ...
                'mean_abs_roll_deg', ...
                'rms_roll_deg', ...
                'std_roll_deg', ...
                'max_abs_roll_deg', ...
                'p95_abs_roll_deg', ...
                'IAE_roll_deg_s', ...
                'ISE_roll_deg2_s', ...
                sprintf('czas_abs_roll_gt_%gdeg_proc', PROG_ROLL_1_DEG), ...
                sprintf('czas_abs_roll_gt_%gdeg_proc', PROG_ROLL_2_DEG) ...
            } ...
        );

        tabela_wariantu = [tabela_wariantu; nowy_wiersz]; %#ok<AGROW>
        podsumowanie = [podsumowanie; nowy_wiersz]; %#ok<AGROW>
    end

    xline(0, "k--", ...
        "LineWidth", 1.0, ...
        "HandleVisibility", "off");

    xlabel("Przechył roll [deg]");
    ylabel("Gęstość prawdopodobieństwa");

    title(sprintf( ...
        "Rozkład przechyłu poprzecznego - wariant %s", wariant), ...
        "Interpreter", "none" ...
    );

    legend(uchwyty, cellstr(etykiety), ...
        "Location", "best", ...
        "Interpreter", "none");

    axis tight;

    sciezka_png = fullfile( ...
        KATALOG_WYKRESOW, ...
        "gestosc_roll_" + wariant + ".png" ...
    );

    exportgraphics(fig, sciezka_png, "Resolution", 300);
    close(fig);

    %% ----------------------------------------------------
    %  Zapis tabeli dla wariantu
    %% ----------------------------------------------------

    sciezka_csv = fullfile( ...
        KATALOG_TABEL, ...
        "wskazniki_roll_" + wariant + ".csv" ...
    );

    writetable(tabela_wariantu, sciezka_csv, "Delimiter", ";");

    if ZAPISZ_XLSX
        sciezka_xlsx = fullfile( ...
            KATALOG_TABEL, ...
            "wskazniki_roll_" + wariant + ".xlsx" ...
        );

        try
            writetable(tabela_wariantu, sciezka_xlsx);
        catch ME
            warning("Nie udało się zapisać XLSX dla wariantu %s: %s", ...
                wariant, ME.message);
        end
    end

    fprintf("Zapisano wariant %s.\n", wariant);
end

%% ========================================================
%  ZAPIS TABELI ZBIORCZEJ
%% ========================================================

if ~isempty(podsumowanie)

    writetable( ...
        podsumowanie, ...
        fullfile(KATALOG_TABEL, "wskazniki_roll_wszystkie.csv"), ...
        "Delimiter", ";" ...
    );

    if ZAPISZ_XLSX
        try
            writetable( ...
                podsumowanie, ...
                fullfile(KATALOG_TABEL, "wskazniki_roll_wszystkie.xlsx") ...
            );
        catch ME
            warning("Nie udało się zapisać zbiorczego XLSX: %s", ME.message);
        end
    end
end

fprintf("\nGotowe. Wyniki zapisano w katalogu: %s\n", KATALOG_WYKRESOW);

%% ========================================================
%  FUNKCJE POMOCNICZE
%% ========================================================

function gestosc = obliczGestosc(roll_deg, x_gestosc, liczba_przedzialow)

    if exist("ksdensity", "file") == 2
        gestosc = ksdensity(roll_deg, x_gestosc, "Function", "pdf");
        return;
    end

    [licznosc, krawedzie] = histcounts( ...
        roll_deg, ...
        liczba_przedzialow, ...
        "Normalization", "pdf" ...
    );

    srodki = (krawedzie(1:end-1) + krawedzie(2:end)) / 2;

    if numel(licznosc) >= 5
        licznosc = smoothdata(licznosc, "gaussian", 5);
    end

    gestosc = interp1( ...
        srodki, ...
        licznosc, ...
        x_gestosc, ...
        "pchip", ...
        0 ...
    );

    gestosc(gestosc < 0) = 0;
end

function wsk = obliczWskaznikiRoll(t, roll_deg, prog1, prog2)

    roll_abs = abs(roll_deg);

    wsk.mean_deg = mean(roll_deg, "omitnan");
    wsk.mean_abs_deg = mean(roll_abs, "omitnan");
    wsk.rms_deg = sqrt(mean(roll_deg.^2, "omitnan"));
    wsk.std_deg = std(roll_deg, "omitnan");
    wsk.max_abs_deg = max(roll_abs, [], "omitnan");
    wsk.p95_abs_deg = percentylLokalny(roll_abs, 95);

    if numel(t) >= 2 && max(t) > min(t)

        czas_trwania = t(end) - t(1);

        wsk.iae_deg_s = trapz(t, roll_abs);
        wsk.ise_deg2_s = trapz(t, roll_deg.^2);

        wsk.czas_powyzej_progu1_proc = ...
            100 * trapz(t, double(roll_abs > prog1)) / czas_trwania;

        wsk.czas_powyzej_progu2_proc = ...
            100 * trapz(t, double(roll_abs > prog2)) / czas_trwania;

    else

        wsk.iae_deg_s = sum(roll_abs, "omitnan");
        wsk.ise_deg2_s = sum(roll_deg.^2, "omitnan");

        wsk.czas_powyzej_progu1_proc = ...
            100 * mean(roll_abs > prog1, "omitnan");

        wsk.czas_powyzej_progu2_proc = ...
            100 * mean(roll_abs > prog2, "omitnan");
    end
end

function p = percentylLokalny(x, procent)

    x = sort(x(isfinite(x)));

    if isempty(x)
        p = NaN;
        return;
    end

    if numel(x) == 1
        p = x(1);
        return;
    end

    pozycja = 1 + (numel(x) - 1) * procent / 100;
    dol = floor(pozycja);
    gora = ceil(pozycja);

    if dol == gora
        p = x(dol);
    else
        alfa = pozycja - dol;
        p = x(dol) + alfa * (x(gora) - x(dol));
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

function kolor = kolorRegulatora(regulator, kolor_pid, kolor_mpc, kolor_smc)

    switch regulator
        case "PID"
            kolor = kolor_pid;
        case "MPC"
            kolor = kolor_mpc;
        case "SMC"
            kolor = kolor_smc;
        otherwise
            kolor = [0.35, 0.35, 0.35];
    end
end
