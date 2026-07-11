clc
clear
close all

%% ==============================================================
%              MODELO DO MOTOR CC E CONTROLADOR PI
% ==============================================================

%% Parâmetros do motor CC

Ra = 0.158109983;        % Resistência da armadura [ohm]
La = 94.87e-6;           % Indutância da armadura [H]

Ke = 0.009530212;        % Constante de força contraeletromotriz [V.s/rad]
Kt = 0.009737475;        % Constante de torque [N.m/A]

Bm = 4.48491e-6;         % Coeficiente de atrito viscoso [N.m.s/rad]
Tc = 0.006683397;        % Torque de atrito seco [N.m]
Jm = 2.03448e-5;         % Momento de inércia [kg.m^2]

Ktaco = 0.0035014;       % = 0.00022 * (100/60) * (60/(2*pi))

%% Escalas utilizadas

Vbus = 12;               % Tensão máxima aplicada ao motor [V]
RPM_max = 12000;         % Velocidade máxima considerada [RPM]

%% Valores experimentais para comparação

tau_eletrica_exp = 600e-6;       % Constante elétrica experimental [s]
tau_mecanica_exp = 34.4e-3;      % Constante mecânica experimental [s]

%% Variável de Laplace

s = tf('s');

%% ==============================================================
%                    PLANTA FÍSICA DO MOTOR
% ==============================================================

num_w = Kt;

den_w = [
    La*Jm, ...
    La*Bm + Ra*Jm, ...
    Ra*Bm + Kt*Ke
];

G_w = tf(num_w, den_w);          % [rad/s]/V
G_rpm = G_w*(60/(2*pi));         % [RPM]/V

%% ==============================================================
%              PLANTA VISTA PELO CONTROLADOR
% ==============================================================

% A planta G_rpm possui entrada em tensão [V] e saída em velocidade [RPM].
% Para que o controlador trabalhe com entrada e saída na mesma escala,
% considera-se que o comando máximo corresponde a RPM_max e aplica Vbus no motor.

G_equivalente = G_rpm*(Vbus/RPM_max);

G = G_equivalente;

%% ==============================================================
%                       CONTROLADOR PI
% ==============================================================

Kp = 1;
Ki = 30.3;

C = Kp + Ki/s;

L = C*G;              % Malha aberta compensada
T = feedback(L, 1);   % Malha fechada com realimentação unitária

%% ==============================================================
%                    RESULTADOS PRINCIPAIS
% ==============================================================

[num_G, den_G] = tfdata(G, 'v');
[num_T, den_T] = tfdata(T, 'v');

p_planta = pole(G_w);
tau_planta = -1./real(p_planta);

tau_eletrica_modelo = min(tau_planta);
tau_mecanica_modelo = max(tau_planta);

erro_tau_eletrica = abs((tau_eletrica_modelo - tau_eletrica_exp)/tau_eletrica_exp)*100;
erro_tau_mecanica = abs((tau_mecanica_modelo - tau_mecanica_exp)/tau_mecanica_exp)*100;

p_mf = pole(T);
tau_mf = -1./real(p_mf);

Kdc_G = dcgain(G);
Kdc_T = dcgain(T);

info_G = stepinfo(G);
info_T = stepinfo(T);

[GM, PM, Wcg, Wcp] = margin(L);

disp('==============================================================')
disp('              RESULTADOS PRINCIPAIS')
disp('==============================================================')
disp(' ')

fprintf('Controlador PI:\n')
fprintf('Kp = %.6f\n', Kp)
fprintf('Ki = %.6f\n\n', Ki)

fprintf('Polos da planta física:\n')
for i = 1:length(p_planta)
    fprintf('p%d = %.6f rad/s\n', i, real(p_planta(i)))
end
fprintf('\n')

fprintf('Constantes de tempo da planta física:\n')
for i = 1:length(tau_planta)
    fprintf('tau%d = %.9f s = %.6f ms\n', ...
        i, tau_planta(i), tau_planta(i)*1000)
end
fprintf('\n')

fprintf('Comparação com valores experimentais:\n')
fprintf('Constante elétrica do modelo: %.6f ms\n', tau_eletrica_modelo*1000)
fprintf('Constante elétrica experimental: %.6f ms\n', tau_eletrica_exp*1000)
fprintf('Erro relativo: %.2f %%\n\n', erro_tau_eletrica)

fprintf('Constante mecânica do modelo: %.6f ms\n', tau_mecanica_modelo*1000)
fprintf('Constante mecânica experimental: %.6f ms\n', tau_mecanica_exp*1000)
fprintf('Erro relativo: %.2f %%\n\n', erro_tau_mecanica)

fprintf('Polos do sistema em malha fechada:\n')
for i = 1:length(p_mf)
    if abs(imag(p_mf(i))) < 1e-9
        fprintf('p%d = %.6f rad/s\n', i, real(p_mf(i)))
    else
        fprintf('p%d = %.6f %+.6fi rad/s\n', ...
            i, real(p_mf(i)), imag(p_mf(i)))
    end
end
fprintf('\n')

fprintf('Ganho DC da planta equivalente: %.6f\n', Kdc_G)
fprintf('Ganho DC em malha fechada: %.6f\n\n', Kdc_T)

fprintf('Resposta ao degrau:\n')
fprintf('Tempo de subida da planta: %.6f s\n', info_G.RiseTime)
fprintf('Tempo de acomodação da planta: %.6f s\n', info_G.SettlingTime)
fprintf('Tempo de subida em malha fechada: %.6f s\n', info_T.RiseTime)
fprintf('Tempo de acomodação em malha fechada: %.6f s\n', info_T.SettlingTime)
fprintf('Sobressinal em malha fechada: %.6f %%\n\n', info_T.Overshoot)

fprintf('Margens de estabilidade da malha aberta compensada:\n')
fprintf('Margem de ganho: %.6f\n', GM)
fprintf('Margem de ganho [dB]: %.6f dB\n', 20*log10(GM))
fprintf('Margem de fase: %.6f graus\n', PM)
fprintf('Frequência de cruzamento de ganho: %.6f rad/s\n', Wcp)
fprintf('Frequência de cruzamento de fase: %.6f rad/s\n', Wcg)
disp(' ')

%% ==============================================================
%                         GRÁFICOS
% ==============================================================

%% Lugar das raízes da planta equivalente

plotarLugarRaizes( ...
    G, ...
    'Lugar das raízes da planta equivalente', ...
    false);


%% Lugar das raízes com controlador PI

plotarLugarRaizes( ...
    L, ...
    'Lugar das raízes com controlador PI', ...
    true);

%% Diagrama de Bode da malha aberta compensada

figure
margin(L)
grid on
title('Diagrama de Bode da malha aberta compensada')

drawnow

% Todas as linhas inicialmente vermelhas
linhas = findall(gcf, 'Type', 'line');

set(linhas, 'Color', [1 0 0], 'LineWidth', 1.2)

% Identifica as duas curvas principais pela quantidade de pontos
quantidadePontos = arrayfun(@(h) numel(h.XData), linhas);
[~, ordem] = sort(quantidadePontos, 'descend');

curvasPrincipais = linhas(ordem(1:2));

% Curvas de magnitude e fase em preto
set(curvasPrincipais, 'Color', [0 0 1], 'LineWidth', 2)

%% Resposta física em RPM para degrau de 12 V no motor

figure
step(G_rpm*Vbus, 2)
grid on
title('Resposta física do motor para degrau de 12 V')
xlabel('Tempo [s]')
ylabel('Velocidade [RPM]')

%% Resposta da planta equivalente ao degrau

figure
step(G, 2)
grid on
title('Resposta da planta equivalente ao degrau')
xlabel('Tempo [s]')
ylabel('Saída na mesma escala da entrada')

%% Resposta em malha fechada com controlador PI

figure
step(T, 2)
grid on
title('Resposta em malha fechada com controlador PI')
xlabel('Tempo [s]')
ylabel('Saída controlada')

%% Comparação entre planta equivalente e sistema controlado

figure

% Planta G(s)
step(G, 2)
hG = findall(gca, 'Type', 'line');
set(hG, 'Color', [1 0 0], 'LineWidth', 1.2)

hold on

% Sistema controlado T(s)
step(T, 2)
hTodas = findall(gca, 'Type', 'line');
hT = setdiff(hTodas, hG);

set(hT, 'Color', [0 0 1], 'LineWidth', 1.2)

grid on
title('Comparação entre planta equivalente e sistema controlado')
xlabel('Tempo [s]')
ylabel('Saída')

axis([0 0.4 0 1.1])

set(gca, 'FontSize', 12, 'LineWidth', 1.2)

legend('Malha fechada T(s)', 'Planta G(s)', 'Location', 'best')

%% ==============================================================
%                 INTERFACE INTERATIVA OPCIONAL
% ==============================================================

% rltool(G)
% rltool(L)




%% Função principal

function plotarLugarRaizes(sys, tituloGrafico, mostrarAmpliacao)

    fig = figure( ...
        'Color', 'w', ...
        'Position', [100 100 950 560]);

    ax = axes(fig);

    axes(ax)
    rlocus(sys)

    hold(ax, 'on')
    grid(ax, 'on')
    drawnow

    % Limites dos gráficos
    xlim(ax, [-1800 100])
    ylim(ax, [-100 100])

    title(ax, tituloGrafico, ...
        'FontSize', 13, ...
        'FontWeight', 'bold')

    xlabel(ax, 'Eixo real')
    ylabel(ax, 'Eixo imaginário')

    set(ax, ...
        'FontSize', 12, ...
        'LineWidth', 1.5, ...
        'Box', 'on', ...
        'Layer', 'top')

    % Formata as curvas e remove os marcadores automáticos
    formatarCurvas(ax)

    % Deixa as linhas pontilhadas em verde
    linhasGrade = findall(ax, ...
        'Type', 'line', ...
        'LineStyle', ':');

    set(linhasGrade, ...
        'Color', [0.75 0.75 0.75], ...
        'LineWidth', 0.8)

    % Polos e zeros de malha aberta
    p = pole(sys);
    z = zero(sys);

    hPolos = plot(ax, real(p), imag(p), ...
        'x', ...
        'Color', [1 0 0], ...
        'MarkerSize', 11, ...
        'LineWidth', 1.5, ...
        'LineStyle', 'none');

    if ~isempty(z)

        hZeros = plot(ax, real(z), imag(z), ...
            'o', ...
            'Color', [0 0 1], ...
            'MarkerSize', 10, ...
            'LineWidth', 1.5, ...
            'LineStyle', 'none');

        legenda = legend(ax, ...
            [hPolos hZeros], ...
            {'Polos', ...
             'Zeros'}, ...
            'Location', 'best');

    else

        legenda = legend(ax, ...
            hPolos, ...
            {'Polos'}, ...
            'Location', 'best');

    end

    % Caixa da legenda
    set(legenda, ...
        'Box', 'on', ...
        'Color', 'w', ...
        'EdgeColor', [0 0 0], ...
        'LineWidth', 0.8, ...
        'FontSize', 10)


    %% Ampliação do polo e zero próximos de -30

    if mostrarAmpliacao

        axZoom = axes(fig, ...
            'Position', [0.57 0.2 0.30 0.28], ...
            'Color', 'w', ...
            'Box', 'on');

        axes(axZoom)
        rlocus(sys)

        hold(axZoom, 'on')
        grid(axZoom, 'on')
        drawnow

        % Região ampliada
        xlim(axZoom, [-36 0.5])
        ylim(axZoom, [-5 5])

        % Curvas pretas e marcadores automáticos removidos
        formatarCurvas(axZoom)

        % Polos destacados
        plot(axZoom, real(p), imag(p), ...
            'x', ...
            'Color', [1 0 0], ...
            'MarkerSize', 8, ...
            'LineWidth', 1.5, ...
            'LineStyle', 'none', ...
            'HandleVisibility', 'off');

        % Zeros destacados
        if ~isempty(z)

            plot(axZoom, real(z), imag(z), ...
                'o', ...
                'Color', [0 0 1], ...
                'MarkerSize', 8, ...
                'LineWidth', 1.5, ...
                'LineStyle', 'none', ...
                'HandleVisibility', 'off');

        end

        title(axZoom, 'Ampliação próxima de -30', ...
            'Interpreter', 'latex', ...
            'FontSize', 9, ...
            'FontWeight', 'bold')

        xlabel(axZoom, '')
        ylabel(axZoom, '')

        

        set(axZoom, ...
            'FontSize', 8, ...
            'LineWidth', 1, ...
            'Box', 'on', ...
            'Layer', 'top')

        drawnow

        % Remove os textos de unidade "(seconds^{-1})" da ampliação
        objetosTexto = findall(fig, '-property', 'String');
        
        for i = 1:numel(objetosTexto)
        
            conteudo = string(get(objetosTexto(i), 'String'));
        
            if any(contains(conteudo, 'seconds', 'IgnoreCase', true))
                set(objetosTexto(i), 'Visible', 'off');
            end
        end
    end
end


%% Função de formatação das curvas

function formatarCurvas(ax)

    linhas = findall(ax, 'Type', 'line');

    for i = 1:numel(linhas)

        marcador = get(linhas(i), 'Marker');
        estilo    = get(linhas(i), 'LineStyle');

        % Esconde polos e zeros criados automaticamente pelo rlocus
        if strcmp(marcador, 'x') || strcmp(marcador, 'o')

            set(linhas(i), 'Visible', 'off')

        % Curvas do lugar das raízes em preto
        elseif strcmp(estilo, '-')

            set(linhas(i), ...
                'Color', [0.5 0.5 1], ...
                'LineWidth', 1.5)

        end
    end
end