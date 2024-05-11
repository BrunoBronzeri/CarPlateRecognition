function info = f_bb_analisa_regioes(I, in, num, edge)

% analise de componentes conectados
[Ilabel, N] = bwlabel(I);

figure; imshow(I); axis on

info = cell(N,1);

m00 = [1,N];
m10 = [1,N];
m01 = [1,N];

vc = [1,N];
uc = [1,N];

for i=1:N
    % Região de Interesse 
    I2 = (Ilabel == i);
    
    % Bounding Box
    [v,u] = find(I2);
    vmin = min(v);
    umin = min(u);
    vmax = max(v);
    umax = max(u);
    
    bb = [umin, vmin, umax, vmax];
    
    hold on, plot([ umin , umin ], [ vmin , vmax ], 'y');
    hold on, plot([ umin , umax ], [ vmin, vmin ], 'y');
    hold on, plot([ umax , umax ], [ vmin, vmax ], 'y');
    hold on, plot([ umin , umax ], [ vmax, vmax ], 'y');
    
    % Momentos de ordem 0 e 1
    k0 = 0;
    k10 = 0;
    k01 = 0;
    for u = umin:umax
        for v = vmin:vmax
            if(I(v,u)==1)
               k0 = k0+1;
               k10 = k10 + u;
               k01 = k01 + v;
            end
        end
    end
    m00(1,i) = k0;
    m10(1,i) = k10;
    m01(1,i) = k01;
    
    % Centroides
    vc(1,i) = m01(1,i)/m00(1,i);
    uc(1,i) = m10(1,i)/m00(1,i);
    
    hold on, plot(uc(1,i) , vc(1,i), '+');
    
    % Plot da identificação numérica das partes
    str = num2str(i);
    hold on, text(uc(1,i)+10, vc(1,i)-15, str, 'Color',...
        'black', 'FontSize', 10, 'BackgroundColor', 'yellow'); 
    
    % Momentos centrais de ordem 1 e 2
    u11 = 0;
    u20 = 0;
    u02 = 0;
    for u = umin:umax
        for v = vmin:vmax
            if(I(v,u)==1)
               u11 = u11 + (u-uc(1,i))*(v-vc(1,i));
               u20 = u20 + (u-uc(1,i))^2;
               u02 = u02 + (v-vc(1,i))^2;
            end
        end
    end
    
    % Matriz de Inércia da elipse equivalente
    Je = 4/m00(1,i) * [u20, u11; u11, u02];
    
    % Plot da elipse
    %plot_ellipse(Je, [uc(1,i), vc(1,i)], 'g');
    
    % Raios da elipse a partir dos autovalores/vetores
    [V,D] = eig(Je); % V -> autovetor (direção vetor != raio)
    % D -> auto valores (rel. com valor dos raios)

    lb1 = D(1,1);
    lb2 = D(2,2);
    
    a(1,i) = sqrt(lb1); % raios da elipse
    b(1,i) = sqrt(lb2);
    % razão de raios
    r(1,i) = a(1,i)/b(1,i);
    
    % cálculo do ângulo da elipse
    v2 = V(:,2); % componetes horizonal e vertical respec.
    theta(1,i) = atand(v2(2)/v2(1));
    
    % pixel de borda
    %IE = edge(I2);
    
    aux = 1;
    clear ue; clear ve;
    for u = umin:umax
        for v = vmin:vmax
            if(I2(v,u) == 1)
               inicialP = [v,u];
               boundaryLim = bwtraceboundary(I2, inicialP, 'N', 8);
            end
        end
        break;
    end 
    
    if(in == 0)
        % Daria pra usar a função Interpolate para limitar em e.g.
        % 400 pontos  
        %
        if(edge == 1)
            hold on;
            plot(boundaryLim(:,2), boundaryLim(:,1), 'r.');
        end
        
        % perimetro
        p = sqrt((boundaryLim(end-1,1)-boundaryLim(1,1))^2+...
            (boundaryLim(end-1,2)-boundaryLim(1,2))^2);

        d = 0;
        aux = size(boundaryLim(:,1));
        for k=1:aux-1
            d = d + sqrt((boundaryLim(k,1)-boundaryLim(k+1,1))^2+...
                (boundaryLim(k,2)-boundaryLim(k+1,2))^2);
        end
        P = d;

        % Circularidade
        rho = (4*pi*m00(i))/(P)^2;

        % Curva de Distancia e Angulo
        aux1 = aux(1);
        DC = zeros(1,aux1);
        theta = zeros(1,aux1);
        for k=1:aux1
            DC(k) = sqrt((boundaryLim(k,1)-vc(1,i))^2+...
                (boundaryLim(k,2)-uc(1,i))^2);

            theta(k) = atan2((boundaryLim(k,1)-vc(1,i)),...
                (boundaryLim(k,2)-uc(1,i)));
        end

        % Definição de Structures
        info{i} = struct('bb', [vmin, umin, vmax, umax],...
        'area', m00(1,i), 'centroide', [vc(1,i), uc(1,i)],...
        'm_inercia', Je, 'razao', r(1,i), 'angulo', theta(1,i),...
        'edge', boundaryLim, 'perimetro', P, 'circularidade', rho,...
        'curva_dist', DC, 'curva_angulo', theta);
    end

    if(in == 1)
        % Daria pra usar a função Interpolate para limitar em e.g.
        % 400 pontos
        %num = 400;
        x = 1:numel(boundaryLim(:,2));
        xi = linspace(1,numel(boundaryLim(:,2)),num);

        vLim = interp1(x,boundaryLim(:,1),xi,'linear','extrap');
        uLim = interp1(x,boundaryLim(:,2),xi,'linear','extrap');

        %
        if(edge == 1)
            hold on;
            plot(uLim, vLim, 'r.');
        end

        % perimetro
        per = sqrt((vLim(end-1)-vLim(1))^2+...
                   (uLim(end-1)-uLim(1))^2);

        d = 0;
        aux = num;
        for k=1:aux-1
            d = d + sqrt((vLim(k)-vLim(k+1))^2+(uLim(k)-uLim(k+1))^2);
        end
        P = d+per;

        % Circularidade
        rho = (4*pi*m00(i))/(P)^2;

        % Curva de Distancia e Angulo
        aux1 = num;
        DC = zeros(1,aux1);
        phi = zeros(1,aux1);
        for k=1:aux1
            DC(k) = sqrt((vLim(k)-vc(1,i))^2+...
                (uLim(k)-uc(1,i))^2);

            phi(k) = atan2((vLim(k)-vc(1,i)),...
                (uLim(k)-uc(1,i)));
        end

        % Definição de Structures
        info{i} = struct('bb', [vmin, umin, vmax, umax],...
        'area', m00(1,i), 'centroide', [vc(1,i), uc(1,i)],...
        'm_inercia', Je, 'razao', r(1,i), 'angulo', theta(1,i),...
        'edge', boundaryLim, 'perimetro', P, 'circularidade', rho,...
        'curva_dist', DC, 'curva_angulo', phi); 
    end

end