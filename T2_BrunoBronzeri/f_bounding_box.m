function info = f_bounding_box(I)

% analise de componentes conectados
[Ilabel, N] = bwlabel(I);

figure; imshow(I); axis on

info = cell(N,1);

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
    

    % Definição de Structures
    info{i} = struct('bb', [vmin, umin, vmax, umax]);
end
end