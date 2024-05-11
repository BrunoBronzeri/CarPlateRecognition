% Bruno Bueno Bronzeri - 20204055;
% Universidade Federal de Santa Catarina (12/2023);
% Visão Computacional Para Robótica;
% Prof.: Dr. Marcos Vinícius Matsuo;
% ALGORITMO DE RECONHECIMENTO DE CARACTERES EM PLACAS DE AUTOMÓVEIS
% PADRÃO MERCOSUL.

clear; close all; clc;
tic
I = imread('./imagens_placas/n3_placa6.jpg');
%n2_placa1 --> ordem de letra e número alterada

%figure; imshow(I);

P = imread('./imagens_placas/fonte_mercosul.png');
P = rgb2gray(P);
P = ~P;
Px = P;
P(120:size(P,1), :) = 0;
P1 = P;
Px(1:120, :) = 0;
P2 = Px;

P1 = imdilate(P1, strel('disk', 1));
P1 = imerode(P1, strel('square', 1));
P1 = imfill(P1, 'holes');

% Laço para Dilatação Número '0' e reinserção na imagem
P2 = imdilate(P2, strel('disk', 1));
Paux = imdilate(P2(130:180, 10:42), strel('square', 3));
Psize = zeros(size(P,1), size(P,2));

conty = 0; 
contx = 0;
for i = 130:180
    conty = conty+1;
    for j = 10:42
        contx = contx+1;
        Psize(i,j) = Paux(conty, contx);
    end
    contx=0;
end
P2 = P2 | Psize;

% Laço para Dilatação Letra 'M' e reinserção na imagem
Paux = imdilate(P1(10:60, 405:440), strel('square', 5));
Paux = imerode(Paux, strel('disk', 5));
Psize = zeros(size(P,1), size(P,2));

conty = 0;
contx = 0;
for i = 10:60
    conty = conty+1;
    for j = 405:440
        contx = contx+1;
        Psize(i,j) = Paux(conty, contx);
    end
    contx=0;
end
P1 = P1 | Psize;

% Laço para Dilatação Letra 'R' e reinserção na imagem
Paux = imdilate(P1(70:120, 142:175), strel('square', 2));
%Paux = imerode(Paux, strel('disk', 5));
Psize = zeros(size(P,1), size(P,2));

conty = 0;
contx = 0;
for i = 70:120
    conty = conty+1;
    for j = 142:175
        contx = contx+1;
        Psize(i,j) = Paux(conty, contx);
    end
    contx=0;
end
P1 = P1 | Psize;

% Laço para Dilatação Letra 'N' e reinserção na imagem
Paux = imdilate(P1(72:117, 11:42), strel('square', 2));
%Paux = imerode(Paux, strel('disk', 5));
Psize = zeros(size(P,1), size(P,2));

conty = 0;
contx = 0;
for i = 72:117
    conty = conty+1;
    for j = 11:42
        contx = contx+1;
        Psize(i,j) = Paux(conty, contx);
    end
    contx=0;
end
P1 = P1 | Psize;


P21 = imfill(P1, 'holes');
%%%%

R1 = rgb2gray(I);

M = size(I,1);
N = size(I,2);

coord = zeros(M,N);

for u = 1:M
    for v = 1:N
        if ((I(u,v,1)+I(u,v,2)) < (1.1*I(u,v,3))) &&...
                (I(u,v,3)>80)
            coord(u,v) = 1;
        elseif (I(u,v,1)>100) && (I(u,v,3)>120) &&...
                (I(u,v,1)<130)
            coord(u,v) = 1;
        end
    end
end

% Definicao do elemento estruturante (abertura)
S = strel('disk', 1);
H1 = imerode(coord, S);
S = strel('disk',2);
H2 = imdilate(H1, S);

I2 = f_bb_analisa_regioes(H2, 1 , 400, 1); %f_bb_analisa

lgth = size(I2);
bgst = 0;

for len = 1:lgth
    aux = I2{len,1}.area;
    if(aux > bgst)
        bgst = aux;
        index = len;
    end
end

H3 = zeros(M,N);

for u = I2{index,1}.bb(1):I2{index,1}.bb(3)
    for v = I2{index,1}.bb(2):I2{index,1}.bb(4)
        H3(u,v) = H2(u,v); 
    end
end

if(M<170)
    S2 = strel('disk', 4);
    S4 = strel('disk', 4);
else
    S1 = strel('disk', 8);
    S3 = strel('disk',8);
end
I3 = imdilate(H3, S);
I4 = imerode(I3, S);
%figure; imshow(I4);

I5 = imfill(I4, 'holes');

H2 = edge(I5, "Canny");

S = strel('disk', 4);
H2 = imdilate(H2, S);

% Transformada Hough
[H, theta, rho] = hough(H2);

% figure; imagesc(theta, rho, H);
% xlabel('\theta'); ylabel('\rho'); axis on;
% colorbar;
% 
% figure; surf(theta, rho, H);
% shading interp;
% xlabel('\theta'); ylabel('\rho'); axis on;

% Idetentifica de picos
numpeaks = 4;
peaks = houghpeaks(H, numpeaks);
% hold on; plot(theta(peaks(:,2)), rho(peaks(:,1)), 's',...
%     'color', 'white');

% Determina linhas
lines = houghlines(H2, theta, rho, peaks);

figure; imshow(H2);
for k = 1:length(lines)
    xy = [lines(k).point1; lines(k).point2];
    hold on; plot(xy(:,1), xy(:,2), 'LineWidth', 2,...
        'Color', 'green');
    
    % plot beginnings and ends os lines
    plot(xy(1,1), xy(1,2), 'x', 'LineWidth', 2,...
        'Color', 'yellow');
    plot(xy(2,1), xy(2,2), 'x', 'LineWidth', 2,...
        'Color', 'red');
end

% Encontrar os dois menores [*u*,v] e os dois maiores [*u*,v]
K = size(lines,2);
lower_u = lines(1).point1;
low_u = [10^4 10^4];
bigger_u = ones(1,2);
biggest_u = ones(1,2);

% Encontrar menor e maior valor
for x = 1:K
    if(lines(x).point1(1) < lower_u(1))
        lower_u = lines(x).point1;
    elseif(lines(x).point1(1) == lower_u(1))
        if(lines(x).point1(2) < lower_u(2))
            lower_u = lines(x).point1;
        end
    end    
    if(lines(x).point2(1) > biggest_u(1))
        biggest_u = lines(x).point2;
    elseif(lines(x).point2(1) == biggest_u(1))
        if(lines(x).point2(2) > biggest_u(2))
            biggest_u = lines(x).point2;
        end
    end
end

% Encontrar os segundos menor e maior valor
for x = 1:K
    if(lines(x).point1(1) > lower_u(1) &&...
            lines(x).point1(1) < low_u(1) &&...
            (lines(x).point1(2) < lower_u(2)-10 ||...
            lines(x).point1(2) > lower_u(2)+10))
        low_u = lines(x).point1;
    elseif(lines(x).point1(1) == lower_u(1))
        if(lines(x).point1(2) > lower_u(2))
            low_u = lines(x).point1;
        end
    end
    
    if(lines(x).point2(1) < biggest_u(1) &&...
            lines(x).point2(1) > bigger_u(1) &&...
            (lines(x).point2(2) > biggest_u(2)+10 ||...
            lines(x).point2(2) < biggest_u(2)-10))
        bigger_u = lines(x).point2;
    elseif(lines(x).point2(1) == biggest_u(1))
        if(lines(x).point2(2) < biggest_u(2)-13 ||...
                lines(x).point2(2) > biggest_u(2)+13)
            bigger_u = lines(x).point2;
        end
    end
end

% Definição ordenada dos quatro pontos de quina
if(lower_u(2) < low_u(2) && bigger_u(2) < biggest_u(2))
    pi1 = lower_u; pf1 = bigger_u;
    pi2 = low_u; pf2 = biggest_u;
    
elseif(low_u(2) < lower_u(2) && bigger_u(2) < biggest_u(2))
    pi1 = low_u; pf1 = bigger_u;
    pi2 = lower_u; pf2 = biggest_u;
    
elseif(lower_u(2) == bigger_u(2))
    pi1 = low_u; pf1 = biggest_u;
    pi2 = lower_u; pf2 = bigger_u;
    
elseif(lower_u(2) == biggest_u(2))
    pi1 = lower_u; pf1 = biggest_u;
    pi2 = low_u; pf2 = bigger_u;
    
elseif(low_u(2) < lower_u(2) && biggest_u(2) < bigger_u(2))
    pi1 = low_u; pf1 = biggest_u;
    pi2 = lower_u; pf2 = bigger_u;
    
elseif(lower_u(2) < low_u(2) && biggest_u(2) < bigger_u(2))
    pi1 = lower_u; pf1 = biggest_u;
    pi2 = low_u; pf2 = bigger_u;
    
end

% Caso placa em pé, limite em v para determinar e ajeitar
% rangeSup = low_u(2) + 4;
% rangeInf = low_u(2) - 4;
% 
% if(lower_u(2) >= rangeInf && lower_u(2) <= rangeSup)
%     pi1 = lines(2).point2; pf1 = lower_u;
%     pi2 = biggest_u; pf2 = low_u;
% end

% pi1 = [106 462]; pf1 = [254 55];
% pi2 = [147 465]; pf2 = [296 55];

% Até aqui --> trnasformada Hough e ordenação de quatro pontos

% Cálculo da distância Euclidiana
D = sqrt((pi2(1)-pf2(1))^2+(pi2(2)-pf2(2))^2);

% obtenção de informação e decalração de correções
angle = I2{index}.angulo;
corr_perspec = 0;
corr_profundR = 0;
corr_profundL = 0;

% Condições para inclinações diferentes (angulos) e obtendo a diferença
% das alturas "v's" para saber como corrigir a perspectiva
%
% A detemrinação do coeficiente h é dada:
% D/h = 4.76 -> h = D/4.76 (proporção das dimensões da placa)
if(angle > -5 && angle < 10 && (pi2(2)-pf2(2) < 80))
    if(size(I,1)<320)
        h = D/4.76; %5.3
        corr_perspec = 1.6; %1.8
    else
        h = D/4.76;
    end
elseif(angle > -7 && angle < 10 && (pi2(2)-pf2(2) < 150))
    h = D/4;
    corr_profundL = pi2(2)-pf2(2);
elseif(angle > -20 && angle < 1 && (pi2(2)-pf2(2) < 30))
    h = D/2.7; %2.76
    if(pi2(2)-pf2(2) > 0)
        corr_profundR = pi2(2)-pf2(2);
    else
        corr_profundL = pf2(2)-pi2(2);
    end
elseif(angle > -30 && angle < 30 && (pi2(2)-pf2(2) < 80))
    if(size(I,1)<170)
        h = D/3;
    elseif(size(I,1)<320)
        h = D/3.4;
    else
        h = D/3.5;
        corr_perspec = 0.2;
        corr_profundR = 5;
    end
elseif(angle > -60 && angle < 60 && (pi2(2)-pf2(2) > 200))
    h = D/2.76;
    corr_perspec = 2;
elseif(angle > -89 && angle < 89 && (pi2(2)-pf2(2) > 100))
    h = D/5;
    corr_perspec = 4;
    corr_profundL = -90;
    corr_profundR = -70;
else
    h = D/2.76;
end

% Aplicação das correções e coef. 'h'
adjust = fix(corr_perspec*0.05*N);
adjust_v = h;

xi = [pi2(1), pi2(1)+adjust];
yi = [pi2(2), pi2(2)+adjust_v+corr_profundL];

xf = [pf2(1), pf2(1)+adjust];
yf = [pf2(2), pf2(2)+adjust_v+corr_profundR];

xc = [pi2(1)+adjust, pf2(1)+adjust];
yc = [pi2(2)+adjust_v + corr_profundL, pf2(2)+adjust_v+corr_profundR];

% plot das linhas que demarcam região com informações
figure; imshow(R1);
hold on
plot(xi,yi,'color', 'r');
plot(xf,yf,'color', 'r');
plot(xc,yc,'color', 'r');
plot([pi2(1), pf2(1)],[pi2(2),pf2(2)],'color', 'r');

vertices = [xi(1), yi(1);
            xi(2), yi(2);
            xc(2), yc(2);
            pf2(1), pf2(2)];
        

img = zeros(M,N);
% Criação uma máscara da área com informação
mascara = poly2mask(vertices(:, 1), vertices(:, 2),...
    size(img, 1), size(img, 2));

% Preencher a imagem com preto fora da área desejada
img(~mascara) = 1;
img = ~img;

%figure; imshow(img);
img = double(img);

R1 = im2double(R1);
R2 = img.*R1;

figure; imshow(R2);

% Matriz de homografia planar
% Coordenadas
u1 = pi1(1); u2 = pi2(1); u3 = pf2(1); u4 = pf1(1);
v1 = pi1(2); v2 = pi2(2); v3 = pf2(2); v4 = pf1(2);
uc1 = pi1(1); uc2 = pi1(1); uc3 = pi1(1)+400; uc4 = pi1(1)+400;
vc1 = pi1(2); vc2 = pi1(2)+26; vc3 = pi1(2)+26; vc4 = pi1(2);
 
% Matriz de homografia (projeção)
A = [u1 v1 1 0 0 0 -uc1*u1 -uc1*v1 ;
     0 0 0 u1 v1 1 -vc1*u1 -vc1*v1 ;
     u2 v2 1 0 0 0 -uc2*u2 -uc2*v2 ;
     0 0 0 u2 v2 1 -vc2*u2 -vc2*v2 ;
     u3 v3 1 0 0 0 -uc3*u3 -uc3*v3 ;
     0 0 0 u3 v3 1 -vc3*u3 -vc3*v3 ;
     u4 v4 1 0 0 0 -uc4*u4 -uc4*v4 ;
     0 0 0 u4 v4 1 -vc4*u4 -vc4*v4 ];
 
b = [uc1; vc1; uc2; vc2; uc3; vc3; uc4; vc4];
h = inv(A)*b;
 
% Matriz de homografia com coeficientes definidos
H = [h(1) h(2) h(3);
     h(4) h(5) h(6);
     h(7) h(8)  1];
 
tform = projective2d(H.'); 
T = imwarp(R2, tform);
 
if(size(I,1)<170)
    T = imadjust(T);
    T = f_filter_gauss(T, 0.5, 1);
    border = 'Sobel';
else
    border = 'Canny';
end

% figure; imshow(T);
% axis on

% Obtenção de bordas para selecionar região desejada
I6 = edge(T, border);

% Dilatação, preenchimento e erosão
if(size(I,1)<320)
    R3 = imdilate(I6, strel('square', 3));
    R3 = imerode(R3, strel('square',2));
else
    R3 = imdilate(I6, strel('square', 3));
    R3 = imerode(R3, strel('square', 2));
end

%figure; imshow(R3); 
Rinfo = f_bounding_box(R3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Laço para extrair maior altura e largura
XXL = [0 0];
for i = 1:size(Rinfo,1)
    height = Rinfo{i}.bb(3)-Rinfo{i}.bb(1);
    compr = Rinfo{i}.bb(4)-Rinfo{i}.bb(2);
    
    if((height > XXL(1)) && (compr > 0.1*height) && (compr < height*1.1))
        XXL = [height, compr];
    end
end
% Laco para extrair segunda maior altura
XL = [0 0];
for i = 1:size(Rinfo,1)
    height = Rinfo{i}.bb(3)-Rinfo{i}.bb(1);
    compr = Rinfo{i}.bb(4)-Rinfo{i}.bb(2);
    
    if(height < XXL(1) && height > XL(1) && compr > 0.1*height &&...
            compr < height*1.1)
        XL = [height, compr];
    end
end


if(XXL(1) > XL(1)+10 && XXL(2) > XL(2)+10)
    XXL = XL;
end

bb = zeros(7,4);
cont = 1;

for i = 1:size(Rinfo,1)
    
    height = Rinfo{i}.bb(3)-Rinfo{i}.bb(1);
    compr = Rinfo{i}.bb(4)-Rinfo{i}.bb(2);
    
    if(cont < 8)
        if(size(I,1)< 320)
            if(Rinfo{i}.bb(3)-Rinfo{i}.bb(1) > (XXL(1)-10) &&...
                    Rinfo{i}.bb(3)-Rinfo{i}.bb(1) < (XXL(1)+10) &&...
                    (compr > 0.1*XXL(1)) && (compr < height*1.1))
                bb(cont,:) = Rinfo{i}.bb;
                cont = cont+1;
            end
        else
            if(Rinfo{i}.bb(3)-Rinfo{i}.bb(1) > (XXL(1)-10) &&...
                    Rinfo{i}.bb(3)-Rinfo{i}.bb(1) < (XXL(1)+10) &&...
                    (compr > 0.18*XXL(1)) && (compr < height*1.1))
                bb(cont,:) = Rinfo{i}.bb;
                cont = cont+1;
            end
        end
    end
end

bb = bb + [-3 -3 4 4;
    -2 -2 4 4;
    -3 -3 4 4;
    -3 -3 4 4;
    -2 -2 4 4;
    -2 -2 4 4;
    -3 -3 4 4];

figure; imshow(T);
axis on
for z = 1:cont-1
    hold on, plot([ bb(z,2) , bb(z,2) ],...
        [ bb(z,1) , bb(z,3) ], 'r', 'LineWidth', 2);
    hold on, plot([ bb(z,2) , bb(z,4) ],...
        [ bb(z,1), bb(z,1) ], 'r', 'LineWidth', 2);
    hold on, plot([ bb(z,4) , bb(z,4) ],...
        [ bb(z,1), bb(z,3) ], 'r', 'LineWidth', 2);
    hold on, plot([ bb(z,2) , bb(z,4) ],...
        [ bb(z,3), bb(z,3) ], 'r', 'LineWidth', 2); 
end

% plot da imagem da placa em escala de cinza (caracteres apenas)
bbu = bb(:,[1,3]);
bbv = bb(:,[2,4]);

min_u = min(bbu(:,1));
min_v = min(bbv(:,1));
max_u = max(bbu(:,2));
max_v = max(bbv(:,2));

Z = zeros(size(T,1),size(T,2));
for i = min_u:max_u
    for j = min_v:max_v
        Z(i,j) = T(i,j);
    end
end

figure; imshow(Z);
axis on

%%

% Definição de imagens individuais para os caracteres
C1 = zeros(bb(1,4)-bb(1,2), bb(1,3)-bb(1,1));
C2 = zeros(bb(2,4)-bb(2,2), bb(2,3)-bb(2,1));
C3 = zeros(bb(3,4)-bb(3,2), bb(3,3)-bb(3,1));
C4 = zeros(bb(4,4)-bb(4,2), bb(4,3)-bb(4,1));
C5 = zeros(bb(5,4)-bb(5,2), bb(5,3)-bb(5,1));
C6 = zeros(bb(6,4)-bb(6,2), bb(6,3)-bb(6,1));
C7 = zeros(bb(7,4)-bb(7,2), bb(7,3)-bb(7,1));
vv = 0;

for i = bb(1,1):bb(1,3)
    uu = 0; vv = vv+1;
    for j = bb(1,2):bb(1,4)
        uu = uu+1;
        C1(uu,vv) = Z(i,j);
    end
end 
vv = 0;
for i = bb(2,1):bb(2,3)
    uu = 0; vv = vv+1;
    for j = bb(2,2):bb(2,4)
        uu = uu+1;
        C2(uu,vv) = Z(i,j);
    end
end
vv = 0;
for i = bb(3,1):bb(3,3)
    uu = 0; vv = vv+1;
    for j = bb(3,2):bb(3,4)
        uu = uu+1;
        C3(uu,vv) = Z(i,j);
    end
end
vv = 0;
for i = bb(4,1):bb(4,3)
    uu = 0; vv = vv+1;
    for j = bb(4,2):bb(4,4)
        uu = uu+1;
        C4(uu,vv) = Z(i,j);
    end
end
vv = 0;
for i = bb(5,1):bb(5,3)
    uu = 0; vv = vv+1;
    for j = bb(5,2):bb(5,4)
        uu = uu+1;
        C5(uu,vv) = Z(i,j);
    end
end
vv = 0;
for i = bb(6,1):bb(6,3)
    uu = 0; vv = vv+1;
    for j = bb(6,2):bb(6,4)
        uu = uu+1;
        C6(uu,vv) = Z(i,j);
    end
end
vv = 0;
for i = bb(7,1):bb(7,3)
    uu = 0; vv = vv+1;
    for j = bb(7,2):bb(7,4)
        uu = uu+1;
        C7(uu,vv) = Z(i,j);
    end
end

bb_c = zeros(7,4);

% Tratamento individual dos caracteres
C1 = edge(C1, 'Canny');
C1 = imdilate(C1, strel('disk', 2)); 
C1 = imerode(C1, strel('sphere', 2)); 
if(size(I,1) < 170)
    C1 = imdilate(C1, strel('square', 3));
end
C1 = imclearborder(C1);
if(size(I,1)<320)
    C1 = imdilate(C1, strel('square', 2));
else
    C1 = imdilate(C1, strel('disk',2));
end
C1 = imfill(C1, 'holes');
C1info = f_bb_regioes(C1, 1, 400, 0);

C2 = edge(C2, 'Canny');
C2 = imdilate(C2, strel('square', 3));
C2 = imerode(C2, strel('sphere', 1));
C2 = imclearborder(C2);
C2 = imfill(C2, 'holes');

C3 = edge(C3, 'Canny');
C3 = imdilate(C3, strel('disk', 2)); %2
C3 = imclearborder(C3);
C3 = imfill(C3, 'holes');
C3info = f_bb_regioes(C3, 1, 400, 0);
if(size(I,1) < 170)
    C3 = imerode(C3, strel('disk', 6));
elseif(C3info{1}.area > 2000 && size(I,1) > 310)
    C3 = imerode(C3, strel('disk', 4));
elseif(C3info{1}.area > 1900 && size(I,1) > 310)
    C3 = imerode(C3, strel('square', 5));
elseif(C3info{1}.area > 1800 && size(I,1) > 300) %300
    C3 = imerode(C3, strel('square', 1));
elseif(C3info{1}.area > 1800 && size(I,1) < 301)
    C3 = imerode(C3, strel('square', 3));
else
    C3 = imerode(C3, strel('sphere', 3)); %3
end

C4 = edge(C4, 'Canny');
C4 = imdilate(C4, strel('disk', 1));
C4 = imerode(C4, strel('square', 1));
C4 = imclearborder(C4);
C4 = imfill(C4, 'holes');
C4info = f_bb_regioes(C4, 1, 400, 0);
if(C4info{1}.area < 1000)
    C4 = imerode(C4, strel('square', 1));
end
C4 = imerode(C4, strel('sphere', 3));

C5 = edge(C5, 'Canny');
C5 = imdilate(C5, strel('square', 3));
C5 = imclearborder(C5);
C5 = imfill(C5, 'holes');
C5 = imerode(C5, strel('disk', 3));

C6 = edge(C6, 'Canny');
C6 = imdilate(C6, strel('square', 3));
C6 = imclearborder(C6);
C6 = imfill(C6, 'holes');
C6 = imerode(C6, strel('disk', 3));

C7 = edge(C7, 'Canny');
C7 = imdilate(C7, strel('disk', 1)); %'disk', 2
C7 = imclearborder(C7);
if(size(I,1)>700)
    C7 = imerode(C7, strel('disk', 1)); 
    C7 = imdilate(C7, strel('disk', 2));
end
C7 = imfill(C7, 'holes');
if(size(I,1) < 170)
    C7 = imerode(C7, strel('square', 3)); %2
    C7 = imdilate(C7, strel('square', 5));
    C7 = imerode(C7, strel('square', 1));
end
if(size(I,1) > 1400)
    C7 = imerode(C7, strel('square', 3));
end
C7info = f_bb_regioes(C7, 1, 400, 0);


figure;

subplot(2,4,1);
imshow(C1.');

subplot(2,4,2);
imshow(C2.');

subplot(2,4,3);
imshow(C3.');

subplot(2,4,4);
imshow(C4.');

subplot(2,4,5);
imshow(C5.');

subplot(2,4,6);
imshow(C6.');

subplot(2,4,7);
imshow(C7.');

% Definição de uma imagem preta para juntar todos os caracteres operados
maiorH = [size(C1,2), size(C2,2), size(C3,2), size(C4,2), size(C5,2),...
    size(C6,2), size(C7,2)];
comprL = size(C1,1) + size(C2,1) + size(C3,1) + size(C4,1) +...
    size(C5,1) + size(C6,1) + size(C7,1);
maiorH = max(maiorH);

imC = zeros(maiorH, comprL);

Sc1 = size(C1); Sc2 = size(C2); Sc3 = size(C3); Sc4 = size(C4);
Sc5 = size(C5); Sc6 = size(C6); Sc7 = size(C7);

% Adicionando cada caracter individual ordenado um ao lado do outro para
%análise única
imC(1:Sc1(2), 1:Sc1(1)) = C1.';
imC(1:Sc2(2), Sc1(1)+1:Sc1(1)+Sc2(1)) = C2.';
imC(1:Sc3(2), Sc1(1)+Sc2(1)+1:Sc1(1)+Sc2(1)+Sc3(1)) = C3.';
imC(1:Sc4(2), Sc1(1)+Sc2(1)+Sc3(1)+1:Sc1(1)+Sc2(1)+Sc3(1)+Sc4(1)) = C4.';
imC(1:Sc5(2), Sc1(1)+Sc2(1)+Sc3(1)+Sc4(1)+1:Sc1(1)+Sc2(1)+Sc3(1)+...
    Sc4(1)+Sc5(1)) = C5.';
imC(1:Sc6(2), Sc1(1)+Sc2(1)+Sc3(1)+Sc4(1)+Sc5(1)+1:Sc1(1)+Sc2(1)+...
    Sc3(1)+Sc4(1)+Sc5(1)+Sc6(1)) = C6.';
imC(1:Sc7(2), Sc1(1)+Sc2(1)+Sc3(1)+Sc4(1)+Sc5(1)+Sc6(1)+1:Sc1(1)+...
    Sc2(1)+ Sc3(1)+Sc4(1)+Sc5(1)+Sc6(1)+Sc7(1)) = C7.';

% Somente caracteres de interesse operados e prontos para comparação

% Obtenção das infos dos caracteres analisados e os carac. fonte
Pi = 400;
info_imC = f_bb_analisa_regioes(imC, 1, Pi, 1);
info_P = f_bb_analisa_regioes(P1, 1, Pi, 1);
info_Px = f_bb_analisa_regioes(P2, 1, Pi, 1);

%%
correl = zeros(1,Pi);
reconC = zeros(1,7);
hst_corr = zeros(4,2);

% Comparação das curvas de distância do caracter com o banco de dados
for i = 1:7
    tempCD = info_imC{i}.curva_dist;
    tempMean = tempCD - mean(tempCD);
    tempEnergy = tempMean/sqrt(sum(tempMean.^2));
    
    cont = 0;
    if(i==4 || i==6 || i==7)
        for j = 1:10
        fontCD = info_Px{j}.curva_dist;
        fontMean = fontCD - mean(fontCD);
        fontEnergy = fontMean/sqrt(sum(fontMean.^2));
        
            for k = 1:Pi
                fontEnergy  = circshift(fontEnergy, 1);
                correl(k) = sum(tempEnergy.*fontEnergy);
                                
            end
            if(max(correl) > 0.8)
                cont = cont+1;
                hst_corr(cont,:) = [max(correl), j];
            end
        end
    else
        for j = 1:26
        fontCD = info_P{j}.curva_dist;
        fontMean = fontCD - mean(fontCD);
        fontEnergy = fontMean/sqrt(sum(fontMean.^2));
        
            for k = 1:Pi
                fontEnergy  = circshift(fontEnergy, 1);
                correl(k) = sum(tempEnergy.*fontEnergy);
                
            end
            if(max(correl) > 0.85)
                cont = cont+1;
                hst_corr(cont,:) = [max(correl), j];
            end
        end
    end
    
    hst_corr
    top_value = 0;
    hst_ant = 0;
    for q = 1:size(hst_corr, 1)
        if(hst_corr(q) > hst_ant)
            top_value = hst_corr(q+size(hst_corr,1));
            hst_ant = hst_corr(q);
        end
    end
    
    reconC(i) = top_value;
    hst_corr(:,:) = 0;
    top_value(:,:) = 0;
    
end

%%
charac = zeros(1,7);

for i = 1:7
    if(i==4 || i==6 || i==7)
       switch reconC(i)
        case 1
            str = '0';
        case 2 
            str = '1';
        case 3
            str = '2';
        case 4 
            str = '3';
        case 5
            str = '4';
        case 6 
            str = '5';
        case 7
            str = '6';
        case 8 
            str = '7';
        case 9
            str = '8';
        case 10 
            str = '9';
       end   
    else
       switch reconC(i)
        case 1
            str = 'A';
        case 2 
            str = 'N';
        case 3
            str = 'B';
        case 4 
            str = 'O';
        case 5
            str = 'C';
        case 6 
            str = 'P';
        case 7
            str = 'D';
        case 8 
            str = 'Q';
        case 9
            str = 'R';
        case 10 
            str = 'E';
        case 11
            str = 'F';
        case 12 
            str = 'S';
        case 13
            str = 'G';
        case 14 
            str = 'T';
        case 15
            str = 'H';
        case 16
            str = 'U';
        case 17
            str = 'V';
        case 18
            str = 'I';
        case 19
            str = 'J';
        case 20 
            str = 'W';
        case 21
            str = 'K';
        case 22 
            str = 'X';
        case 23
            str = 'L';
        case 24 
            str = 'Y';
        case 25
            str = 'M';
        case 26 
            str = 'Z';
        end 
    end
    
    charac(i) = str;
    
end

charac = char(charac)

%%
close all;

Ip = imread('./imagens_placas/modelo.png');

figure; imshow(Ip);
axis on;

aloc_u = 480;
aloc_v = 100;

text(aloc_v, aloc_u, charac, 'FontSize', 67, 'FontName',...
    'ZapfDingbats');
toc
