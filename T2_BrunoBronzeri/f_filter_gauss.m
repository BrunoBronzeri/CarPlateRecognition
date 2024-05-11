function O = f_filter_gauss(I, sig, X)

% Suavização
w = 6*sig-1;
K = fspecial('gaussian', w, sig);
Is = filter2(K, I, 'same');

% mascara
Ig = I - Is;

% Img resultante
O = I + X*Ig;

% saturação 
O(O > 1) = 1;
O(O < 0) = 0;

end