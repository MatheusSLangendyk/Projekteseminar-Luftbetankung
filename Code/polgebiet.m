function polgebiet(region, A, a1, b1, R)      
%Plottet das Polgebiet unter den gegebenen
%Parametern
%a1 und b1 sind die Parameter des rechten Randes
%a2, b2 und R sind die Parameter des linken Randes. R stellt dabei eine
%Verschiebung entlang der d-Achse dar
%A ist die Systemmatrix
%nargin == 2 entspricht der Eingabe: polgebiet(region, A)
%nargin == 6 entspricht der Eingabe: polgebiet(a1, b1, a2, b2, R, A)
if nargin == 2
    a1 = region.a1; a2 = region.a2; b1 = region.b1; b2 = region.b2; R = region.R; A = A;
elseif nargin == 6
    A1 = region; B1 = A; A2 = a1; B2 = b1; r = a2; a = b2;
    a1 = A1; b1 = B1; a2 = A2; b2 = B2; R = r; A = a;
end

%Eigenwerte berechnen
[~, ew] = eig(A);

%EW in Imaginär- und Realteil aufteilen
re = diag(real(ew));
im = diag(imag(ew));

%Achsenskalierungen berechnen
%y-Achse
ymax = R + a1/b1 * sqrt(R^2 + b1^2);
ymin = -ymax;

%x-Achse
if R > max(re)
    xmin = -R - R/4;
    xmax = R/4;
else 
    xmin = -(max(re) + max(re)/4);
    xmax = max(re)/4;
end

%Kurven plotten
ezplot(@(d, w)kurve1(a1, b1, d, w), [xmin xmax ymin ymax]);
hold on;
ezplot(@(d, w)kurve2(a2, b2, R, d, w), [xmin xmax ymin ymax]);

%Achsenbegrenzungen setzen
axis([xmin xmax ymin ymax]);

hold off;
end
