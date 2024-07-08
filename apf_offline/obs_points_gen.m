function obs_points = obs_points_gen(obstacle)

% obstacle=[4.5 5.5 0.3 0 10];
% xcentro ycentro r zcentro h

% Parametri dell'ostacolo (cilindro)
center_x = obstacle(1);             % Coordinata x del centro
center_y = obstacle(2);             % Coordinata y del centro
center_z = 0;                       % Coordinata z del centro
radius = obstacle(3);               % Raggio della circonferenza
height = obstacle(5);               % Altezza del cilindro
num_points_circumference = 10;      % Numero di punti da generare lungo la circonferenza
num_points_height = 50;             % Numero di punti da generare lungo l'altezza

% Genera angoli da 0 a 2*pi per la circonferenza
theta = linspace(0, 2*pi, num_points_circumference);

% Genera altezze da center_z a center_z + height per il cilindro
z_values = linspace(center_z, center_z + height, num_points_height);

% Inizializza i vettori per i punti 3D con la dimensione corretta
x_points = zeros(1, num_points_circumference * num_points_height);
y_points = zeros(1, num_points_circumference * num_points_height);
z_points = zeros(1, num_points_circumference * num_points_height);

% Indice per l'assegnazione dei punti
index = 1;

% Calcola le coordinate dei punti esterni lungo la superficie del cilindro
for z = z_values
    x_temp = center_x + radius * cos(theta);
    y_temp = center_y + radius * sin(theta);
    z_temp = z * ones(1, num_points_circumference);
    
    % Assegna i punti ai vettori principali
    x_points(index:index+num_points_circumference-1) = x_temp;
    y_points(index:index+num_points_circumference-1) = y_temp;
    z_points(index:index+num_points_circumference-1) = z_temp;
    
    % Aggiorna l'indice
    index = index + num_points_circumference;

end

% Definisci le posizioni degli ostacoli come una matrice 3xn
obs_points = [x_points' y_points' -z_points'];

end