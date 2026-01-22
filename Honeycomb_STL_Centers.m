%% Generador de Trayectoria V8: Corrección con K-Means
% Objetivo: Encontrar centros usando Clustering Geométrico en lugar de Conectividad.

% --- 1. CONFIGURACIÓN ---
target_hexagons = 7; % <--- DATO CLAVE: Forzamos la búsqueda de 7 centros
stl_filename = 'honeycomb.stl'; 

% --- 2. CARGA Y ESCALA ---
try
    TR_raw = stlread(stl_filename);
catch
    error('No se encuentra el archivo STL.');
end

points = TR_raw.Points;
connectivity = TR_raw.ConnectivityList;

% Escala automática (m a mm si es necesario)
if max(points(:)) < 5.0
    disp('--> Escala: Metros detectados. Convirtiendo a mm...');
    points = points * 1000;
end

% --- 3. SLICING: FILTRADO DE LA CAPA SUPERIOR ---
z_vals = points(:,3);
max_z = max(z_vals);
slice_tol = 0.1; 

% Detectar triángulos planos en la cima
top_node_indices = find(z_vals > (max_z - slice_tol));
in_layer_mask = ismember(connectivity, top_node_indices);
valid_triangles_mask = all(in_layer_mask, 2);
top_conn = connectivity(valid_triangles_mask, :);

if isempty(top_conn)
    error('Error: No se encontraron triángulos en la capa superior.');
end

fprintf('--> Triángulos analizados en la cara superior: %d\n', size(top_conn, 1));

% --- 4. CÁLCULO DE CENTROIDES POR TRIÁNGULO ---
% En lugar de promediar todo, calculamos el centro (X,Y) de CADA triángulo individual
num_tris = size(top_conn, 1);
tri_centers = zeros(num_tris, 2); % Solo nos importa X e Y para el clustering

for i = 1:num_tris
    v_indices = top_conn(i, :);
    v_coords = points(v_indices, :);
    center_tri = mean(v_coords, 1);
    tri_centers(i, :) = center_tri(1:2); % Guardamos X, Y
end

% --- 5. ALGORITMO K-MEANS (SEGMENTACIÓN GEOMÉTRICA) ---
disp(['--> Ejecutando K-Means para separar ' num2str(target_hexagons) ' clusters...']);

% idx: a qué grupo (1 al 7) pertenece cada triángulo
% C: coordenadas (X,Y) de los centros de esos grupos
rng(1); % Semilla fija para reproducibilidad
[idx_cluster, C_centers_2d] = kmeans(tri_centers, target_hexagons, ...
                                    'Replicates', 5); 

% Recuperamos la altura Z (asumimos plano)
final_Z = max_z;
centers_3d = [C_centers_2d, repmat(final_Z, target_hexagons, 1)];

fprintf('--> ¡Éxito! %d Centros calculados.\n', size(centers_3d, 1));

% --- 6. ORDENAMIENTO DE RUTA (GREEDY) ---
sorted_centers = zeros(size(centers_3d));
is_visited = false(target_hexagons, 1);
current_pos = [0, 0, max_z]; 
count = 0;
pool_centers = centers_3d;

while count < target_hexagons
    dists = sum((pool_centers - current_pos).^2, 2);
    dists(is_visited) = inf; 
    [min_dist, idx_next] = min(dists);
    if isinf(min_dist), break; end
    
    current_pos = pool_centers(idx_next, :);
    sorted_centers(count + 1, :) = current_pos;
    is_visited(idx_next) = true;
    count = count + 1;
end

% --- 7. VISUALIZACIÓN ---
figure('Name', 'Validación de Clustering K-Means', 'Color', 'w');
hold on; axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');

% A) Dibujar triángulos coloreados por cluster (para ver la separación)
cmap = lines(target_hexagons); % Paleta de colores distintivos
for i = 1:target_hexagons
    % Triángulos que pertenecen al hexágono 'i'
    tris_in_group = top_conn(idx_cluster == i, :);
    patch('Faces', tris_in_group, 'Vertices', points, ...
          'FaceColor', cmap(i,:), 'EdgeColor', 'none', 'FaceAlpha', 0.8);
end

% B) Dibujar Centros y Ruta
plot3(sorted_centers(:,1), sorted_centers(:,2), sorted_centers(:,3), ...
      'k.', 'MarkerSize', 20, 'DisplayName', 'Centros');
plot3(sorted_centers(:,1), sorted_centers(:,2), sorted_centers(:,3), ...
      'k-', 'LineWidth', 2, 'DisplayName', 'Ruta Optimizada');

% Inicio
plot3(sorted_centers(1,1), sorted_centers(1,2), sorted_centers(1,3), ...
      'go', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Inicio');

legend('Location', 'bestoutside');
title(['Segmentación Exitosa: ' num2str(target_hexagons) ' Hexágonos']);

% --- 8. EXPORTAR ---
traj_centers = [sorted_centers, ones(target_hexagons, 1)]; 
assignin('base', 'traj_centers', traj_centers);