%% Generador de Trayectoria V12: Base Plana + Pilares Verticales
% Objetivo: Imprimir base hexagonal UNA VEZ y crecer solo con columnas en los centros.

% --- 0. PARÁMETROS DE REPARACIÓN ---
layer_height = 0.5;      % Resolución en Z (mm)
travel_lift  = 3.0;      % Altura de seguridad para viajes (mm)
print_flag   = 1;        % Señal de extrusión (1 = ON)
travel_flag  = 0;        % Señal de viaje (0 = OFF)

% --- 1. CARGA DE STL ---
stl_filename = 'honeycomb.stl'; 
try
    TR = stlread(stl_filename);
catch
    error('No se encuentra el archivo STL. Verifica el nombre.');
end

points = TR.Points;
connectivity = TR.ConnectivityList;

% Escala Automática
if max(points(:)) < 5.0
    disp('--> Escala: Metros detectados. Convirtiendo a mm...');
    points = points * 1000;
end

% --- 2. ANÁLISIS DE GEOMETRÍA ---
z_vals = points(:,3);
z_min = min(z_vals);      % Suelo
z_max = max(z_vals);      % Techo
total_height = z_max - z_min;

% Obtener geometría desde la tapa (que es más limpia)
slice_tol = 0.1;
top_indices = find(z_vals > (z_max - slice_tol));
in_layer_mask = ismember(connectivity, top_indices);
top_triangles = connectivity(all(in_layer_mask, 2), :);

% A) PERÍMETRO (PAREDES) - Solo geometría 2D
edges_raw = [top_triangles(:,1) top_triangles(:,2); ...
             top_triangles(:,2) top_triangles(:,3); ...
             top_triangles(:,3) top_triangles(:,1)];
unique_edges = unique(sort(edges_raw, 2), 'rows');

% Ordenar segmentos (Greedy simple)
segments_start = points(unique_edges(:,1), 1:2);
segments_end   = points(unique_edges(:,2), 1:2);
base_walls_2d = [];
current_xy = segments_start(1,:);
used = false(size(segments_start, 1), 1);

for k = 1:length(used)
    idx = find(~used);
    if isempty(idx), break; end
    % Buscar el más cercano al punto actual
    d1 = sum((segments_start(idx,:) - current_xy).^2, 2);
    d2 = sum((segments_end(idx,:)   - current_xy).^2, 2);
    [min_d1, i1] = min(d1);
    [min_d2, i2] = min(d2);
    
    if min_d2 < min_d1
        best = idx(i2);
        p_s = segments_end(best, :);
        p_e = segments_start(best, :);
    else
        best = idx(i1);
        p_s = segments_start(best, :);
        p_e = segments_end(best, :);
    end
    
    base_walls_2d = [base_walls_2d; p_s; p_e];
    current_xy = p_e;
    used(best) = true;
end

% B) CENTROS (K-Means) - Solo geometría 2D
target_hexagons = 7;
tri_centers = zeros(size(top_triangles,1), 2);
for i = 1:size(top_triangles,1)
    tri_centers(i,:) = mean(points(top_triangles(i,:), 1:2), 1);
end
rng(1); 
[~, centers_2d] = kmeans(tri_centers, target_hexagons, 'Replicates', 5);

% Ordenar Centros para optimizar movimiento
centers_ordered = zeros(target_hexagons, 2);
visited_c = false(target_hexagons, 1);
curr_c = [0,0];
for k = 1:target_hexagons
    dists = sum((centers_2d - curr_c).^2, 2);
    dists(visited_c) = inf;
    [~, idx] = min(dists);
    centers_ordered(k,:) = centers_2d(idx,:);
    curr_c = centers_2d(idx,:);
    visited_c(idx) = true;
end

% --- 3. CONSTRUCCIÓN DE LA TRAYECTORIA ---
full_path = []; % [X, Y, Z, Extrusion_Flag]

fprintf('--> Generando Trayectoria: Base en Z=%.2f, Columnas hasta Z=%.2f\n', z_min, z_max);

% --- FASE 1: IMPRIMIR BASE (SOLO UNA VEZ EN Z_MIN) ---
% Viaje al inicio
start_p = base_walls_2d(1,:);
full_path = [full_path; start_p(1), start_p(2), z_min + travel_lift, travel_flag]; 
full_path = [full_path; start_p(1), start_p(2), z_min,               travel_flag]; 

% Dibujar Hexágonos (Base)
walls_z = repmat(z_min, size(base_walls_2d,1), 1);
walls_flag = repmat(print_flag, size(base_walls_2d,1), 1);
full_path = [full_path; base_walls_2d, walls_z, walls_flag];

% --- FASE 2: IMPRIMIR PILARES (CAPA POR CAPA HACIA ARRIBA) ---
num_layers = floor(total_height / layer_height);
current_z = z_min;
last_xy = full_path(end, 1:2);

for layer = 1:num_layers
    current_z = current_z + layer_height;
    
    % En cada capa, visitamos SOLO los 7 centros
    for c = 1:target_hexagons
        cx = centers_ordered(c, 1);
        cy = centers_ordered(c, 2);
        
        % Viaje aéreo seguro al centro
        full_path = [full_path; last_xy(1), last_xy(2), current_z + travel_lift, travel_flag];
        full_path = [full_path; cx,         cy,         current_z + travel_lift, travel_flag];
        
        % Bajar e Imprimir PUNTO (Extrusión vertical pequeña o punto)
        full_path = [full_path; cx, cy, current_z, travel_flag]; % Aproximación
        full_path = [full_path; cx, cy, current_z, print_flag];  % DEPOSITAR MATERIAL
        
        last_xy = [cx, cy];
    end
end

% Salida final segura
full_path = [full_path; last_xy(1), last_xy(2), current_z + 10, travel_flag];

% --- 4. VISUALIZACIÓN ---
figure('Name', 'Trayectoria V12: Base y Pilares', 'Color', 'w');
hold on; axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');

% STL Fantasma
patch('Faces', connectivity, 'Vertices', points, ...
      'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.2);

% Separar para graficar
p_travel = full_path(full_path(:,4) == 0, :);
p_print  = full_path(full_path(:,4) == 1, :);

% Gráfica
plot3(p_travel(:,1), p_travel(:,2), p_travel(:,3), 'b:', 'LineWidth', 0.5, 'DisplayName', 'Viajes (Aire)');
plot3(p_print(:,1), p_print(:,2), p_print(:,3), 'r.-', 'LineWidth', 2, 'DisplayName', 'Extrusión (Material)');

legend show;
title('Estrategia: Base Única + Columnas Crecientes');

% --- 5. EXPORTAR A SIMULINK ---
% Interpolación
t_total = size(full_path,1) * 0.1;
num_sim = size(full_path,1) * 2;
t_vec = linspace(0, t_total, num_sim)';
idx_raw = 1:size(full_path,1);
idx_sim = linspace(1, size(full_path,1), num_sim);

xyz_sim = interp1(idx_raw, full_path(:,1:3), idx_sim, 'linear') / 1000; % mm -> m
flag_sim = interp1(idx_raw, full_path(:,4), idx_sim, 'nearest');

ref_data_8dof.time = t_vec;
ref_data_8dof.signals.values = [xyz_sim, flag_sim', zeros(num_sim, 4)];
ref_data_8dof.signals.dimensions = 8;

assignin('base', 'ref_data_8dof', ref_data_8dof);
disp('--> Listo. Ahora solo suben los centroides (columnas).');