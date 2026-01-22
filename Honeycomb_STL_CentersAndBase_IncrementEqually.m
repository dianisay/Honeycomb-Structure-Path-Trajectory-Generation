%% Generador de Trayectoria V11: De Abajo hacia Arriba (Layer-by-Layer)
% Objetivo: Imprimir base en Z_min y crecer volumétricamente de forma segura.

% --- 0. PARÁMETROS DE REPARACIÓN ---
layer_height = 0.5;      % Resolución en Z (mm)
travel_lift  = 5.0;      % Altura de seguridad para viajes (mm)
print_speed_flag = 1;    % Señal para Simulink (1 = Extruir)
travel_speed_flag = 0;   % Señal para Simulink (0 = Viaje)

% --- 1. CARGA DE STL ---
stl_filename = 'honeycomb.stl'; 
try
    TR = stlread(stl_filename);
catch
    error('No se encuentra el archivo STL. Verifica el nombre.');
end

points = TR.Points;
connectivity = TR.ConnectivityList;

% Escala Automática (Metros -> Milímetros si es necesario)
if max(points(:)) < 5.0
    disp('--> Escala: Metros detectados. Convirtiendo a mm...');
    points = points * 1000;
end

% --- 2. ANÁLISIS DE LÍMITES (BOUNDING BOX) ---
z_vals = points(:,3);
z_min = min(z_vals);      % El suelo del objeto ("Hasta abajo")
z_max = max(z_vals);      % El techo del objeto
total_height = z_max - z_min;

fprintf('--> Análisis Z: Min=%.2f mm, Max=%.2f mm. Altura Total=%.2f mm\n', ...
        z_min, z_max, total_height);

% --- 3. EXTRACCIÓN DE GEOMETRÍA (DESDE LA TAPA PARA MAYOR LIMPIEZA) ---
% Usamos la capa superior para leer la forma, pero la imprimiremos abajo.
slice_tol = 0.1;
top_indices = find(z_vals > (z_max - slice_tol));
in_layer_mask = ismember(connectivity, top_indices);
top_triangles = connectivity(all(in_layer_mask, 2), :);

if isempty(top_triangles)
    error('No se pudo leer la geometría superior para proyectarla.');
end

% A) Obtener Perímetros (Walls)
edges_raw = [top_triangles(:,1) top_triangles(:,2); ...
             top_triangles(:,2) top_triangles(:,3); ...
             top_triangles(:,3) top_triangles(:,1)];
unique_edges = unique(sort(edges_raw, 2), 'rows');

% Ordenar Perímetros (Greedy para optimizar trazo)
segments_start = points(unique_edges(:,1), 1:2); % Solo X,Y
segments_end   = points(unique_edges(:,2), 1:2); % Solo X,Y
base_path_2d = [];
current_xy = [0, 0];
used = false(size(segments_start, 1), 1);

for k = 1:length(used)
    % Buscar segmento más cercano no usado
    idx = find(~used);
    if isempty(idx), break; end
    
    d1 = sum((segments_start(idx,:) - current_xy).^2, 2);
    d2 = sum((segments_end(idx,:)   - current_xy).^2, 2);
    [min_d1, i1] = min(d1);
    [min_d2, i2] = min(d2);
    
    best_idx = idx(i1); % Por defecto
    if min_d2 < min_d1
        best_idx = idx(i2);
        p_start = segments_end(best_idx, :);
        p_end   = segments_start(best_idx, :);
    else
        p_start = segments_start(best_idx, :);
        p_end   = segments_end(best_idx, :);
    end
    
    % Añadir a la ruta 2D
    base_path_2d = [base_path_2d; p_start; p_end];
    current_xy = p_end;
    used(best_idx) = true;
end

% B) Obtener Centros (K-Means)
target_hexagons = 7;
tri_centers = zeros(size(top_triangles,1), 2);
for i = 1:size(top_triangles,1)
    tri_centers(i,:) = mean(points(top_triangles(i,:), 1:2), 1);
end
rng(1); % Semilla fija
[~, centers_2d] = kmeans(tri_centers, target_hexagons, 'Replicates', 5);

% Ordenar Centros (Greedy)
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

% --- 4. GENERACIÓN VOLUMÉTRICA (LAYER-BY-LAYER) ---
full_path = []; % [X, Y, Z, Extrusion_Flag]
num_layers = floor(total_height / layer_height);
fprintf('--> Generando %d capas desde Z=%.2f hasta Z=%.2f...\n', num_layers, z_min, z_max);

current_z = z_min; % ¡EMPEZAMOS ABAJO!

for layer = 1:num_layers
    % 1. Fase Paredes (Perímetro)
    % ---------------------------
    % Aproximación (Viaje)
    start_p = base_path_2d(1,:);
    full_path = [full_path; start_p(1), start_p(2), current_z + travel_lift, travel_speed_flag];
    full_path = [full_path; start_p(1), start_p(2), current_z,               travel_speed_flag];
    
    % Imprimir Paredes
    xy_layer = base_path_2d;
    z_col = repmat(current_z, size(xy_layer,1), 1);
    flag_col = repmat(print_speed_flag, size(xy_layer,1), 1);
    full_path = [full_path; xy_layer, z_col, flag_col];
    
    % 2. Fase Relleno (Centros)
    % -------------------------
    last_p = full_path(end, 1:2);
    
    for c = 1:target_hexagons
        cx = centers_ordered(c, 1);
        cy = centers_ordered(c, 2);
        
        % Viaje seguro (levantar)
        full_path = [full_path; last_p(1), last_p(2), current_z + travel_lift, travel_speed_flag];
        full_path = [full_path; cx,        cy,        current_z + travel_lift, travel_speed_flag];
        
        % Bajar e Imprimir Punto
        full_path = [full_path; cx, cy, current_z, travel_speed_flag]; % Pre-pos
        full_path = [full_path; cx, cy, current_z, print_speed_flag];  % EXTRUSIÓN
        
        last_p = [cx, cy];
    end
    
    % 3. Preparar siguiente capa
    % Terminar subiendo para no raspar
    full_path = [full_path; last_p(1), last_p(2), current_z + travel_lift, travel_speed_flag];
    
    % Incrementar Z
    current_z = current_z + layer_height;
end

% --- 5. VISUALIZACIÓN ---
figure('Name', 'Trayectoria V11: De Abajo hacia Arriba', 'Color', 'w');
hold on; axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');

% STL Fantasma
patch('Faces', connectivity, 'Vertices', points, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% Trayectoria
print_idx = full_path(:,4) == 1;
travel_idx = full_path(:,4) == 0;

plot3(full_path(travel_idx,1), full_path(travel_idx,2), full_path(travel_idx,3), 'b:', 'LineWidth', 0.5, 'DisplayName', 'Viajes');
plot3(full_path(print_idx,1), full_path(print_idx,2), full_path(print_idx,3), 'r.-', 'LineWidth', 1, 'DisplayName', 'Impresión');

% Marcar Inicio (Debe estar abajo)
plot3(full_path(1,1), full_path(1,2), full_path(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Inicio (Suelo)');

legend show;
title(['Estrategia: Capa por Capa | Altura: ' num2str(total_height) 'mm']);

% --- 6. EXPORTAR A SIMULINK ---
% Interpolación para suavizar (Timeseries)
num_points = size(full_path, 1);
t_total = num_points * 0.1; 
num_sim_steps = num_points * 2; % Upsampling x2

t_vec = linspace(0, t_total, num_sim_steps)';
raw_idx = 1:num_points;
sim_idx = linspace(1, num_points, num_sim_steps);

% Interpolar XYZ
x_sim = interp1(raw_idx, full_path(:,1), sim_idx, 'linear');
y_sim = interp1(raw_idx, full_path(:,2), sim_idx, 'linear');
z_sim = interp1(raw_idx, full_path(:,3), sim_idx, 'linear');
flag_sim = interp1(raw_idx, full_path(:,4), sim_idx, 'nearest'); % Banderas no se interpolan linealmente

% Convertir a Metros para Simulink (si tu modelo usa metros)
% Asumiendo que full_path está en mm, dividimos por 1000.
xyz_meters = [x_sim', y_sim', z_sim'] / 1000;

% Matriz final [X, Y, Z, Flag, 0, 0, 0, 0] (8 columnas para tu bus)
final_data = zeros(num_sim_steps, 8);
final_data(:,1:3) = xyz_meters;
final_data(:,4)   = flag_sim';

ref_data_8dof.time = t_vec;
ref_data_8dof.signals.values = final_data;
ref_data_8dof.signals.dimensions = 8;

assignin('base', 'ref_data_8dof', ref_data_8dof);
disp('--> Variable "ref_data_8dof" actualizada. Verifica que la gráfica empiece en Z mínimo.');