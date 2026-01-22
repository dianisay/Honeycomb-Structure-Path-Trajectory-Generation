%% Generador de Trayectoria V6: Wireframe + Visualización STL Completo

% --- 1. CARGA DE STL ---
stl_filename = 'honeycomb.stl'; 
try
    TR = stlread(stl_filename);
catch
    error('No se encuentra el archivo STL. Asegúrate de que está en la carpeta.');
end

% A. Escala Inteligente
points = TR.Points;
connectivity = TR.ConnectivityList;

if max(points(:)) < 5.0
    disp('--> Escala: Detectado Metros. Convirtiendo a mm...');
    points = points * 1000;
else
    disp('--> Escala: Milímetros detectados.');
end

% --- 2. SLICING: FILTRADO DE LA CAPA SUPERIOR ---
z_vals = points(:,3);
max_z = max(z_vals);
slice_tol = 0.1; % Tolerancia fina para capturar solo la cara plana superior

% Índices de vértices en la tapa
top_node_indices = find(z_vals > (max_z - slice_tol));

% Filtrar triángulos que tienen sus 3 vértices en la tapa
in_layer_mask = ismember(connectivity, top_node_indices);
valid_triangles_mask = all(in_layer_mask, 2);
top_triangles = connectivity(valid_triangles_mask, :);

if isempty(top_triangles)
    error('No se encontraron triángulos en la capa superior. Revisa max_z.');
end

% --- 3. EXTRACCIÓN DE ARISTAS (WIREFRAME) ---
% Aristas: [A B], [B C], [C A]
edges_raw = [top_triangles(:,1) top_triangles(:,2); ...
             top_triangles(:,2) top_triangles(:,3); ...
             top_triangles(:,3) top_triangles(:,1)];

% Ordenar y eliminar duplicados para obtener líneas únicas
edges_sorted = sort(edges_raw, 2);
unique_edges = unique(edges_sorted, 'rows');

fprintf('--> Geometría Detectada: %d segmentos de línea.\n', size(unique_edges, 1));

segments_start = points(unique_edges(:,1), :);
segments_end   = points(unique_edges(:,2), :);

% --- 4. ORDENAMIENTO DE RUTA (ALGORITMO GREEDY) ---
sorted_path = [];
pool_start = segments_start;
pool_end = segments_end;
num_segments = size(pool_start, 1);
is_used = false(num_segments, 1);

current_pos = [0, 0, max_z]; 
segments_ordered_count = 0;

while segments_ordered_count < num_segments
    indices = find(~is_used);
    if isempty(indices), break; end
    
    d_start = sum((pool_start(indices,:) - current_pos).^2, 2);
    d_end   = sum((pool_end(indices,:)   - current_pos).^2, 2);
    
    [min_d_start, idx_s] = min(d_start);
    [min_d_end, idx_e]   = min(d_end);
    
    if min_d_start <= min_d_end
        best_idx_global = indices(idx_s);
        seg_p1 = pool_start(best_idx_global, :);
        seg_p2 = pool_end(best_idx_global, :);
    else
        best_idx_global = indices(idx_e);
        seg_p1 = pool_end(best_idx_global, :);
        seg_p2 = pool_start(best_idx_global, :);
    end
    
    % Travel moves (saltos)
    dist_jump = norm(current_pos - seg_p1);
    if dist_jump > 1.0 
        sorted_path = [sorted_path; [NaN NaN NaN]]; 
    end
    
    sorted_path = [sorted_path; seg_p1; seg_p2];
    current_pos = seg_p2;
    is_used(best_idx_global) = true;
    segments_ordered_count = segments_ordered_count + 1;
end

% --- 5. VISUALIZACIÓN COMPLETA (STL + RUTA) ---
h_fig = figure('Name', 'Trayectoria Final sobre STL', 'Color', 'w');
hold on; axis equal; grid on; view(3);
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');

% A) Dibujar el STL original (Fantasma Gris)
% Usamos 'Faces' del STL original pero 'Vertices' escalados
patch('Faces', connectivity, 'Vertices', points, ...
      'FaceColor', [0.8 0.8 0.8], ...  % Gris claro
      'EdgeColor', 'none', ...         % Sin bordes negros para limpieza
      'FaceAlpha', 0.4);               % Transparencia (0.0 a 1.0)

% Iluminación para que se vea 3D
camlight('headlight'); lighting gouraud; 

% B) Dibujar la Trayectoria Calculada (Línea Roja Gruesa)
plot3(sorted_path(:,1), sorted_path(:,2), sorted_path(:,3), ...
      'r-', 'LineWidth', 2.5);

% C) Marcar Inicio (Verde) y Fin (Azul)
start_pt = sorted_path(1,:);
end_pt = sorted_path(end,:);
plot3(start_pt(1), start_pt(2), start_pt(3), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Inicio');
plot3(end_pt(1), end_pt(2), end_pt(3), 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Fin');

legend('STL Original', 'Trayectoria Robot', 'Inicio', 'Fin');
title(['Validación Visual: ' num2str(num_segments) ' segmentos extraídos']);

% --- 6. EXPORTAR A SIMULINK ---
clean_path = sorted_path(~isnan(sorted_path(:,1)), :); 
path_m = clean_path / 1000; % mm -> m

% Interpolación 
t_total = 20;
num_steps = size(path_m, 1) * 10; 
t_vec = linspace(0, t_total, num_steps)';
raw_idx = 1:size(path_m,1);
sim_idx = linspace(1, size(path_m,1), num_steps);

traj_x = interp1(raw_idx, path_m(:,1), sim_idx, 'linear');
traj_y = interp1(raw_idx, path_m(:,2), sim_idx, 'linear');
traj_z = interp1(raw_idx, path_m(:,3), sim_idx, 'linear');

final_data = zeros(num_steps, 8);
final_data(:,1) = traj_x'; 
final_data(:,2) = traj_y'; 
final_data(:,3) = traj_z'; 

ref_data_8dof.time = t_vec;
ref_data_8dof.signals.values = final_data;
ref_data_8dof.signals.dimensions = 8;

assignin('base', 'ref_data_8dof', ref_data_8dof);
disp('--> Listo. Corre Simulink.');