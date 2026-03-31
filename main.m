%% 基于SSA与ISSA的无人机3D路径规划对比程序

clear; clc; close all;

%% 1. 环境参数设置（威胁等效地形建模）
fprintf('=== Initializing threat equivalent terrain environment ===\n');

terrain_params.p      = 8;
terrain_params.h      = [55, 50, 70, 60, 30, 40, 70, 70];
terrain_params.x_c    = [45, 60, 80, 90, 100, 125, 150, 160];
terrain_params.y_c    = [30, 160, 100, 175, 35, 75, 120, 40];
terrain_params.alpha_k= [20, 15, 30, 20, 20, 15, 30, 20];
terrain_params.beta_k = [20, 15, 30, 20, 20, 15, 30, 20];

obstacle_params.num = 5;
obstacle_params.x_O = [50, 105, 110, 130, 170];
obstacle_params.y_O = [130, 140, 90, 30, 65];
obstacle_params.r_O = [10, 10, 7, 15, 8];
obstacle_params.h_O = [50, 70, 80, 50, 60];

start_point     = [1, 20, 30];
end_point       = [200, 160, 40];
direct_distance = norm(end_point - start_point);

weights = [5, 1.0, 0.5, 0.1];
h_safe  = 5;

fprintf('Threat equivalent terrain environment initialization completed\n');
fprintf('Direct distance: %.1f m\n', direct_distance);

%% 2. 算法参数设置
num_waypoints = 8;
dim           = 3 * num_waypoints;
num_runs      = 5;

lb = zeros(1, dim);
ub = zeros(1, dim);

direction = (end_point - start_point) / norm(end_point - start_point);

perpendicular1 = [-direction(2), direction(1), 0];
if norm(perpendicular1) < 1e-6
    perpendicular1 = [1, 0, 0];
else
    perpendicular1 = perpendicular1 / norm(perpendicular1);
end

for i = 1:num_waypoints
    progress      = i / (num_waypoints + 1);
    center        = start_point + progress * (end_point - start_point);
    search_radius = 40;
    lb(3*(i-1)+1) = max(0,   center(1) - search_radius);
    ub(3*(i-1)+1) = min(200, center(1) + search_radius);
    lb(3*(i-1)+2) = max(0,   center(2) - search_radius);
    ub(3*(i-1)+2) = min(200, center(2) + search_radius);
    lb(3*(i-1)+3) = 20;
    ub(3*(i-1)+3) = 100;
end

N        = 80;
Max_iter = 1000;

fprintf('Algorithm parameters setting completed\n');

%% 3. 适应度函数
fobj = @(path) optimized_fitness_function_v3(path, start_point, end_point, ...
    terrain_params, obstacle_params, weights, h_safe);

%% 4. 初始化结果存储
algorithms = {'SSA', 'ISSA'};
results    = struct();

for alg_idx = 1:length(algorithms)
    alg = algorithms{alg_idx};
    results.(alg).best_fitness      = zeros(1, num_runs);
    results.(alg).best_path         = [];
    results.(alg).convergence       = zeros(num_runs, Max_iter);
    results.(alg).computation_time  = zeros(1, num_runs);
    results.(alg).final_paths       = {};
    results.(alg).safety_violations = zeros(1, num_runs);
    results.(alg).path_efficiency   = zeros(1, num_runs);
    results.(alg).path_lengths      = zeros(1, num_runs);
    results.(alg).valid_runs        = [];
end

%% 5. 执行所有算法
for alg_idx = 1:length(algorithms)
    alg = algorithms{alg_idx};
    fprintf('\n=== Starting %s-based path planning experiment ===\n', alg);

    for run = 1:num_runs
        fprintf('%s - Run %d/%d... ', alg, run, num_runs);
        tic;

        try
            switch alg
                case 'SSA'
                    [fMin, bestPath, Curve] = SSA(N, Max_iter, lb, ub, dim, fobj);
                case 'ISSA'
                    [fMin, bestPath, Curve] = ISSA(N, Max_iter, lb, ub, dim, fobj);
            end
        catch ME
            fprintf('\n%s algorithm error: %s\n', alg, ME.message);
            continue;
        end

        elapsed_time = toc;

        results.(alg).best_fitness(run)     = fMin;
        results.(alg).convergence(run, :)   = Curve;
        results.(alg).computation_time(run) = elapsed_time;
        results.(alg).valid_runs(end+1)     = run;

        waypoints  = reshape(bestPath, 3, num_waypoints)';
        full_path  = [start_point; waypoints; end_point];

        smooth_path = improved_smooth_path(full_path, terrain_params, obstacle_params, h_safe);
        results.(alg).final_paths{run} = smooth_path;

        path_length = calculate_path_length(smooth_path);
        results.(alg).path_lengths(run) = path_length;

        path_efficiency = direct_distance / path_length * 100;
        results.(alg).path_efficiency(run) = path_efficiency;

        violations = check_safety_violations(smooth_path, terrain_params, obstacle_params, h_safe);
        results.(alg).safety_violations(run) = violations;

        if run == 1 || fMin < min(results.(alg).best_fitness(1:run-1))
            results.(alg).best_path = bestPath;
        end

        fprintf('Fitness=%.4f, Path length=%.1fm, Efficiency=%.1f%%, Violations=%d, Time=%.2fs\n', ...
                fMin, path_length, path_efficiency, violations, elapsed_time);
    end
end

%% 6. 统计分析
fprintf('\n=== Statistical Analysis ===\n');

stats = struct();

for alg_idx = 1:length(algorithms)
    alg = algorithms{alg_idx};

    valid_runs = results.(alg).valid_runs;

    if ~isempty(valid_runs)
        valid_fitness      = results.(alg).best_fitness(valid_runs);
        valid_path_lengths = results.(alg).path_lengths(valid_runs);

        stats.(alg).best             = min(valid_fitness);
        stats.(alg).mean             = mean(valid_fitness);
        stats.(alg).std              = std(valid_fitness);
        stats.(alg).worst            = max(valid_fitness);
        stats.(alg).avg_time         = mean(results.(alg).computation_time(valid_runs));
        stats.(alg).std_time         = std(results.(alg).computation_time(valid_runs));
        stats.(alg).avg_violations   = mean(results.(alg).safety_violations(valid_runs));
        stats.(alg).success_rate     = sum(results.(alg).safety_violations(valid_runs) == 0) ...
                                       / length(valid_runs) * 100;
        stats.(alg).avg_efficiency   = mean(results.(alg).path_efficiency(valid_runs));
        stats.(alg).best_path_length = min(valid_path_lengths);
        stats.(alg).avg_path_length  = mean(valid_path_lengths);
        stats.(alg).std_path_length  = std(valid_path_lengths);

        [~, best_idx_relative] = min(valid_fitness);
        best_idx               = valid_runs(best_idx_relative);
        best_path              = results.(alg).final_paths{best_idx};
        stats.(alg).path_length= calculate_path_length(best_path);
        stats.(alg).smoothness = calculate_path_smoothness(best_path);
        stats.(alg).best_idx   = best_idx;

        fprintf('\n--- %s Algorithm Results ---\n', alg);
        fprintf('Success rate: %.1f%%\n', stats.(alg).success_rate);
        fprintf('Best fitness: %.4f (Mean: %.4f +/- %.4f)\n', ...
                stats.(alg).best, stats.(alg).mean, stats.(alg).std);
        fprintf('Best path length: %.1f m (Mean: %.1f +/- %.1f)\n', ...
                stats.(alg).best_path_length, stats.(alg).avg_path_length, ...
                stats.(alg).std_path_length);
        fprintf('Path efficiency: %.1f%%\n', ...
                direct_distance / stats.(alg).path_length * 100);
        fprintf('Path smoothness: %.3f\n', stats.(alg).smoothness);
        fprintf('Average computation time: %.2f +/- %.2f s\n', ...
                stats.(alg).avg_time, stats.(alg).std_time);
        fprintf('Average safety violations: %.1f\n', stats.(alg).avg_violations);
    end
end

%% 7. 设置保存路径
save_path = fullfile(pwd, '3D_Results');

if ~exist(save_path, 'dir')
    mkdir(save_path);
    fprintf('Created save path: %s\n', save_path);
end

%% 8. 导出结果到Excel
export_results_to_excel(results, stats, algorithms, direct_distance, save_path);

%% 9. 绘制并保存四个独立图形
plot_3d_path_comparison(results, stats, algorithms, terrain_params, obstacle_params, ...
                        start_point, end_point, save_path);
plot_top_view_comparison(results, stats, algorithms, terrain_params, obstacle_params, ...
                         start_point, end_point, save_path);
plot_convergence_comparison(results, algorithms, save_path);
plot_boxplot_comparison(results, algorithms, save_path);

fprintf('\nAll done! Figures and tables saved to: %s\n', save_path);


%% ========================================================================
%%                          工具函数区
%% ========================================================================

%% 适应度函数
function fitness = optimized_fitness_function_v3(path, start_point, end_point, ...
        terrain_params, obstacle_params, weights, h_safe)

    num_waypoints = length(path) / 3;
    path_points   = reshape(path, 3, num_waypoints)';
    full_path     = [start_point; path_points; end_point];
    n             = size(full_path, 1);

    [obstacle_penalty, num_violations] = check_obstacle_collision_v2(full_path, obstacle_params, h_safe);

    % 1. 路径长度代价
    path_length = 0;
    direct_dist = norm(end_point - start_point);
    for i = 1:n-1
        path_length = path_length + norm(full_path(i+1,:) - full_path(i,:));
    end
    C1 = max(0, (path_length - direct_dist) / max(direct_dist, 1e-6));

    % 2. 威胁等效地形代价
    threat_count    = 0;
    threat_severity = 0;
    for i = 1:n
        h_terrain = calculate_threat_terrain_height(full_path(i,1), full_path(i,2), terrain_params);
        clearance = full_path(i,3) - h_terrain;
        if clearance < h_safe
            threat_count    = threat_count + 1;
            threat_severity = threat_severity + (h_safe - clearance);
        end
    end
    C2 = (threat_count / n) + (threat_severity / (100 * n));

    % 3. 平滑度代价
    angle_sum         = 0;
    angle_count       = 0;
    height_change_sum = 0;

    for i = 2:n-1
        v1 = full_path(i,:)   - full_path(i-1,:);
        v2 = full_path(i+1,:) - full_path(i,:);
        if norm(v1) > 0 && norm(v2) > 0
            cos_angle = dot(v1,v2) / (norm(v1)*norm(v2));
            cos_angle = max(-1, min(1, cos_angle));
            angle     = acos(cos_angle);
            if angle > deg2rad(10)
                angle_sum   = angle_sum + angle;
                angle_count = angle_count + 1;
            end
        end
    end

    for i = 1:n-1
        height_change = abs(full_path(i+1,3) - full_path(i,3));
        if height_change > 5
            height_change_sum = height_change_sum + height_change;
        end
    end

    avg_angle         = angle_sum / max(1, angle_count);
    avg_height_change = height_change_sum / n;
    C3 = (avg_angle / pi) + (avg_height_change / 50);

    % 4. 高度代价
    C4 = mean(full_path(:,3)) / 100;

    % 正常代价
    regular_fitness = weights(1)*C1 + weights(2)*C2 + weights(3)*C3 + weights(4)*C4;

    % 梯度惩罚：碰撞越多 fitness 越高，同时保留正常代价的梯度信息
    if num_violations > 0
        fitness = 500 * num_violations + regular_fitness;
    else
        fitness = regular_fitness;
    end
end

%% 碰撞检测（梯度化 + 统一安全裕度 + 密采样）
function [penalty, num_violations] = check_obstacle_collision_v2(path, obstacle_params, h_safe)
    penalty        = 0;
    num_violations = 0;
    n              = size(path, 1);
    safety_margin  = 2;

    % 航点检测
    for i = 1:n
        for j = 1:obstacle_params.num
            dist_xy = sqrt((path(i,1) - obstacle_params.x_O(j))^2 + ...
                           (path(i,2) - obstacle_params.y_O(j))^2);
            if dist_xy < obstacle_params.r_O(j) + safety_margin
                if path(i,3) < obstacle_params.h_O(j) + h_safe
                    penalty        = penalty + 1000;
                    num_violations = num_violations + 1;
                end
            end
        end
    end

    % 段插值检测（20个采样点）
    for i = 1:n-1
        for j = 1:obstacle_params.num
            for t_frac = linspace(0, 1, 20)
                point   = path(i,:) + t_frac * (path(i+1,:) - path(i,:));
                dist_xy = sqrt((point(1) - obstacle_params.x_O(j))^2 + ...
                               (point(2) - obstacle_params.y_O(j))^2);
                if dist_xy < obstacle_params.r_O(j) + safety_margin && ...
                        point(3) < obstacle_params.h_O(j) + h_safe
                    penalty        = penalty + 500;
                    num_violations = num_violations + 1;
                    break;
                end
            end
        end
    end
end

%% 路径平滑（平滑不引入新碰撞）
function smooth_path = improved_smooth_path(path, terrain_params, obstacle_params, min_clearance)
    if isempty(path) || size(path, 1) < 2
        smooth_path = path;
        return;
    end

    smooth_path = path;
    n           = size(path, 1);

    % pchip 插值（若引入碰撞则保留原始路径）
    if n > 3
        try
            t_orig      = 1:n;
            t_new       = linspace(1, n, n*5);
            interp_path = interp1(t_orig, path, t_new, 'pchip');

            if ~path_has_collision(interp_path, terrain_params, obstacle_params, min_clearance)
                smooth_path = interp_path;
                n           = size(smooth_path, 1);
            else
                fprintf('Warning: pchip interpolation introduced collision, keeping original\n');
            end
        catch
            fprintf('Warning: Interpolation failed\n');
        end
    end

    % 高斯平滑（逐点验证）
    for iter = 1:10
        temp_path = smooth_path;
        for i = 2:n-1
            w1 = 0.3; w2 = 0.4; w3 = 0.3;
            new_pos = w1*smooth_path(i-1,:) + w2*smooth_path(i,:) + w3*smooth_path(i+1,:);

            % 地形高度保护
            h_terrain  = calculate_threat_terrain_height(new_pos(1), new_pos(2), terrain_params);
            new_pos(3) = max(new_pos(3), h_terrain + min_clearance);

            % 障碍物高度保护
            for j = 1:obstacle_params.num
                dist_xy = sqrt((new_pos(1) - obstacle_params.x_O(j))^2 + ...
                               (new_pos(2) - obstacle_params.y_O(j))^2);
                if dist_xy < obstacle_params.r_O(j) + 2
                    if new_pos(3) < obstacle_params.h_O(j) + min_clearance
                        new_pos(3) = obstacle_params.h_O(j) + min_clearance + 2;
                    end
                end
            end

            % 仅当平滑后不在障碍内时才接受
            collision = false;
            for j = 1:obstacle_params.num
                dist_xy = sqrt((new_pos(1) - obstacle_params.x_O(j))^2 + ...
                               (new_pos(2) - obstacle_params.y_O(j))^2);
                if dist_xy < obstacle_params.r_O(j) + 1 && ...
                   new_pos(3) < obstacle_params.h_O(j) + min_clearance
                    collision = true;
                    break;
                end
            end

            if ~collision
                temp_path(i,:) = new_pos;
            end
        end
        smooth_path = temp_path;
    end

    % 路径简化 + 自适应采样
    smooth_path = path_shortening(smooth_path, terrain_params, obstacle_params, min_clearance);
    smooth_path = adaptive_sampling(smooth_path, 30);

    fprintf('Final path points: %d\n', size(smooth_path, 1));
end

%% ★ [修复] 碰撞检测辅助函数：与适应度函数使用相同安全裕度（safety_margin=2）
function has_col = path_has_collision(path, terrain_params, obstacle_params, min_clearance)
    has_col       = false;
    n             = size(path, 1);
    safety_margin = 2;   % ★ 修复：与 check_obstacle_collision_v2 保持一致（原代码裕度=0）
    for i = 1:n
        % 地形检查
        h_terrain = calculate_threat_terrain_height(path(i,1), path(i,2), terrain_params);
        if path(i,3) < h_terrain + min_clearance
            has_col = true;
            return;
        end
        % 障碍物检查
        for j = 1:obstacle_params.num
            dist_xy = sqrt((path(i,1) - obstacle_params.x_O(j))^2 + ...
                           (path(i,2) - obstacle_params.y_O(j))^2);
            if dist_xy < obstacle_params.r_O(j) + safety_margin && ...
               path(i,3) < obstacle_params.h_O(j) + min_clearance
                has_col = true;
                return;
            end
        end
    end
end

%% 路径简化
function shortened_path = path_shortening(path, terrain_params, obstacle_params, min_clearance)
    n = size(path, 1);
    if n < 3
        shortened_path = path;
        return;
    end
    keep = true(n, 1);
    i    = 1;
    while i < n-1
        j = i + 2;
        while j <= n
            if check_path_segment_valid(path(i,:), path(j,:), terrain_params, obstacle_params, min_clearance)
                keep(i+1:j-1) = false;
                i = j;
                break;
            else
                j = j + 1;
            end
        end
        if j > n
            i = i + 1;
        end
    end
    shortened_path = path(keep, :);
end

%% 检查路径段有效性
function valid = check_path_segment_valid(p1, p2, terrain_params, obstacle_params, min_clearance)
    valid   = true;
    samples = 30;
    for t_frac = linspace(0, 1, samples)
        point     = p1 + t_frac * (p2 - p1);
        h_terrain = calculate_threat_terrain_height(point(1), point(2), terrain_params);
        if point(3) < h_terrain + min_clearance
            valid = false;
            return;
        end
        for j = 1:obstacle_params.num
            dist_xy = sqrt((point(1) - obstacle_params.x_O(j))^2 + ...
                           (point(2) - obstacle_params.y_O(j))^2);
            if dist_xy < obstacle_params.r_O(j) + 1 && point(3) < obstacle_params.h_O(j) + min_clearance
                valid = false;
                return;
            end
        end
    end
end

%% 自适应采样
function sampled_path = adaptive_sampling(path, target_points)
    n = size(path, 1);
    if n <= target_points
        sampled_path = path;
        return;
    end
    importance = zeros(n-1, 1);
    for i = 1:n-1
        dist = norm(path(i+1,:) - path(i,:));
        if i > 1
            v1 = path(i,:)   - path(i-1,:);
            v2 = path(i+1,:) - path(i,:);
            if norm(v1) > 0 && norm(v2) > 0
                cos_angle     = dot(v1,v2) / (norm(v1)*norm(v2));
                angle         = acos(max(-1, min(1, cos_angle)));
                importance(i) = dist + 10 * angle;
            else
                importance(i) = dist;
            end
        else
            importance(i) = dist;
        end
    end
    [~, idx] = sort(importance, 'descend');
    keep     = false(n, 1);
    keep([1, n]) = true;
    keep(idx(1:min(target_points-2, length(idx)))) = true;
    sampled_path = path(keep, :);
end

%% 威胁等效地形高度
function z = calculate_threat_terrain_height(x, y, terrain_params)
    z = 0;
    for i = 1:terrain_params.p
        z = z + terrain_params.h(i) * ...
            exp(-((x - terrain_params.x_c(i))  / terrain_params.alpha_k(i))^2 ...
                -((y - terrain_params.y_c(i))   / terrain_params.beta_k(i))^2);
    end
end

%% ★ [修复] 安全检查：统一安全裕度（原代码裕度=0，导致报告违规数偏低）
function violations = check_safety_violations(path, terrain_params, obstacle_params, h_safe)
    violations    = 0;
    n             = size(path, 1);
    safety_margin = 2;   % ★ 修复：与适应度函数保持一致
    for i = 1:n
        h_terrain = calculate_threat_terrain_height(path(i,1), path(i,2), terrain_params);
        if path(i,3) < h_terrain + h_safe
            violations = violations + 1;
        end
        for j = 1:obstacle_params.num
            dist_xy = sqrt((path(i,1) - obstacle_params.x_O(j))^2 + ...
                           (path(i,2) - obstacle_params.y_O(j))^2);
            if dist_xy < obstacle_params.r_O(j) + safety_margin && ...
               path(i,3) < obstacle_params.h_O(j) + h_safe
                violations = violations + 1;
            end
        end
    end
end

%% 路径长度
function total_len = calculate_path_length(path)
    total_len = 0;
    for i = 1:size(path, 1)-1
        total_len = total_len + norm(path(i+1,:) - path(i,:));
    end
end

%% 路径平滑度
function smoothness = calculate_path_smoothness(path)
    smoothness = 0;
    n          = size(path, 1);
    if n < 3, return; end
    for i = 2:n-1
        v1 = path(i,:)   - path(i-1,:);
        v2 = path(i+1,:) - path(i,:);
        if norm(v1) > 0 && norm(v2) > 0
            cos_angle  = dot(v1,v2) / (norm(v1)*norm(v2));
            cos_angle  = max(-1, min(1, cos_angle));
            smoothness = smoothness + acos(cos_angle);
        end
    end
    smoothness = smoothness / max(1, n-2);
end

%% 导出结果到Excel
function export_results_to_excel(results, stats, algorithms, direct_distance, save_path)
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename  = fullfile(save_path, sprintf('UAV_Path_Planning_Results_%s.xlsx', timestamp));

    % Sheet1：算法性能对比
    data1 = {'Algorithm','Best Path Length (m)','Average Path Length (m)', ...
             'Path Length Std Dev','Best Fitness','Avg Fitness','Fitness Std Dev', ...
             'Path Efficiency (%)','Smoothness','Computation Time (s)', ...
             'Time Std Dev','Success Rate (%)'};

    for i = 1:length(algorithms)
        alg = algorithms{i};
        if isfield(stats, alg)
            row = i + 1;
            data1{row,1}  = alg;
            data1{row,2}  = stats.(alg).best_path_length;
            data1{row,3}  = stats.(alg).avg_path_length;
            data1{row,4}  = stats.(alg).std_path_length;
            data1{row,5}  = stats.(alg).best;
            data1{row,6}  = stats.(alg).mean;
            data1{row,7}  = stats.(alg).std;
            data1{row,8}  = direct_distance / stats.(alg).path_length * 100;
            data1{row,9}  = stats.(alg).smoothness;
            data1{row,10} = stats.(alg).avg_time;
            data1{row,11} = stats.(alg).std_time;
            data1{row,12} = stats.(alg).success_rate;
        end
    end

    % Sheet2：收敛数据
    Max_iter_val = size(results.(algorithms{1}).convergence, 2);
    data2        = cell(floor(Max_iter_val/10) + 2, length(algorithms) + 1);
    data2{1,1}   = 'Iteration';
    for i = 1:length(algorithms)
        data2{1,i+1} = algorithms{i};
    end

    row = 2;
    for iter = 1:10:Max_iter_val
        data2{row,1} = iter;
        for i = 1:length(algorithms)
            alg        = algorithms{i};
            valid_runs = results.(alg).valid_runs;
            if ~isempty(valid_runs)
                mean_curve     = mean(results.(alg).convergence(valid_runs, :), 1);
                data2{row,i+1} = mean_curve(iter);
            else
                data2{row,i+1} = NaN;
            end
        end
        row = row + 1;
    end

    try
        writecell(data1, filename, 'Sheet', 'Performance Comparison');
        writecell(data2, filename, 'Sheet', 'Convergence Data');
        fprintf('\nResults exported to Excel: %s\n', filename);
    catch ME
        fprintf('\nWarning: Excel export failed (%s), saving as CSV\n', ME.message);
        csv_file = fullfile(save_path, 'UAV_Performance_Results.csv');
        writecell(data1, csv_file);
        fprintf('CSV saved: %s\n', csv_file);
    end
end

%% 绘制3D路径对比图
function plot_3d_path_comparison(results, stats, algorithms, terrain_params, ...
        obstacle_params, start_point, end_point, save_path)

    [X, Y] = meshgrid(0:2:200, 0:2:200);
    Z = zeros(size(X));
    for i = 1:numel(X)
        Z(i) = calculate_threat_terrain_height(X(i), Y(i), terrain_params);
    end

    best_paths = cell(length(algorithms), 1);
    for i = 1:length(algorithms)
        alg = algorithms{i};
        if isfield(stats, alg)
            best_paths{i} = results.(alg).final_paths{stats.(alg).best_idx};
        end
    end

    fig = figure('Name', '3D Path Planning Comparison - SSA vs ISSA', 'Position', [50, 50, 1200, 900]);
    surf(X, Y, Z, 'EdgeColor', 'none', 'FaceAlpha', 0.85, 'HandleVisibility', 'off');
    colormap(hot);
    hold on;

    for i = 1:obstacle_params.num
        [cx, cy, cz] = cylinder(obstacle_params.r_O(i), 30);
        cx = cx + obstacle_params.x_O(i);
        cy = cy + obstacle_params.y_O(i);
        cz = cz * obstacle_params.h_O(i);
        surf(cx, cy, cz, 'FaceColor', [0.2 0.2 0.2], 'FaceAlpha', 0.9, ...
             'EdgeColor', 'none', 'HandleVisibility', 'off');
    end

    colors      = {'b', 'r'};
    line_styles = {'-', '--'};
    line_widths = [3, 3.5];

    plot3(start_point(1), start_point(2), start_point(3), 'go', 'MarkerSize', 15, ...
          'MarkerFaceColor', 'g', 'DisplayName', 'Start', 'LineWidth', 3);
    plot3(end_point(1), end_point(2), end_point(3), 'ro', 'MarkerSize', 15, ...
          'MarkerFaceColor', 'r', 'DisplayName', 'Target', 'LineWidth', 3);

    for i = 1:length(algorithms)
        if ~isempty(best_paths{i})
            plot3(best_paths{i}(:,1), best_paths{i}(:,2), best_paths{i}(:,3), ...
                  [colors{i}, line_styles{i}], 'LineWidth', line_widths(i), ...
                  'DisplayName', algorithms{i});
        end
    end

    xlabel('X/m','FontSize',14,'FontWeight','bold');
    ylabel('Y/m','FontSize',14,'FontWeight','bold');
    zlabel('Height/m','FontSize',14,'FontWeight','bold');
    title('UAV 3D Path Planning: SSA vs ISSA','FontSize',16,'FontWeight','bold');
    legend('Location','best','FontSize',12);
    grid on;
    view(45, 30);
    zlim([0, 120]);
    lighting gouraud;
    light('Position',[100,100,200],'Style','local');

    saveas(fig, fullfile(save_path, '3D_Path_Planning_SSA_ISSA.fig'));
    print(fig, fullfile(save_path, '3D_Path_Planning_SSA_ISSA.emf'), '-dmeta');
    fprintf('3D path planning comparison figure saved\n');
end

%% 绘制俯视图对比
function plot_top_view_comparison(results, stats, algorithms, terrain_params, ...
        obstacle_params, start_point, end_point, save_path)

    [X, Y] = meshgrid(0:2:200, 0:2:200);
    Z = zeros(size(X));
    for i = 1:numel(X)
        Z(i) = calculate_threat_terrain_height(X(i), Y(i), terrain_params);
    end

    best_paths = cell(length(algorithms), 1);
    for i = 1:length(algorithms)
        alg = algorithms{i};
        if isfield(stats, alg)
            best_paths{i} = results.(alg).final_paths{stats.(alg).best_idx};
        end
    end

    fig = figure('Name','Top View Path Comparison - SSA vs ISSA','Position',[100,50,1200,900]);
    contourf(X, Y, Z, 20, 'HandleVisibility', 'off');
    colormap(jet);
    colorbar('FontSize',12);
    hold on;

    for i = 1:obstacle_params.num
        theta    = linspace(0, 2*pi, 100);
        x_circle = obstacle_params.x_O(i) + obstacle_params.r_O(i) * cos(theta);
        y_circle = obstacle_params.y_O(i) + obstacle_params.r_O(i) * sin(theta);
        fill(x_circle, y_circle, [0.2 0.2 0.2], 'FaceAlpha', 0.8, ...
             'EdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end

    colors      = {'b', 'r'};
    line_styles = {'-', '--'};

    plot(start_point(1), start_point(2), 'go', 'MarkerSize', 15, ...
         'MarkerFaceColor', 'g', 'DisplayName', 'Start', 'LineWidth', 3);
    plot(end_point(1), end_point(2), 'ro', 'MarkerSize', 15, ...
         'MarkerFaceColor', 'r', 'DisplayName', 'Target', 'LineWidth', 3);

    for i = 1:length(algorithms)
        if ~isempty(best_paths{i})
            plot(best_paths{i}(:,1), best_paths{i}(:,2), ...
                 [colors{i}, line_styles{i}], 'LineWidth', 3, 'DisplayName', algorithms{i});
        end
    end

    xlabel('X/m','FontSize',14,'FontWeight','bold');
    ylabel('Y/m','FontSize',14,'FontWeight','bold');
    title('Top View Path Comparison (XY Plane): SSA vs ISSA','FontSize',16,'FontWeight','bold');
    legend('Location','best','FontSize',12);
    grid on;
    axis equal;
    xlim([0, 200]);
    ylim([0, 200]);

    saveas(fig, fullfile(save_path,'Top_View_SSA_ISSA.fig'));
    print(fig, fullfile(save_path,'Top_View_SSA_ISSA.emf'),'-dmeta');
    fprintf('Top view path comparison figure saved\n');
end

%% 绘制收敛曲线对比
function plot_convergence_comparison(results, algorithms, save_path)
    fig = figure('Name','Convergence Curve Comparison - SSA vs ISSA','Position',[150,50,1200,900]);

    colors      = {'b', 'r'};
    line_styles = {'-', '--'};

    for i = 1:length(algorithms)
        alg = algorithms{i};
        if isfield(results, alg) && ~isempty(results.(alg).valid_runs)
            valid_runs = results.(alg).valid_runs;
            mean_curve = mean(results.(alg).convergence(valid_runs, :), 1);
            plot(mean_curve, [colors{i}, line_styles{i}], 'LineWidth', 3, 'DisplayName', alg);
            hold on;
        end
    end

    xlabel('Iteration','FontSize',14,'FontWeight','bold');
    ylabel('Fitness Value','FontSize',14,'FontWeight','bold');
    title('Convergence Curve Comparison: SSA vs ISSA','FontSize',16,'FontWeight','bold');
    legend('Location','northeast','FontSize',12);
    grid on;
    set(gca,'YScale','log');
    set(gca,'FontSize',12);

    saveas(fig, fullfile(save_path,'Convergence_Curve_SSA_ISSA.fig'));
    print(fig, fullfile(save_path,'Convergence_Curve_SSA_ISSA.emf'),'-dmeta');
    fprintf('Convergence curve comparison figure saved\n');
end

%% 绘制箱线图对比（路径长度分布）
function plot_boxplot_comparison(results, algorithms, save_path)
    fig = figure('Name','Path Length Box Plot - SSA vs ISSA','Position',[200,50,1200,900]);

    n_alg    = length(algorithms);
    num_runs = 0;
    for i = 1:n_alg
        alg = algorithms{i};
        if isfield(results, alg)
            num_runs = max(num_runs, length(results.(alg).path_lengths));
        end
    end

    data_matrix  = NaN(num_runs, n_alg);
    valid_labels = {};
    valid_colors = [];

    colors_rgb = [
        0, 0, 1;
        1, 0, 0;
    ];

    col = 0;
    for i = 1:n_alg
        alg = algorithms{i};
        if isfield(results, alg)
            valid_len = results.(alg).path_lengths(results.(alg).path_lengths > 0);
            if ~isempty(valid_len)
                col = col + 1;
                data_matrix(1:length(valid_len), col) = valid_len(:);
                valid_labels{col}   = alg;
                valid_colors(col,:) = colors_rgb(i,:);
            end
        end
    end

    if col == 0
        fprintf('Warning: No valid data for box plot\n');
        return;
    end

    data_matrix = data_matrix(:, 1:col);

    boxplot(data_matrix, ...
            'Labels',  valid_labels, ...
            'Colors',  'k',          ...
            'Symbol',  '+',          ...
            'Widths',  0.6);

    box_handles = findobj(gca, 'Tag', 'Box');
    for i = 1:length(box_handles)
        color_idx = col - i + 1;
        if color_idx >= 1 && color_idx <= size(valid_colors, 1)
            patch(get(box_handles(i),'XData'), get(box_handles(i),'YData'), ...
                  valid_colors(color_idx,:), 'FaceAlpha', 0.6, ...
                  'EdgeColor', 'k', 'LineWidth', 1.5);
        end
    end

    set(findobj(gca,'Tag','Median'),   'Color','k','LineWidth',2);
    set(findobj(gca,'Tag','Whisker'),  'Color','k','LineWidth',1.5);
    set(findobj(gca,'Tag','Outliers'), 'MarkerEdgeColor','k','MarkerSize',8,'LineWidth',1.5);

    ylabel('Path Length (m)','FontSize',14,'FontWeight','bold');
    title('Path Length Distribution: SSA vs ISSA','FontSize',16,'FontWeight','bold');
    grid on;
    set(gca,'FontSize',12,'LineWidth',1.2);

    all_valid = data_matrix(~isnan(data_matrix));
    if ~isempty(all_valid)
        ylim([min(all_valid) - 10, max(all_valid) + 10]);
    end

    saveas(fig, fullfile(save_path,'Path_Length_BoxPlot_SSA_ISSA.fig'));
    print(fig, fullfile(save_path,'Path_Length_BoxPlot_SSA_ISSA.emf'),'-dmeta');
    fprintf('Box plot comparison figure saved\n');
end
