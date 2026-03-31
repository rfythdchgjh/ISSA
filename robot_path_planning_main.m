function robot_path_planning_main()
% =========================================================================
%  Robot Path Planning — SSA vs ISSA (完整修复版 v2)

clc; clear; close all;

%% ===================================================================
%  ██████ CONFIG — 按需修改此节 ██████████████████████████████████████
%% ===================================================================

% ---- 结果保存路径 ----
RESULT_DIR = '.\PathPlanning_Results';

% ---- 代价模式 ----
COST_MODE_CFG = 'euclid';

% ---- 重复实验次数 ----
N_RUNS = 10;

% ---- 地图选择 ----
%   'builtin_20x20' : 内置 20×20 地图
%   'builtin_30x30' : 内置 30×30 地图
%   'custom'        : 使用下方 CUSTOM_MAP 手动定义
MAP_CHOICE = 'builtin_20x20';

% ---- 自定义地图（MAP_CHOICE='custom' 时生效）----
%   0=自由, 1=障碍
CUSTOM_MAP = [
    0 0 0 0 0 0 0 0 0 0
    0 1 1 0 0 1 1 0 0 0
    0 1 1 0 0 1 1 0 0 0
    0 0 0 0 0 0 0 0 1 0
    0 0 1 1 1 0 0 0 1 0
    0 0 1 1 1 0 0 0 0 0
    0 0 0 0 0 0 1 1 0 0
    0 0 0 1 0 0 1 1 0 0
    0 0 0 1 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
];

% ---- 起点 / 终点  [行, 列] ----
START_PT = [18, 3];
GOAL_PT  = [2, 18];

% ---- 优化器参数 ----
OPT.num_waypoints = 4;
OPT.iter_max      = 200;
OPT.popsize       = 30;

% ---- 是否启用交互式障碍物编辑 ----
%   true  = 运行前弹出图形界面，可点击添加/删除障碍物
%   false = 直接使用上方配置的地图
ENABLE_MAP_EDITOR = true;

%% ===================================================================
%  初始化全局变量
%% ===================================================================
global S E map_global COST_MODE
COST_MODE = COST_MODE_CFG;

fprintf('=========================================\n');
fprintf('=== Robot Path Planning (SSA vs ISSA) ===\n');
fprintf('=========================================\n');

%% ===================================================================
%  加载地图
%% ===================================================================
switch MAP_CHOICE
    case 'builtin_20x20'
        map = load_builtin_map_20x20();
    case 'builtin_30x30'
        map = load_builtin_map_30x30();
    case 'custom'
        map = double(logical(CUSTOM_MAP));
    otherwise
        error('MAP_CHOICE 未知: %s', MAP_CHOICE);
end

%% ===================================================================
%  [FF] 交互式障碍物编辑
%% ===================================================================
if ENABLE_MAP_EDITOR
    fprintf('\n打开交互式地图编辑器...\n');
    fprintf('  操作说明: 左键点击格子 = 切换障碍/自由\n');
    fprintf('  完成后关闭窗口（或按任意键）继续运行\n');
    [map, START_PT, GOAL_PT] = interactive_map_editor(map, START_PT, GOAL_PT);
    fprintf('编辑完成，继续规划...\n\n');
end

map_global = map;
[nRows, nCols] = size(map);
fprintf('地图: %dx%d (%s)\n', nRows, nCols, MAP_CHOICE);

%% ===================================================================
%  设置起终点并验证
%% ===================================================================
S = START_PT;
E = GOAL_PT;
validate_points(S, E, map, nRows, nCols);
fprintf('起点=[%d,%d]  终点=[%d,%d]\n', S(1),S(2), E(1),E(2));

%% ===================================================================
%  优化参数 [FA: 搜索范围扩展至全图]
%% ===================================================================
params = build_params(OPT, S, E, map_global);
fprintf('路径点=%d  维数=%d  迭代=%d  种群=%d\n', ...
    params.num_waypoints, params.dimensions, params.iter_max, params.popsize);
fprintf('行范围[%.1f,%.1f]  列范围[%.1f,%.1f]\n', ...
    params.x_min, params.x_max, params.y_min, params.y_max);

%% ===================================================================
%  连通性校验（用 A* 确认起终点可达，不参与对比展示）
%% ===================================================================
fprintf('\n验证起终点连通性...\n');
path_check = astar_pathfinding(S, E, map);
if size(path_check, 1) <= 1
    error('起点到终点不可达，请检查地图和起终点设置。');
end
fprintf('连通性验证通过。\n');

%% ===================================================================
%  单次运行（SSA vs ISSA）
%% ===================================================================
rng(42);
fprintf('\n--- 单次运行（固定种子=42）---\n');

[cv_ssa,  pos_ssa,  t_ssa]  = run_SSA(params);
[cv_issa, pos_issa, t_issa] = run_ISSA(params);

cv_ssa  = sanitize_curve(cv_ssa,  params.iter_max);
cv_issa = sanitize_curve(cv_issa, params.iter_max);

route_ssa  = reconstruct_path(pos_ssa,  params.num_waypoints);
route_issa = reconstruct_path(pos_issa, params.num_waypoints);

len_ssa  = compute_path_length(route_ssa,  COST_MODE);
len_issa = compute_path_length(route_issa, COST_MODE);

fprintf('SSA  路径长度=%.4f  用时=%.4fs\n', len_ssa,  t_ssa);
fprintf('ISSA 路径长度=%.4f  用时=%.4fs\n', len_issa, t_issa);

% 验证路径是否无碰撞
check_collision(route_ssa,  map, 'SSA');
check_collision(route_issa, map, 'ISSA');

%% ===================================================================
%  保存目录
%% ===================================================================
if ~exist(RESULT_DIR,'dir'), mkdir(RESULT_DIR); end

%% ===================================================================
%  绘图（仅 SSA vs ISSA）
%% ===================================================================
fig_conv = plot_convergence({cv_ssa, cv_issa}, {'SSA', 'ISSA'});
savefig(fig_conv, fullfile(RESULT_DIR, 'single_convergence.fig'));
print(fig_conv, fullfile(RESULT_DIR, 'single_convergence.png'), '-dpng', '-r200');

fig_path = plot_paths({route_ssa, route_issa}, {'SSA', 'ISSA'}, map, S, E);
savefig(fig_path, fullfile(RESULT_DIR, 'single_paths.fig'));
print(fig_path, fullfile(RESULT_DIR, 'single_paths.png'), '-dpng', '-r200');

%% ===================================================================
%  单次结果保存 Excel
%% ===================================================================
T1 = table( ...
    {'SSA';'ISSA'}, ...
    [len_ssa; len_issa], ...
    [t_ssa;   t_issa], ...
    'VariableNames', {'Algorithm','PathLength','Time_s'});
writetable(T1, fullfile(RESULT_DIR,'single_run.xlsx'), 'Sheet','Single');
fprintf('单次结果已写入 single_run.xlsx\n');

%% ===================================================================
%  重复实验
%% ===================================================================
rng('shuffle');
fprintf('\n--- 重复实验 N=%d ---\n', N_RUNS);
run_repeated_experiments(N_RUNS, params, map, S, E, COST_MODE, RESULT_DIR);

fprintf('\n全部完成。结果已保存到: %s\n', RESULT_DIR);
end

%% ===================================================================
%% 算法调用封装
%% ===================================================================

function [curve, pos, elapsed] = run_SSA(params)
% [对称修复] 改为调用独立 SSA.m，与 run_ISSA 完全对称，确保对比公平性。
% 原来内嵌的 SSA 实现在领导者公式上去掉了 lb(j) 偏移项（注释为[FD]修复），
% 而 ISSA.m 保留了该项，导致两者领导者公式不一致，对比失去意义。
% SSA.m 使用原始论文标准公式（保留 lb(j)），与 ISSA.m 的领导者部分完全一致，
% 差异仅体现在：初始化方式、跟随者更新策略、局部优化三个方面。
tic;
lb     = build_lb(params);
ub     = build_ub(params);
dim    = params.dimensions;
N      = params.popsize;
T      = params.iter_max;
num_wp = params.num_waypoints;
fobj   = @(x) fitness(x, num_wp);

[~, FoodPosition, Convergence_curve] = SSA(N, T, lb, ub, dim, fobj);

pos     = FoodPosition;
curve   = Convergence_curve;
elapsed = toc;
fprintf('SSA 完成，用时=%.4fs\n', elapsed);
end

% -------------------------------------------------------------------
function [curve, pos, elapsed] = run_ISSA(params)
% [F2] 直接调用 ISSA.m（标准 ISSA 算法）
tic;
lb  = build_lb(params);
ub  = build_ub(params);
dim = params.dimensions;
N   = params.popsize;
T   = params.iter_max;
num_wp = params.num_waypoints;
fobj = @(x) fitness(x, num_wp);

[~, FoodPosition, Convergence_curve] = ISSA(N, T, lb, ub, dim, fobj);

pos    = FoodPosition;
curve  = Convergence_curve;
elapsed = toc;
fprintf('ISSA 完成，用时=%.4fs\n', elapsed);
end

%% ===================================================================
%% 适应度函数 [FA][FB]
%% ===================================================================
function fx = fitness(wcoords, num_wp)
% [FA] 搜索范围已扩展至全图，waypoint可在任意自由格
% [FB] 增加额外惩罚：若waypoint落在障碍物且吸附代价过高则惩罚
global S E map_global COST_MODE

PENALTY = 1e6;

if numel(wcoords) ~= num_wp*2
    fx = PENALTY; return;
end

wps = S;
for i = 1:num_wp
    r = round(wcoords(2*i-1));
    c = round(wcoords(2*i));
    r = max(1, min(size(map_global,1), r));
    c = max(1, min(size(map_global,2), c));
    if map_global(r,c) == 1
        [r,c] = find_nearest_free([r,c], map_global);
        if map_global(r,c) == 1
            fx = PENALTY; return;
        end
    end
    wps = [wps; r, c]; %#ok<AGROW>
end
wps = [wps; E];

% 单调排序防止回跑
wps = sort_waypoints(wps, S, E);

total = 0;
for i = 1:size(wps,1)-1
    seg = astar_pathfinding(wps(i,:), wps(i+1,:), map_global);
    if size(seg,1) < 2
        fx = PENALTY; return;
    end
    % [FB] 检查路径段是否穿越障碍（理论上A*不会，但检测异常情况）
    for k = 1:size(seg,1)
        if map_global(seg(k,1), seg(k,2)) == 1
            fx = PENALTY; return;
        end
    end
    total = total + segment_length(seg, COST_MODE);
end
fx = total;
end

% -------------------------------------------------------------------
function wps = sort_waypoints(wps, S, E)
if size(wps,1) <= 2, return; end
mid  = wps(2:end-1, :);
dir  = E - S;
ndir = norm(dir);
if ndir < 1e-9, return; end
proj = (mid - S) * dir' / ndir;
[~, idx] = sort(proj);
wps = [S; mid(idx,:); E];
end

%% ===================================================================
%% 路径重建 [FE]
%% ===================================================================
function path = reconstruct_path(wcoords, num_wp)
global S E map_global

wps = S;
for i = 1:num_wp
    r = round(wcoords(2*i-1));
    c = round(wcoords(2*i));
    r = max(1, min(size(map_global,1), r));
    c = max(1, min(size(map_global,2), c));
    if map_global(r,c) == 1
        [r,c] = find_nearest_free([r,c], map_global);
    end
    wps = [wps; r, c]; %#ok<AGROW>
end
wps = [wps; E];
wps = sort_waypoints(wps, S, E);

path = wps(1,:);
for i = 1:size(wps,1)-1
    seg = astar_pathfinding(wps(i,:), wps(i+1,:), map_global);
    if size(seg,1) > 1
        % [FE] 不使用 unique()，直接拼接，保留所有路径点（防止绕障路径被截断）
        path = [path; seg(2:end,:)]; %#ok<AGROW>
    end
end
end

%% ===================================================================
%% 碰撞验证
%% ===================================================================
function check_collision(path, map, name)
if size(path,1) < 2
    fprintf('  [警告] %s 路径节点数不足\n', name);
    return;
end
hit = 0;
for k = 1:size(path,1)
    r = path(k,1); c = path(k,2);
    if r>=1 && r<=size(map,1) && c>=1 && c<=size(map,2)
        if map(r,c) == 1
            hit = hit + 1;
        end
    end
end
if hit > 0
    fprintf('  [碰撞警告] %s 路径中有 %d 个节点位于障碍物！\n', name, hit);
else
    fprintf('  [✓] %s 路径无碰撞\n', name);
end
end

%% ===================================================================
%% 重复实验
%% ===================================================================
function run_repeated_experiments(N, params, map, S_pt, E_pt, cost_mode, RESULT_DIR)
algos = {'SSA','ISSA'};
M  = 2;
it = params.iter_max;

final_best = nan(N, M);
run_times  = nan(N, M);
curves_all = nan(M, it, N);

for r = 1:N
    rng('shuffle');
    [c1, ~, t1] = run_SSA(params);
    [c2, ~, t2] = run_ISSA(params);

    C = {sanitize_curve(c1,it), sanitize_curve(c2,it)};
    for k = 1:M
        curves_all(k,:,r) = C{k};
        final_best(r,k)   = C{k}(end);
    end
    run_times(r,:) = [t1, t2];
    fprintf('Run %02d/%02d: SSA=%.2f  ISSA=%.2f\n', r, N, final_best(r,1), final_best(r,2));
end

bestVal  = min(final_best,[],1);
meanVal  = mean(final_best,1);
stdVal   = std(final_best,0,1);
meanTime = mean(run_times,1);
map_str  = sprintf('%dx%d', size(map,1), size(map,2));

fprintf('\n=== 重复实验统计（SSA vs ISSA）===\n');
for k = 1:M
    fprintf('%s: Best=%.2f  Mean=%.2f  Std=%.2f  AvgTime=%.3fs\n', ...
            algos{k}, bestVal(k), meanVal(k), stdVal(k), meanTime(k));
end

T = table(repmat(map_str,M,1), algos(:), bestVal(:), meanVal(:), stdVal(:), meanTime(:), ...
    'VariableNames',{'MapSize','Algorithm','Best','Mean','Std','AvgTime_s'});
writetable(T, fullfile(RESULT_DIR,'repeated_results.xlsx'), 'Sheet','Stats');

mu = zeros(M,it);
for k = 1:M
    mu(k,:) = mean(squeeze(curves_all(k,:,:)), 2, 'omitnan')';
end
fig = plot_convergence({mu(1,:), mu(2,:)}, {'SSA(均值)', 'ISSA(均值)'});
title(gca, sprintf('平均收敛曲线 (%d次实验)', N));
savefig(fig, fullfile(RESULT_DIR,'avg_convergence.fig'));
print(fig, fullfile(RESULT_DIR,'avg_convergence.png'), '-dpng', '-r200');
fprintf('重复实验结果已保存。\n');
end

%% ===================================================================
%% 参数构建 [FA: 搜索范围扩展至全图]
%% ===================================================================
function params = build_params(OPT, S, E, map_global)
params.num_waypoints = OPT.num_waypoints;
params.dimensions    = OPT.num_waypoints * 2;
params.iter_max      = OPT.iter_max;
params.popsize       = OPT.popsize;

% [FA] 关键修复：搜索范围设为整张地图，不再限制在S→E连线附近
% 原代码将搜索限制在 R=max(4, 20%*dist) 的狭窄带状区域，导致障碍物在带外时无法绕行
params.x_min = 1;
params.x_max = size(map_global, 1);
params.y_min = 1;
params.y_max = size(map_global, 2);
end

function lb = build_lb(params)
lb = zeros(1, params.dimensions);
for j = 1:params.dimensions
    if mod(j,2)==1, lb(j)=params.x_min; else, lb(j)=params.y_min; end
end
end

function ub = build_ub(params)
ub = zeros(1, params.dimensions);
for j = 1:params.dimensions
    if mod(j,2)==1, ub(j)=params.x_max; else, ub(j)=params.y_max; end
end
end

%% ===================================================================
%% 可视化 [FC: 修正坐标系]
%% ===================================================================
function fig = plot_convergence(curves, names)
% SSA vs ISSA 收敛曲线对比（已移除 A* 参考线）
fig = figure('Position',[60 60 960 560],'Name','SSA vs ISSA 收敛曲线');
styles = {'-', '--', '-.', ':'};
colors = {[0 114 189]/255, [217 83 25]/255, [119 172 48]/255, [126 47 142]/255};

leg_h = []; leg_names = {};
for i = 1:numel(curves)
    c = curves{i};
    if isempty(c), continue; end
    c = sanitize_curve(c(:)', numel(c));
    h = plot(1:numel(c), c, styles{i}, 'Color', colors{i}, 'LineWidth', 2.5);
    hold on;
    leg_h(end+1) = h; %#ok<AGROW>
    leg_names{end+1} = names{i}; %#ok<AGROW>
end

xlabel('迭代次数','FontSize',13); ylabel('路径长度','FontSize',13);
title('SSA vs ISSA 收敛曲线对比','FontSize',14);
legend(leg_h, leg_names, 'Location','northeast','FontSize',11);
grid on; box on;
end

% -------------------------------------------------------------------
function fig = plot_paths(routes, names, map, S, E)
% [FC] 坐标系修正：
%   不对 map 做 flipud，直接使用原始矩阵坐标系绘制
%   路径 [row,col] → 显示 (x=col-0.5, y=row-0.5)，两者完全一致
%   视觉效果：row=1在屏幕底部（Y轴最小值），row=nR在屏幕顶部（Y轴最大值）
[nR, nC] = size(map);

fig = figure('Position',[1080 60 960 700],'Name','路径对比');
hold on;

% 绘制地图格子（使用原始 map，不 flipud）
% 显示时 y 轴从下到上，row=1 对应 y in [0,1]，即屏幕底部
for i = 1:nR
    for j = 1:nC
        x = [j-1, j, j, j-1];
        % [FC] 直接映射：行i → y区间 [i-1, i]
        y = [i-1, i-1, i, i];
        if map(i,j)==1
            fill(x, y, [77 144 138]/255, 'EdgeColor','none');
        else
            fill(x, y, [233 248 246]/255, 'EdgeColor',[194 222 219]/255, 'EdgeAlpha',0.8);
        end
    end
end

styles = {'-','--','-.',':','-'};
colors = {[0 114 189]/255, [217 83 25]/255, [119 172 48]/255, ...
          [126 47 142]/255, [77 190 238]/255};

leg_h = []; leg_names = {};
for k = 1:numel(routes)
    R = routes{k};
    if size(R,1) < 2
        h = plot(nan, nan, styles{k}, 'Color', colors{k}, 'LineWidth',2.5);
    else
        % [FC] 路径 row→y, col→x，与地图格子绘制完全一致
        px = R(:,2) - 0.5;
        py = R(:,1) - 0.5;
        h = plot(px, py, styles{k}, 'Color', colors{k}, 'LineWidth',2.5);
    end
    leg_h(end+1) = h; %#ok<AGROW>
    leg_names{end+1} = names{k}; %#ok<AGROW>
end

% 起点/终点
hS = plot(S(2)-0.5, S(1)-0.5, 'o', 'Color',[0 0.8 0], ...
          'MarkerFaceColor',[0 0.8 0], 'MarkerSize',11, 'LineWidth',1.5);
hE = plot(E(2)-0.5, E(1)-0.5, 's', 'Color',[0.9 0 0], ...
          'MarkerFaceColor',[0.9 0 0], 'MarkerSize',11, 'LineWidth',1.5);
text(S(2)-0.5+0.3, S(1)-0.5+0.3, 'S', 'FontSize',11,'FontWeight','bold','Color','k');
text(E(2)-0.5+0.3, E(1)-0.5+0.3, 'G', 'FontSize',11,'FontWeight','bold','Color','k');

xlabel('列 (X)','FontSize',13); ylabel('行 (Y)','FontSize',13);
xlim([0 nC]); ylim([0 nR]);
title('SSA vs ISSA 路径规划结果对比','FontSize',14);
legend([leg_h, hS, hE], [leg_names, {'起点(S)','终点(G)'}], 'Location','best','FontSize',11);
grid on; box on;
end

%% ===================================================================
%% 工具函数
%% ===================================================================
function L = compute_path_length(path, mode)
L = 0;
for i = 1:size(path,1)-1
    dr = abs(path(i+1,1)-path(i,1));
    dc = abs(path(i+1,2)-path(i,2));
    if strcmpi(mode,'hop')
        L = L + 1;
    else
        if dr==0 || dc==0, L = L + 1; else, L = L + sqrt(2); end
    end
end
end

function L = segment_length(seg, mode)
L = 0;
for i = 1:size(seg,1)-1
    dr = abs(seg(i+1,1)-seg(i,1));
    dc = abs(seg(i+1,2)-seg(i,2));
    if strcmpi(mode,'hop')
        L = L + 1;
    else
        if dr==0 || dc==0, L = L+1; else, L = L+sqrt(2); end
    end
end
end

function [fr,fc] = find_nearest_free(pt, map)
[nR,nC] = size(map); min_d = inf; fr = pt(1); fc = pt(2);
for R = 1:5
    for r = max(1,pt(1)-R):min(nR,pt(1)+R)
        for c = max(1,pt(2)-R):min(nC,pt(2)+R)
            if map(r,c)==0
                d = hypot(r-pt(1),c-pt(2));
                if d < min_d, min_d=d; fr=r; fc=c; end
            end
        end
    end
    if min_d < inf, break; end
end
end

function c = sanitize_curve(c, it)
if isempty(c), c = nan(1,it); end
c = c(:)';
for k = 1:numel(c)
    if k==1 && (~isfinite(c(k))||c(k)<=0), c(k)=nan; end
    if k>1  && (~isfinite(c(k))||c(k)<=0), c(k)=c(k-1); end
end
if ~isfinite(c(1))
    idx=find(isfinite(c),1,'first');
    c(1) = ~isempty(idx)*c(max(idx,1)) + isempty(idx)*0;
end
if numel(c)<it,    c(end+1:it)=c(end);
elseif numel(c)>it, c=c(1:it); end
last = find(isfinite(c)&c>0, 1,'last');
if ~isempty(last) && (~isfinite(c(end))||c(end)<=0)
    c(end) = c(last);
end
end

function validate_points(S, E, map, nR, nC)
assert(S(1)>=1&&S(1)<=nR && S(2)>=1&&S(2)<=nC, '起点 [%d,%d] 超出地图范围',S(1),S(2));
assert(E(1)>=1&&E(1)<=nR && E(2)>=1&&E(2)<=nC, '终点 [%d,%d] 超出地图范围',E(1),E(2));
assert(map(S(1),S(2))==0, '起点 [%d,%d] 是障碍物！请修改 START_PT',S(1),S(2));
assert(map(E(1),E(2))==0, '终点 [%d,%d] 是障碍物！请修改 GOAL_PT', E(1),E(2));
fprintf('  ✓ 起点[%d,%d] 和终点[%d,%d] 均为自由格\n',S(1),S(2),E(1),E(2));
end

%% ===================================================================
%% 内置地图
%% ===================================================================
function map = load_builtin_map_20x20()
% 0=自由, 1=障碍
% 注意：此处第1行对应屏幕底部（不经过flipud），符合 [row,col] 直觉
map_data = [
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  1 1 1 1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0
  1 1 1 1 0 0 0 0 0 0 1 1 0 0 1 1 1 1 0 0
  1 1 1 1 0 0 0 1 1 0 0 0 0 0 1 1 1 1 0 0
  0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0
  0 0 0 1 1 1 0 0 0 0 0 1 1 0 0 0 0 0 0 0
  0 0 0 1 1 1 0 0 0 0 0 1 1 0 0 1 1 1 0 0
  0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 1 1 0 0
  0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 1 1 1 0 0
  0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0
  1 1 1 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0
  1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0
  0 0 0 0 0 1 1 0 0 1 1 1 1 1 1 1 1 1 0 0
  0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
];
map = double(map_data);
end

function map = load_builtin_map_30x30()
map = zeros(30,30);
obstacles = {[4,5,6,7],[4,5,6,7];
             [10,11,12],[8,9,10,11];
             [15,16,17],[15,16,17,18,19];
             [20,21],[5,6,7,8,9];
             [8,9,10],[20,21,22];
             [25,26,27],[12,13,14,15]};
for k = 1:size(obstacles,1)
    map(obstacles{k,1}, obstacles{k,2}) = 1;
end
end
