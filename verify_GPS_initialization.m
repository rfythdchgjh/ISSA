% =========================================================================
%  plot_initialization_comparison.m
%  SSA随机初始化 vs ISSA GPS正确初始化 — 对比图
%  输出：自动创建"初始化对比图"文件夹，保存 .fig 和 .emf
% =========================================================================

clear; clc; close all;

%% ---- 参数 ----------------------------------------------------------------
N   = 500;
dim = 2;
ub  = 1;
lb  = 0;

%% ---- 创建输出文件夹 -------------------------------------------------------
save_dir = '初始化对比图';
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
    fprintf('已创建文件夹：%s\n', save_dir);
end

%% ---- SSA 随机初始化 -------------------------------------------------------
rng(42);
Pos_SSA = lb + (ub - lb) .* rand(N, dim);

%% ---- ISSA GPS 正确初始化（p > 2*dim+1）-----------------------------------
p = 2 * dim + 2;
while ~isprime(p)
    p = p + 1;
end
i_vec    = (1:N)';
j_vec    = 1:dim;
x_gps    = i_vec * (2 * cos(2 * pi * j_vec / p));
JD       = mod(x_gps, 1);
Pos_ISSA = lb + (ub - lb) .* JD;

%% ---- 通用绘图参数 ---------------------------------------------------------
font_name  = 'Times New Roman';
font_axis  = 10;
font_title = 11;
marker_sz  = 6;
fig_w      = 3.5;
fig_h      = 3.2;

%% =========================================================================
%  Figure 1 — SSA 随机初始化散点图
% =========================================================================
fig1 = figure('Units','inches','Position',[1 1 fig_w fig_h],'Color','w');

scatter(Pos_SSA(:,1), Pos_SSA(:,2), marker_sz, ...
        [0.2 0.4 0.8], 'filled', 'MarkerFaceAlpha', 0.55);
xlabel('$x_1$','Interpreter','latex','FontSize',font_axis,'FontName',font_name);
ylabel('$x_2$','Interpreter','latex','FontSize',font_axis,'FontName',font_name);
title('SSA: Random Initialization', ...
      'FontSize',font_title,'FontName',font_name,'FontWeight','bold');
set(gca,'FontSize',font_axis,'FontName',font_name,'Box','on','LineWidth',0.8,...
        'XLim',[lb ub],'YLim',[lb ub],'XTick',0:0.25:1,'YTick',0:0.25:1);
grid on; axis square;

fname1 = fullfile(save_dir, 'Fig1_SSA_Random_Init');
savefig(fig1, [fname1 '.fig']);
print(fig1,   [fname1 '.emf'], '-dmeta', '-r600');
fprintf('Figure 1 已保存：%s\n', fname1);

%% =========================================================================
%  Figure 2 — ISSA GPS 初始化散点图
% =========================================================================
fig2 = figure('Units','inches','Position',[5 1 fig_w fig_h],'Color','w');

scatter(Pos_ISSA(:,1), Pos_ISSA(:,2), marker_sz, ...
        [0.85 0.33 0.10], 'filled', 'MarkerFaceAlpha', 0.55);
xlabel('$x_1$','Interpreter','latex','FontSize',font_axis,'FontName',font_name);
ylabel('$x_2$','Interpreter','latex','FontSize',font_axis,'FontName',font_name);
title('ISSA: GPS Initialization', ...
      'FontSize',font_title,'FontName',font_name,'FontWeight','bold');
set(gca,'FontSize',font_axis,'FontName',font_name,'Box','on','LineWidth',0.8,...
        'XLim',[lb ub],'YLim',[lb ub],'XTick',0:0.25:1,'YTick',0:0.25:1);
grid on; axis square;

fname2 = fullfile(save_dir, 'Fig2_ISSA_GPS_Init');
savefig(fig2, [fname2 '.fig']);
print(fig2,   [fname2 '.emf'], '-dmeta', '-r600');
fprintf('Figure 2 已保存：%s\n', fname2);

%% =========================================================================
%  Figure 3 — 热力图：SSA 覆盖均匀性
% =========================================================================
n_bins = 10;
edges  = linspace(lb, ub, n_bins+1);
count_SSA = histcounts2(Pos_SSA(:,1), Pos_SSA(:,2), edges, edges);

fig3 = figure('Units','inches','Position',[1 5 fig_w fig_h],'Color','w');
imagesc(edges(1:end-1)+diff(edges(1:2))/2, ...
        edges(1:end-1)+diff(edges(1:2))/2, count_SSA');
set(gca,'YDir','normal','FontSize',font_axis,'FontName',font_name,...
        'Box','on','LineWidth',0.8,'XLim',[lb ub],'YLim',[lb ub]);
colorbar('FontSize',font_axis-1,'FontName',font_name);
colormap('hot'); axis square;
xlabel('$x_1$','Interpreter','latex','FontSize',font_axis,'FontName',font_name);
ylabel('$x_2$','Interpreter','latex','FontSize',font_axis,'FontName',font_name);
title('SSA: Cell Count Heatmap', ...
      'FontSize',font_title,'FontName',font_name,'FontWeight','bold');

fname3 = fullfile(save_dir, 'Fig3_SSA_Heatmap');
savefig(fig3, [fname3 '.fig']);
print(fig3,   [fname3 '.emf'], '-dmeta', '-r600');
fprintf('Figure 3 已保存：%s\n', fname3);

%% =========================================================================
%  Figure 4 — 热力图：ISSA 覆盖均匀性
% =========================================================================
count_ISSA = histcounts2(Pos_ISSA(:,1), Pos_ISSA(:,2), edges, edges);

fig4 = figure('Units','inches','Position',[5 5 fig_w fig_h],'Color','w');
imagesc(edges(1:end-1)+diff(edges(1:2))/2, ...
        edges(1:end-1)+diff(edges(1:2))/2, count_ISSA');
set(gca,'YDir','normal','FontSize',font_axis,'FontName',font_name,...
        'Box','on','LineWidth',0.8,'XLim',[lb ub],'YLim',[lb ub]);
colorbar('FontSize',font_axis-1,'FontName',font_name);
colormap('hot'); axis square;
xlabel('$x_1$','Interpreter','latex','FontSize',font_axis,'FontName',font_name);
ylabel('$x_2$','Interpreter','latex','FontSize',font_axis,'FontName',font_name);
title('ISSA: Cell Count Heatmap', ...
      'FontSize',font_title,'FontName',font_name,'FontWeight','bold');

fname4 = fullfile(save_dir, 'Fig4_ISSA_Heatmap');
savefig(fig4, [fname4 '.fig']);
print(fig4,   [fname4 '.emf'], '-dmeta', '-r600');
fprintf('Figure 4 已保存：%s\n', fname4);

fprintf('\n全部完成，4张图已保存至"%s"文件夹。\n', save_dir);