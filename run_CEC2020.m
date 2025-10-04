clc;
clear;
close all;

% 参数设置
N = 50; % 种群数
Max_iter = 2000; % 最大迭代次数
dim = 20; % 维度
num_functions = 10; % 测试函数数量
maxiter = 50; % 每个函数运行次数

% 定义算法标签、颜色和标记
algorithm_labels = {'SCA', 'OOA', 'GSA', 'SSA', 'CDCSSA', 'ESSA', 'PWSSA', 'ISSA'};
colors = {'#b0d992', '#99b9e9', '#f7df87', '#54beaa', '#05B9E2', '#eca680', '#fccccb', '#e3716e'};
marker_types = {'s', 'o', '^', '*', '<', 'h', 'd', 'v'};

% 确保算法数量与颜色和标记匹配
assert(numel(algorithm_labels) == numel(colors), 'Number of algorithms does not match the number of colors and markers.');

for j = 1:num_functions
    for p = 1:maxiter
        Function_name = ['F', num2str(j)]; % 测试函数编号
        [lb, ub, dim, fobj] = Get_Functions_cec2020(j, dim); % 获取目标函数对应参数
        fprintf('\nExecuting Function F%d, Iteration %d:\n', j, p);
        
        Optimal_results = cell(5, numel(algorithm_labels)); % 初始化结果存储
        
        % 调用各个算法
        algorithms = {@SCA, @OOA, @GSA, @SSA, @CDCSSA, @ESSA, @PWSSA, @ISSA};
        for k = 1:numel(algorithms)
            tic
            [best_fitness, best_position, convergence_curve] = feval(algorithms{k}, N, Max_iter, lb, ub, dim, fobj);
            Optimal_results{1, k} = algorithm_labels{k}; % 算法名称
            Optimal_results{2, k} = convergence_curve; % 收敛曲线
            Optimal_results{3, k} = best_fitness; % 最佳适应度
            Optimal_results{4, k} = best_position; % 最佳位置
            Optimal_results{5, k} = toc; % 运行时间
        end
        
        % 将结果写入Excel
        excel_file = 'D:\研究生\Algorithm\赵涛\ISSA2\程序\结果\2020\表\收敛表.xlsx';
        sheet = num2str(j);
        for k = 1:numel(algorithm_labels)
            xlswrite(excel_file, Optimal_results{3, k}, sheet, [char('A'+k-1), num2str(p)]);
        end
    end
    
    % 绘制并保存收敛曲线
    figure;
    hold on;
    for k = 1:numel(algorithm_labels)
        convergence_curve = log10(Optimal_results{2, k});
        plot(convergence_curve, 'Linewidth', 2, 'Color', colors{k}, ...
             'Marker', marker_types{k}, 'MarkerIndices', 1:200:length(convergence_curve), ...
             'MarkerEdgeColor', colors{k}, 'MarkerFaceColor', colors{k}, 'MarkerSize', 5);
    end
    title(['CEC2020-' Function_name]);
    xlabel('Iteration');
    ylabel('Fitness');
    axis tight;
    box on;
    legend(algorithm_labels, 'Location', 'best');
    hold off;
    
    % 保存图像
    baseFolder = 'D:\研究生\Algorithm\赵涛\ISSA2\程序\结果\2020\图\收敛图循环\';
    figFilename = fullfile(baseFolder, sprintf('F%d.fig', j));
    pngFilename = fullfile(baseFolder, sprintf('F%d.png', j));
    saveas(gcf, figFilename); % 保存为.fig文件
    print(gcf, pngFilename, '-dpng'); % 保存为PNG文件
end