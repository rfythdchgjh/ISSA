clc
clear
close all

%% 参数设置
N = 50; % 种群数
Max_iter = 2000; % 最大迭代次数
dim = 20; % 维数，可选 2, 5, 10, 15, 20

%% 选择函数
Function_name = 10; % 函数名： 1 - 10
% lb->下限，ub->上限，fobj->目标函数，dim-> 维度
[lb, ub, dim, fobj] = Get_Functions_cec2020(Function_name, dim);

%% 调用算法
Optimal_results = {}; % 保存Optimal results
index = 1;

% SCA
tic
[Destination_fitness, Destination_position, Curve_SCA] = SCA(N, Max_iter, lb, ub, dim, fobj);
Optimal_results{1, index} = "SCA";
Optimal_results{2, index} = Curve_SCA;
Optimal_results{3, index} = Destination_fitness;
Optimal_results{4, index} = Destination_position;
Optimal_results{5, index} = toc;
index = index + 1;


% OOA
tic
[Alpha_score,Alpha_pos,Curve_OOA]=OOA(N, Max_iter,lb,ub,dim,fobj  );
Optimal_results{1,index}="OOA";
Optimal_results{2,index}=Curve_OOA;
Optimal_results{3,index}=Alpha_score;
Optimal_results{4,index}=Alpha_pos;
Optimal_results{5,index}=toc;
index = index +1;

% OOA
tic
[BestScore_GoldSA, BestPos_GoldSA, Curve_GoldSA] = GSA(N, Max_iter, lb, ub, dim, fobj);
Optimal_results{1,index}="GSA";
Optimal_results{2,index}=Curve_GoldSA;
Optimal_results{3,index}=BestScore_GoldSA;
Optimal_results{4,index}=BestPos_GoldSA;
Optimal_results{5,index}=toc;
index = index +1;


% SSA
tic
[FoodFitness, FoodPosition, Curve_SSA] = SSA(N, Max_iter, lb, ub, dim, fobj);
Optimal_results{1, index} = "SSA";
Optimal_results{2, index} = Curve_SSA;
Optimal_results{3, index} = FoodFitness;
Optimal_results{4, index} = FoodPosition;
Optimal_results{5, index} = toc;
index = index + 1;

% CDCSSA
tic
[FoodFitness, FoodPosition, Curve_CDCSSA] = CDCSSA(N, Max_iter, lb, ub, dim, fobj);
Optimal_results{1, index} = "CDCSSA";
Optimal_results{2, index} = Curve_CDCSSA;
Optimal_results{3, index} = FoodFitness;
Optimal_results{4, index} = FoodPosition;
Optimal_results{5, index} = toc;
index = index + 1;

% ESSA
tic
[FoodFitness, FoodPosition, Curve_ESSA] = ESSA(N, Max_iter, lb, ub, dim, fobj);
Optimal_results{1, index} = "ESSA";
Optimal_results{2, index} = Curve_ESSA;
Optimal_results{3, index} = FoodFitness;
Optimal_results{4, index} = FoodPosition;
Optimal_results{5, index} = toc;
index = index + 1;

% PWSSA
tic
[FoodFitness, FoodPosition, Curve_PWSSA] = PWSSA(N, Max_iter, lb, ub, dim, fobj);
Optimal_results{1, index} = "PWSSA";
Optimal_results{2, index} = Curve_PWSSA;
Optimal_results{3, index} = FoodFitness;
Optimal_results{4, index} = FoodPosition;
Optimal_results{5, index} = toc;
index = index + 1;
% 
% % ISSA
tic
[IfMin, IbestX, Curve_ISSA] = ISSA(N, Max_iter, lb, ub, dim, fobj);
Optimal_results{1, index} = "ISSA";
Optimal_results{2, index} = Curve_ISSA;
Optimal_results{3, index} = IfMin;
Optimal_results{4, index} = IbestX;
Optimal_results{5, index} = toc;
index = index + 1;

% 初始化一个单元数组来存储达到最优解的迭代次数
iterations = cell(1, size(Optimal_results, 2));

% 循环遍历每个算法的收敛曲线，找出达到最小值的迭代次数
for i = 1:size(Optimal_results, 2)
    convergence_curve = Optimal_results{2, i};
    [~, idx] = min(convergence_curve); % 找出最小值的索引
    iterations{i} = idx; % 存储迭代次数
end

% 创建并显示一个1x10的表格
T = table(Optimal_results(1, :), iterations, 'VariableNames', {'Algorithm', 'Iteration_at_Optimum'});
disp(T);

% 定义8种不同的颜色用于8种不同的算法
colors = {'#b0d992', '#99b9e9', '#f7df87', '#54beaa','#05B9E2','#eca680', '#fccccb', '#e3716e'};% ,'#54beaa', '#eca680'

% 定义标记类型
marker_types = {'s', 'o', '^', '*', '< ' , 'h','d', 'v'};%,'x', '+', 
algorithm_labels = {'SCA', 'OOA', 'GSA', 'SSA','CDCSSA','ESSA','PWSSA','ISSA'};

% 确保Optimal_results结构正确
assert(size(Optimal_results, 2) == length(colors), 'Number of algorithms does not match the number of colors and markers.');

% 绘制收敛曲线
figure
hold on

for i = 1:size(Optimal_results, 2)
    algorithm = Optimal_results{1, i};
    convergence_curve = log10(Optimal_results{2, i});

    % 从颜色列表中获取颜色
    color = colors{i};
    % 从标记类型列表中获取标记
    marker = marker_types{i};

    % 确定标记间隔，例如每20个点标记一次
    marker_spacing = 200;
    marker_indices = 1:marker_spacing:length(convergence_curve);

    % 绘制曲线并指定颜色和标记
    plot(convergence_curve, 'Linewidth', 2, 'Color', color, 'Marker', marker, 'MarkerIndices', marker_indices, 'MarkerEdgeColor', color, 'MarkerFaceColor', color, 'MarkerSize', 5);
end

title(['CEC2020-F' num2str(Function_name)])
xlabel('Iteration');
ylabel('Fitness');
axis tight
box on

% 显示图例，由于在plot中已经设置了颜色和标记，图例会自动反映这些属性
legend(algorithm_labels, 'Location', 'best');

hold off % 结束hold状态

 % 生成文件名
baseFolder = 'D:\研究生\Algorithm\赵涛\ISSA2\程序\结果\2020\图\收敛图\';
figFilename = sprintf('%sF%d.fig', baseFolder, Function_name);
emfFilename = sprintf('%sF%d.emf', baseFolder, Function_name);
saveas(gcf, figFilename); % 保存为.fig文件
print(gcf, emfFilename, '-dmeta'); % 保存为.emf文件