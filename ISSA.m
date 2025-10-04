function [IfMin, IbestX, Curve_ISSA] = ISSA(N, Max_iter, lb, ub, dim, fobj)
% 定义函数ISSA，输入参数包括：种群数量N、最大迭代次数Max_iter、下界lb、上界ub、维度dim以及目标函数fobj

if size(ub, 1) == 1 % 如果上界或下界是一个标量
    ub = ones(1, dim) .* ub; % 将其转换为与维度相匹配的行向量
    lb = ones(1, dim) .* lb; % 同上
end

% 初始化收敛曲线
Curve_ISSA = zeros(1, Max_iter);

% 初始化座头鲸的位置
SalpPositions = ISSAinitialization(N, dim, ub, lb); % 调用初始化函数以随机生成座头鲸的初始位置

% 初始化食物位置和适应度值
IbestX = zeros(1, dim); % 食物位置初始化为零向量
IfMin = inf; % 最优适应度初始化为无穷大

% 计算初始座头鲸的适应度
for i = 1:size(SalpPositions, 1) % 对于每一个座头鲸
    SalpFitness(1, i) = fobj(SalpPositions(i, :)); % 计算适应度
end

% 对座头鲸按适应度排序
[sorted_salps_fitness, sorted_indexes] = sort(SalpFitness); % 获取排序后的适应度及其索引

% 按适应度排序座头鲸的位置
for newindex = 1:N % 根据排序后的索引重新排列座头鲸的位置
    Sorted_salps(newindex, :) = SalpPositions(sorted_indexes(newindex), :);
end

% 更新食物位置和适应度
IbestX = Sorted_salps(1, :); % 将最优座头鲸的位置作为食物位置
IfMin = sorted_salps_fitness(1); % 将最优座头鲸的适应度作为食物适应度

% 主循环
l = 2; % 从第二次迭代开始，因为第一次迭代用于计算座头鲸的适应度
while l < Max_iter + 1 % 当前迭代次数小于最大迭代次数时继续迭代
    c1 = 2 * exp(-(4 * l / Max_iter)^2); % 计算参数c1，用于控制探索与开发之间的平衡
    t = T(Max_iter); % 这里假设T是一个预先定义的函数或变量，用于计算温度或其他动态参数

    % 更新座头鲸的位置
    for i = 1:size(SalpPositions, 1) % 对于每一个座头鲸
        SalpPositions = SalpPositions'; % 转置座头鲸的位置矩阵，便于后续操作
        
        if i <= N / 2 % 前半部分座头鲸
            for j = 1:dim % 对于每一个维度
                c2 = rand(); % 生成一个0到1之间的随机数
                c3 = rand(); % 再生成一个0到1之间的随机数
                
                % 根据公式更新座头鲸的位置
                if c3 < 0.5
                    SalpPositions(j, i) = IbestX(j) + c1 * ((ub(j) - lb(j)) * c2 + lb(j)); % 探索新区域
                else
                    SalpPositions(j, i) = IbestX(j) - c1 * ((ub(j) - lb(j)) * c2 + lb(j)); % 或者返回已知区域
                end
            end
        elseif i > N / 2 && i < N + 1 % 后半部分座头鲸
            tao = (sqrt(5) - 1) / 2; % 黄金分割比
            R1 = t(l) * pi * 2; % 动态参数
            R2 = t(l) * pi; % 动态参数
            x1 = -pi + 2 * pi * (1 - tao); % 动态参数
            x2 = -pi + tao * 2 * pi; % 动态参数
            % 使用复杂的数学公式更新座头鲸的位置
            SalpPositions(:, i) = SalpPositions(:, i) * abs(sin(R1)) + R2 * sin(R1) * abs(x1 * IbestX' - x2 * SalpPositions(:, i));
        end
        SalpPositions = SalpPositions'; % 转置回原形式
    end

    % 重新计算适应度并排序
    for o = 1:size(SalpPositions, 1)
        SalpFitness(1, o) = fobj(SalpPositions(o, :)); % 重新计算每个座头鲸的适应度
    end
    [~, sorted_indexes] = sort(SalpFitness); % 重新排序适应度
    for newindex = 1:N
        Sorted_salps(newindex, :) = SalpPositions(sorted_indexes(newindex), :); % 重新排序座头鲸的位置
    end

    % 更新座头鲸的位置（探索与开发阶段）
    for i = 1:(N / 2 + floor(N * (1 - atan(1 + (i / l)^3)))) % 前半部分座头鲸
        sita = rand() * pi / 3; % 生成一个角度
        choose = IbestX; % 选择一个目标位置
        while isequal(choose, SalpPositions(i, :)) % 确保选择的目标位置不是当前位置
            choose = SalpPositions(ceil(rand() * N), :);
        end
        SalpPositions1(i, :) = SalpPositions(i, :) + tan(sita) * (IbestX - SalpPositions(i, :)) * (1 - l / Max_iter); % 更新位置
        if fobj(SalpPositions1(i, :)) < fobj(SalpPositions(i, :)) % 如果新位置的适应度更好
            SalpPositions(i, :) = SalpPositions1(i, :); % 则接受新位置
        end
    end

    for i = ((N / 2 + floor(N * (1 - atan(1 + (i / l)^3)))) + 1):N % 后半部分座头鲸
        sig = 1 - sin((rand() * pi * l) / (2 * Max_iter)); % 计算一个动态参数
        SalpPositions1(i, :) = SalpPositions(i, :) + rand() * sig * (IbestX - SalpPositions(i, :)) + (1 - rand()) * sig * (SalpPositions(ceil(rand() * N), :) - SalpPositions(i, :)); % 更新位置
        if fobj(SalpPositions1(i, :)) < fobj(SalpPositions(i, :)) % 如果新位置的适应度更好
            SalpPositions(i, :) = SalpPositions1(i, :); % 则接受新位置
        end
    end

    % 边界处理
    for i = 1:size(SalpPositions, 1)
        Tp = SalpPositions(i, :) > ub; % 检查是否超出上界
        Tm = SalpPositions(i, :) < lb; % 检查是否低于下界
        
        % 生成一个标量随机数
        rand_scalar = rand;
        
        % 对超出边界的座头鲸进行位置调整
        SalpPositions(i, :) = (SalpPositions(i, :) .* ~(Tp + Tm)) + (rand_scalar * ub .* Tp) + (rand_scalar * lb .* Tm);
        
        % 重新计算适应度
        SalpFitness(1, i) = fobj(SalpPositions(i, :));
        
        % 如果找到更好的解，则更新食物位置和适应度
        if SalpFitness(1, i) < IfMin
            IbestX = SalpPositions(i, :);
            IfMin = SalpFitness(1, i);
        end
    end

    % 更新收敛曲线
    Curve_ISSA(l - 1) = IfMin; % 记录当前最优适应度
    l = l + 1; % 迭代计数器加1
end