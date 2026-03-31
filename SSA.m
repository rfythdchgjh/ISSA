function [fMin, bestX, Curve_SSA] = SSA(N, Max_iter, lb, ub, dim, fobj)
% 樽海鞘群算法 (SSA) — 完整修复版本
%
% 修复清单：
%   [S1] 领导者更新去除多余 +lb(j) 偏移
%   [S2] 跟随者链式更新加 i>1 越界防护
%   [S3] 初始化后排序，确保 bestX 是真实最优
%   [S4] 输出参数名与 main.m 调用顺序保持一致

%% 边界扩展
lb = lb(:)';
ub = ub(:)';
if numel(lb) == 1, lb = repmat(lb, 1, dim); end
if numel(ub) == 1, ub = repmat(ub, 1, dim); end

%% 初始化种群
SalpPositions = lb + (ub - lb) .* rand(N, dim);

SalpFitness = zeros(1, N);
for i = 1:N
    SalpFitness(i) = fobj(SalpPositions(i, :));
end

% ★ [S3]：初始化后排序，确保 bestX 是真实最优
[SalpFitness, sort_idx] = sort(SalpFitness);
SalpPositions = SalpPositions(sort_idx, :);

fMin  = SalpFitness(1);
bestX = SalpPositions(1, :);

Curve_SSA = zeros(1, Max_iter);

%% 主循环
for iter = 1:Max_iter
    c1 = 2 * exp(-(4 * iter / Max_iter)^2);

    for i = 1:N
        if i <= N / 2
            %% 领导者更新  [SSA Eq.3.1]
            % ★ [S1] 步长 = (ub-lb)*c2，不含 +lb(j)
            for j = 1:dim
                c2 = rand();
                c3 = rand();
                if c3 < 0.5
                    SalpPositions(i, j) = bestX(j) + c1 * (ub(j) - lb(j)) * c2;
                else
                    SalpPositions(i, j) = bestX(j) - c1 * (ub(j) - lb(j)) * c2;
                end
            end
        else
            %% 跟随者：算术平均
            % ★ [S2] i>1 防护
            if i > 1
                SalpPositions(i, :) = 0.5 * (SalpPositions(i, :) + SalpPositions(i-1, :));
            end
        end
    end

    %% 边界约束 + 适应度 + 全局最优更新
    for i = 1:N
        SalpPositions(i, :) = max(SalpPositions(i, :), lb);
        SalpPositions(i, :) = min(SalpPositions(i, :), ub);

        SalpFitness(i) = fobj(SalpPositions(i, :));

        if SalpFitness(i) < fMin
            bestX = SalpPositions(i, :);
            fMin  = SalpFitness(i);
        end
    end

    Curve_SSA(iter) = fMin;
end

end
