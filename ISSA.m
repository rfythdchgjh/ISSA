% =========================================================================
%  ISSA — Improved Salp Swarm Algorithm v4
%
%  在 v3 基础上的关键修复：
%   [F1] Step A 增加贪婪选择：领导者和跟随者均只接受改进的更新
%        ★ 这是 ISSA 比 SSA 差的核心原因：原来差的移动也无条件接受，
%          导致精英池被污染，Bootstrap 双区优化完全失效
%   [F2] 每粒子独立混沌状态 T_vec(N)，替代全局共享的单一 T
%        ★ 原来所有跟随者共用一个 T，跟随者群多样性为零
%   [F3] GSCS 公式重构：振荡收缩 + c1 衰减幅度控制，去除不稳定的 step_size
%        ★ 原 step_size=|disp_food+disp_pred|/2 在粒子分散时产生巨大步长
%   [F4] 排序后 T_vec 跟随 sort_idx 重排，保持混沌状态与粒子绑定
%
%  保留原框架（不改变基本改进方式）：
%   GPS 准随机 + 伪随机混合初始化 [I10]
%   Bootstrap 区正切飞行 + gamma 余弦衰减 [I6]
%   学习区双样本学习
%   动态分区 delta + 上限 75%N [I5, I12]
%   帐篷混沌映射 [I4]
%   反射边界 [I9]
% =========================================================================

function [FoodFitness, FoodPosition, Convergence_curve] = ISSA(N, Max_iter, lb, ub, dim, fobj)

%% 边界扩展
if numel(ub) == 1, ub = repmat(ub, 1, dim); end
if numel(lb) == 1, lb = repmat(lb, 1, dim); end
lb    = lb(:)';
ub    = ub(:)';
range = ub - lb;

% =========================================================================
%  GPS 准随机矩阵  [Eq. 6]
% =========================================================================
p = 2 * dim + 2;
while ~isprime(p)
    p = p + 1;
end
i_vec = (1:N)';
j_vec = 1:dim;
JD    = mod(i_vec * (2 * cos(2 * pi * j_vec / p)), 1);   % N×dim, [0,1)

% [I10] 混合初始化：50% GPS 准随机 + 50% 伪随机
SalpPositions = lb + range .* (0.5 * JD + 0.5 * rand(N, dim));

% =========================================================================
%  [F2] 每粒子独立混沌状态初始化
%       使用 JD 第一列作为基础种子，加细粒度随机扰动区分每粒子
% =========================================================================
beta  = 0.7;
T_vec = mod(JD(:, 1) + 0.01 * rand(N, 1), 1);   % N×1
for i = 1:N
    if T_vec(i) < 1e-5 || T_vec(i) > 1 - 1e-5 || abs(T_vec(i) - beta) < 1e-4
        T_vec(i) = 0.2 + 0.6 * rand();
    end
end

% =========================================================================
%  初始适应度
% =========================================================================
SalpFitness = zeros(1, N);
for i = 1:N
    SalpFitness(i) = fobj(SalpPositions(i, :));
end

% 升序排列
[SalpFitness, sort_idx] = sort(SalpFitness);
SalpPositions = SalpPositions(sort_idx, :);
T_vec         = T_vec(sort_idx);        % [F4] 混沌状态跟随粒子排序

FoodPosition = SalpPositions(1, :);
FoodFitness  = SalpFitness(1);

Convergence_curve = zeros(1, Max_iter);

% =========================================================================
%  主循环
% =========================================================================
for t = 1:Max_iter

    c1 = 2 * exp(-(4 * t / Max_iter)^2);

    % =====================================================================
    %  STEP A — 领导者 & 跟随者更新  [F1: 贪婪选择]
    %
    %  ★ 关键修复：每次移动前保存旧位置，只在适应度改善时才接受新位置
    %    作用：防止差的移动污染种群，使 Bootstrap 双区始终作用于高质量粒子
    % =====================================================================
    for i = 1:N
        OldPos = SalpPositions(i, :);
        OldFit = SalpFitness(i);

        if i <= N / 2
            % ----------------------------------------------------------------
            %  领导者更新  [SSA Eq. 3.1]
            % ----------------------------------------------------------------
            NewPos = OldPos;
            for j = 1:dim
                c2 = rand();
                if rand() < 0.5
                    NewPos(j) = FoodPosition(j) + c1 * range(j) * c2;
                else
                    NewPos(j) = FoodPosition(j) - c1 * range(j) * c2;
                end
            end
            NewPos = max(lb, min(ub, NewPos));

        else
            % ----------------------------------------------------------------
            %  跟随者：独立混沌 GSCS  [F2, F3]
            % ----------------------------------------------------------------

            % [F2] 迭代此粒子自身的混沌状态（帐篷映射）
            T_i = T_vec(i);
            if T_i < beta
                T_i = T_i / beta;
            else
                T_i = (1 - T_i) / (1 - beta);
            end
            T_i = max(1e-6, min(1 - 1e-6, T_i));
            if abs(T_i - beta) < 1e-4, T_i = T_i + 1e-3; end
            T_vec(i) = T_i;

            R1 = T_i * 2 * pi;

            % [F3] 重构的 GSCS 公式
            %
            %  原理：以食物为振荡中心，abs(sin(R1)) 控制向食物的收缩率
            %   - abs(sin(R1)) = 0  →  NewPos = FoodPosition（收敛到食物）
            %   - abs(sin(R1)) = 1  →  NewPos ≈ OldPos（保持不动 + 小扰动）
            %   - 第二项：c1 衰减的混沌扰动，保证后期探索幅度自动缩小
            %
            %  相比原公式优势：
            %   - 去除不稳定的 step_size（原来在粒子分散时可以超大）
            %   - amplitude 上限 = 0.1×c1×range×T_i，可控且后期收缩
            %   - sin(R1) 提供双向探索（正负方向），符合 GSA 设计意图

            to_food   = FoodPosition - OldPos;
            amplitude = c1 .* range .* 0.1 .* T_i;
            NewPos    = FoodPosition + abs(sin(R1)) .* to_food ...
                      + sin(R1) .* amplitude .* (2 * rand(1, dim) - 1);

            % [I9] 反射边界（最多 10 次迭代）
            for j = 1:dim
                cnt = 0;
                while (NewPos(j) < lb(j) || NewPos(j) > ub(j)) && cnt < 10
                    if NewPos(j) < lb(j), NewPos(j) = 2*lb(j) - NewPos(j); end
                    if NewPos(j) > ub(j), NewPos(j) = 2*ub(j) - NewPos(j); end
                    cnt = cnt + 1;
                end
                NewPos(j) = max(lb(j), min(ub(j), NewPos(j)));
            end
        end

        % ------------------------------------------------------------------
        %  [F1] 贪婪接受：只接受适应度改善的移动
        % ------------------------------------------------------------------
        NewFit = fobj(NewPos);
        if NewFit <= OldFit
            SalpPositions(i, :) = NewPos;
            SalpFitness(i)      = NewFit;
            if NewFit < FoodFitness
                FoodPosition = NewPos;
                FoodFitness  = NewFit;
            end
        end
        % 否则：SalpPositions(i,:) 和 SalpFitness(i) 保持不变
    end

    % =====================================================================
    %  STEP B — 动态双区优化  [Sec 3.3]
    % =====================================================================

    % 升序重排（Step A 之后）
    [SalpFitness, sort_idx] = sort(SalpFitness);
    SalpPositions = SalpPositions(sort_idx, :);
    T_vec         = T_vec(sort_idx);          % [F4] 混沌状态跟随重排

    % [I5] 自适应分区，[I12] 上限钳位到 75%N
    delta = (t / Max_iter)^2 * 0.5;
    m     = round(N / 2 + N * delta);
    m     = max(1, min(round(0.75 * N), m));

    % ------------------------------------------------------------------
    %  Bootstrap 区：正切飞行 + gamma 余弦衰减  [I6]
    % ------------------------------------------------------------------
    gamma = 0.5 * (1 + cos(pi * t / Max_iter));

    for i = 1:m
        theta = rand() * pi / 3;
        X_new = SalpPositions(i, :) + ...
                tan(theta) * gamma .* (FoodPosition - SalpPositions(i, :));
        X_new = max(lb, min(ub, X_new));
        f_new = fobj(X_new);
        if f_new < SalpFitness(i)
            SalpPositions(i, :) = X_new;
            SalpFitness(i)      = f_new;
            if f_new < FoodFitness
                FoodPosition = X_new;
                FoodFitness  = f_new;
            end
        end
    end

    % ------------------------------------------------------------------
    %  学习区：双样本学习  [Eq. 16-19]
    % ------------------------------------------------------------------
    for i = m + 1 : N
        alpha = cos(rand() * pi * t / (2 * Max_iter));
        j_idx = randi(N);
        while j_idx == i, j_idx = randi(N); end
        r1 = rand();
        r2 = 1 - r1;
        X_new = SalpPositions(i, :) + alpha * ( ...
                    r1 .* (FoodPosition             - SalpPositions(i, :)) + ...
                    r2 .* (SalpPositions(j_idx, :)  - SalpPositions(i, :)) );
        X_new = max(lb, min(ub, X_new));
        f_new = fobj(X_new);
        if f_new < SalpFitness(i)
            SalpPositions(i, :) = X_new;
            SalpFitness(i)      = f_new;
            if f_new < FoodFitness
                FoodPosition = X_new;
                FoodFitness  = f_new;
            end
        end
    end

    Convergence_curve(t) = FoodFitness;
end

end
