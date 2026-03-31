% =========================================================================
%  ISSAinitialization — Good Point Set (GPS) Initialization
%  [Sec 3.1, Eq. 4-6 of ISSA paper]
%
%  BUG-FIX v3（关键修正）：
%
%  【第一个bug，已知】 p=3 时 2cos(2πj/3)=-1（整数），mod(-i,1)=0，
%   所有点退化到 lb（单点塌缩）。
%
%  【第二个bug，本次修正】 p=5, dim=2 时仍然退化：
%   代数恒等式：对任意奇素数 p，有
%       sum_{j=1}^{(p-1)/2}  2*cos(2*pi*j/p) = -1
%   当 dim = (p-1)/2 时，第 i 行两列 GPS 值之和 = i*(-1) = -i（整数），
%   导致 mod(col1,1)+mod(col2,1)=1，所有点落在 X+Y=1 的对角线上。
%
%  【根本原因】必须保证 p > 2*dim+1，即 dim < (p-1)/2，
%   才能避免"所有生成元之和为整数"的代数恒等式造成退化。
%
%  【正确修复】从 2*dim+2 开始搜索最小素数 p，确保 p > 2*dim+1：
%     dim=1 → p=5  （原来 p=3 或 p=5 凑巧正确，但从语义上应显式保证）
%     dim=2 → p=7  （原来 p=5 错误，对角线退化）
%     dim=3 → p=11 （原来 p=5 或 p=7 均退化，11 是 >7 的最小素数）
%     dim=4 → p=11
%     dim=5 → p=13
%     ...
%
%  Inputs:
%    N   - Population size
%    dim - Number of dimensions
%    ub  - Upper bound(s) (scalar or 1×dim vector)
%    lb  - Lower bound(s) (scalar or 1×dim vector)
%
%  Output:
%    Positions - N×dim matrix of initialised positions in [lb, ub]
% =========================================================================

function Positions = ISSAinitialization(N, dim, ub, lb)

%% ---- Boundary expansion (scalar → vector) --------------------------------
if numel(ub) == 1
    ub = repmat(ub, 1, dim);
    lb = repmat(lb, 1, dim);
end

% =========================================================================
%  Good Point Set (GPS) Initialization  [Eq. 4-6]
%
%  Eq. 6:  x_ij  = i * 2*cos(2*pi*j / p)
%  Eq. 5:  JD    = mod(x_ij, 1)                (fractional part → [0,1))
%  Eq. 4:  X_j   = lb_j + (ub_j - lb_j) * JD  (map to search space)
%
%  关键要求：必须选取素数 p > 2*dim + 1
%  理由：对任意奇素数 p，sum_{j=1}^{(p-1)/2} 2cos(2πj/p) = -1（整数）。
%        若 dim = (p-1)/2，所有生成元之和为整数，导致各行坐标线性相关，
%        点集退化到一条超平面上（dim=2时为对角线 X+Y=1）。
%        确保 p > 2*dim+1（即 dim < (p-1)/2）可避开此恒等式。
% =========================================================================

% Step 1: 寻找满足 p > 2*dim+1 的最小素数
%   从 2*dim+2 开始，保证 p > 2*dim+1
%   这样 dim < (p-1)/2，不会触发"所有生成元之和=-1"的代数退化
p = 2 * dim + 2;          % 起始搜索点，确保 p > 2*dim+1
while ~isprime(p)
    p = p + 1;
end

% Step 2: 构造 GPS 原始矩阵  [Eq. 6]
%   x_ij = i * 2*cos(2*pi*j/p)，i=1..N，j=1..dim
i_vec = (1:N)';                                      % N×1
j_vec = 1:dim;                                       % 1×dim
x_gps = i_vec * (2 * cos(2 * pi * j_vec / p));      % N×dim（外积）

% Step 3: 取小数部分  [Eq. 5]
JD = mod(x_gps, 1);                                  % N×dim，值域 [0,1)

% Step 4: 映射到搜索空间  [Eq. 4]
Positions = lb + (ub - lb) .* JD;                   % N×dim

end
