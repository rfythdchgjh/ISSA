function path = astar_pathfinding(S, E, map)
% =========================================================================
%  8连通 A* 路径搜索
%  [F1修复] 若全局 COST_MODE 未初始化，默认使用 'euclid'（向后兼容）
%  输入:
%    S   - 起点 [row, col]
%    E   - 终点 [row, col]
%    map - 地图矩阵（0=自由, 1=障碍）
%  输出:
%    path - Nx2 路径矩阵，无路径则返回 S（单行）
% =========================================================================
global COST_MODE
if isempty(COST_MODE)
    COST_MODE = 'euclid';   % [F1] 安全默认值
end

rows  = size(map,1);
cols  = size(map,2);
start = sub2ind([rows,cols], S(1), S(2));
goal  = sub2ind([rows,cols], E(1), E(2));

% 起终点相同直接返回
if start == goal
    path = S;
    return;
end

g    = inf(rows*cols, 1);  g(start) = 0;
h    = local_heuristic_all(rows, cols, E, COST_MODE);
f    = inf(rows*cols, 1);  f(start) = h(start);
came = zeros(rows*cols, 1, 'uint32');
open   = false(rows*cols, 1);  open(start)   = true;
closed = false(rows*cols, 1);

while any(open)
    idx     = find(open);
    [~, k]  = min(f(idx));
    cur     = idx(k);

    if cur == goal, break; end

    open(cur)   = false;
    closed(cur) = true;

    [r,c] = ind2sub([rows,cols], cur);
    nbrs  = neighbors8(r, c, rows, cols);

    for m = 1:size(nbrs,1)
        rr  = nbrs(m,1);  cc  = nbrs(m,2);
        nid = sub2ind([rows,cols], rr, cc);

        if closed(nid) || map(rr,cc)==1, continue; end

        dr = abs(rr-r);  dc = abs(cc-c);
        if strcmpi(COST_MODE,'hop')
            step = 1;
        else
            % 直行=1, 斜行=sqrt(2)
            step = (dr==0 || dc==0)*1 + (dr==1 && dc==1)*sqrt(2);
        end

        tg = g(cur) + step;
        if ~open(nid) || tg < g(nid)
            came(nid) = cur;
            g(nid)    = tg;
            f(nid)    = g(nid) + h(nid);
            open(nid) = true;
        end
    end
end

% ---- 回溯路径 ----
if came(goal)==0 && goal~=start
    path = S;   % 无路径可达
    return;
end

rev = goal;
while rev(end) ~= start
    prev = came(rev(end));
    if prev == 0, break; end
    rev(end+1) = prev; %#ok<AGROW>
end
rev  = fliplr(rev);
path = zeros(numel(rev), 2);
for i = 1:numel(rev)
    [r,c]    = ind2sub([rows,cols], rev(i));
    path(i,:) = [r, c];
end
end

% =========================================================================
%  局部子函数（仅此文件可见）
% =========================================================================
function h = local_heuristic_all(rows, cols, E, mode)
% 预计算所有节点到终点的启发值
if strcmpi(mode,'hop')
    D = 1;
    h = zeros(rows*cols, 1);
    for r = 1:rows
        for c = 1:cols
            dx = abs(r-E(1));  dy = abs(c-E(2));
            h(sub2ind([rows,cols],r,c)) = D * max(dx,dy);      % Chebyshev
        end
    end
else
    D = 1;  D2 = sqrt(2);
    h = zeros(rows*cols, 1);
    for r = 1:rows
        for c = 1:cols
            dx = abs(r-E(1));  dy = abs(c-E(2));
            % Octile 距离（8邻域的最优可容纳启发）
            h(sub2ind([rows,cols],r,c)) = D*(dx+dy) + (D2-2*D)*min(dx,dy);
        end
    end
end
end

function nb = neighbors8(r, c, R, C)
% 返回 (r,c) 的 8邻域格子（不含自身，不越界）
nb = [];
for dr = -1:1
    for dc = -1:1
        if dr==0 && dc==0, continue; end
        rr = r+dr;  cc = c+dc;
        if rr>=1 && rr<=R && cc>=1 && cc<=C
            nb(end+1,:) = [rr, cc]; %#ok<AGROW>
        end
    end
end
end
