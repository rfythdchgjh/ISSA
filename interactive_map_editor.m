function [map_out, S_out, E_out] = interactive_map_editor(map_in, S_in, E_in)
% =========================================================================
%  interactive_map_editor  —  交互式地图编辑器（完整重写版）
%
%  修复项：
%    [R1] 自适应格子大小：根据屏幕和地图尺寸动态计算 cell_px，大地图不再超屏
%    [R2] 调整地图尺寸时全部清零（不保留旧障碍），避免混乱
%    [R3] 全面重设计 UI：分区清晰，按钮齐全
%
%  新增功能：
%    [N1] 随机障碍物生成（可设置障碍概率 0~0.8）
%    [N2] 填充全部障碍 / 清除全部障碍
%    [N3] 撤销（Undo，最多 20 步）
%    [N4] 地图尺寸预设快捷按钮（10×10 / 20×20 / 30×30 / 50×50）
%    [N5] 拖拽绘制：鼠标按住拖动可连续绘制/擦除障碍
%
%  操作模式（Radio 互斥）:
%    模式1 — 切换障碍/自由（支持拖拽）
%    模式2 — 设置起点 S
%    模式3 — 设置终点 G
% =========================================================================

%% ── 初始状态 ─────────────────────────────────────────────────────────────
map_out  = map_in;
S_out    = S_in;
E_out    = E_in;

cur_map  = map_in;
cur_S    = S_in;
cur_E    = E_in;
orig_map = map_in;
[nR, nC] = size(map_in);

cur_mode     = 1;       % 1=障碍 2=起点 3=终点
is_dragging  = false;   % 拖拽绘制状态
drag_val     = 1;       % 拖拽时写入的值（1=加障碍 0=擦除）

% 撤销栈（最多保存 20 步）
UNDO_MAX  = 20;
undo_stack = {};        % cell array of map snapshots

%% ── 屏幕尺寸检测（用于自适应 cell_px）────────────────────────────────────
scr       = get(0, 'ScreenSize');   % [1 1 W H]
SCREEN_W  = scr(3);
SCREEN_H  = scr(4);

%% ── 布局常量（像素，不随地图变化）─────────────────────────────────────────
TOP_H   = 130;   % 顶部控制区总高度
BOT_H   =  50;   % 底部状态栏高度
PAD_L   =  55;   % 坐标轴左边距（Y轴标签）
PAD_B   =  36;   % 坐标轴下边距（X轴标签）
PAD_R   =  12;   % 坐标轴右边距
PAD_TOP =   8;   % 坐标轴与顶部区之间间隙

%% ── 首次建窗 ─────────────────────────────────────────────────────────────
[cell_px, win_w, win_h] = calc_layout(nR, nC);
fig = figure('Name','交互式地图编辑器', 'NumberTitle','off', ...
    'Position',[max(1,round((SCREEN_W-win_w)/2)), ...
                max(40,round((SCREEN_H-win_h)/2)), win_w, win_h], ...
    'Resize','on', 'CloseRequestFcn',@on_close, ...
    'Color',[0.95 0.96 0.97]);

ax = axes('Parent',fig,'Units','pixels', ...
    'Position',[PAD_L, BOT_H+PAD_B, nC*cell_px, nR*cell_px], ...
    'Color',[0.97 0.99 0.98]);

build_controls();

set(fig,'WindowButtonDownFcn',  @on_btn_down);
set(fig,'WindowButtonMotionFcn',@on_btn_motion);
set(fig,'WindowButtonUpFcn',    @on_btn_up);

draw_map();
waitfor(fig);

map_out = cur_map;
S_out   = cur_S;
E_out   = cur_E;

%% ═════════════════════════════════════════════════════════════════════════
%%  布局计算
%% ═════════════════════════════════════════════════════════════════════════
    function [cpx, ww, wh] = calc_layout(nr, nc)
        % 可用绘图区最大像素
        max_ax_w = SCREEN_W - PAD_L - PAD_R  - 40;
        max_ax_h = SCREEN_H - TOP_H - PAD_B  - BOT_H - PAD_TOP - 80;
        % 每格最大允许像素（同时不超过 40px，不小于 6px）
        cpx = min(40, min(floor(max_ax_w/nc), floor(max_ax_h/nr)));
        cpx = max(cpx, 6);
        ww  = max(700, PAD_L + nc*cpx + PAD_R + 20);
        wh  = TOP_H + PAD_TOP + nr*cpx + PAD_B + BOT_H + 10;
        % 不超过屏幕 90%
        ww  = min(ww, round(SCREEN_W * 0.92));
        wh  = min(wh, round(SCREEN_H * 0.90));
    end

%% ═════════════════════════════════════════════════════════════════════════
%%  控件构建
%% ═════════════════════════════════════════════════════════════════════════
    function build_controls()
        % ── 共用颜色 ────────────────────────────────────────────────────
        BG   = get(fig,'Color');
        C_BL = [0.12 0.18 0.28];   % 深蓝文字
        C_GR = [0 0.48 0.18];      % 绿色
        C_RD = [0.78 0.08 0.08];   % 红色

        top_y = win_h - TOP_H;     % 顶部区域底边Y坐标

        % ══════════════════════════════════════════════════════════════
        %  第1行：操作模式标题
        % ══════════════════════════════════════════════════════════════
        uicontrol('Parent',fig,'Style','text','Units','pixels', ...
            'Position',[8, top_y+100, 110, 20], ...
            'String','操作模式选择：','FontSize',9,'FontWeight','bold', ...
            'ForegroundColor',C_BL,'HorizontalAlignment','left', ...
            'BackgroundColor',BG);

        % ══════════════════════════════════════════════════════════════
        %  第1行：Radio 按钮组（操作模式）
        % ══════════════════════════════════════════════════════════════
        bg_mode = uibuttongroup('Parent',fig,'Units','pixels', ...
            'Position',[8, top_y+74, win_w-16, 28], ...
            'BorderType','none','BackgroundColor',BG, ...
            'SelectionChangedFcn',@on_mode_change);

        rb1 = uicontrol('Parent',bg_mode,'Style','radiobutton', ...
            'Units','pixels','Position',[0,3,190,22], ...
            'String','■  切换障碍 / 自由 (拖拽)', ...
            'FontSize',9,'Value',1,'BackgroundColor',BG,'ForegroundColor',C_BL);
        rb2 = uicontrol('Parent',bg_mode,'Style','radiobutton', ...
            'Units','pixels','Position',[200,3,140,22], ...
            'String','●  设置起点 S', ...
            'FontSize',9,'Value',0,'BackgroundColor',BG,'ForegroundColor',C_GR);
        rb3 = uicontrol('Parent',bg_mode,'Style','radiobutton', ...
            'Units','pixels','Position',[350,3,140,22], ...
            'String','●  设置终点 G', ...
            'FontSize',9,'Value',0,'BackgroundColor',BG,'ForegroundColor',C_RD);

        % ══════════════════════════════════════════════════════════════
        %  第2行：地图尺寸区
        % ══════════════════════════════════════════════════════════════
        row2_y = top_y + 44;

        uicontrol('Parent',fig,'Style','text','Units','pixels', ...
            'Position',[8,row2_y,60,22], 'String','地图行数:', ...
            'FontSize',9,'HorizontalAlignment','left','BackgroundColor',BG);
        edit_rows = uicontrol('Parent',fig,'Style','edit','Units','pixels', ...
            'Position',[68,row2_y,44,22],'String',num2str(nR),'FontSize',9);

        uicontrol('Parent',fig,'Style','text','Units','pixels', ...
            'Position',[120,row2_y,60,22],'String','地图列数:', ...
            'FontSize',9,'HorizontalAlignment','left','BackgroundColor',BG);
        edit_cols = uicontrol('Parent',fig,'Style','edit','Units','pixels', ...
            'Position',[180,row2_y,44,22],'String',num2str(nC),'FontSize',9);

        uicontrol('Parent',fig,'Style','pushbutton','Units','pixels', ...
            'Position',[234,row2_y,68,24],'String','应用尺寸', ...
            'FontSize',9,'Callback',@btn_resize_map);

        lbl_size_hint = uicontrol('Parent',fig,'Style','text','Units','pixels', ...
            'Position',[310,row2_y,160,22], ...
            'String',sprintf('当前：%d×%d', nR, nC), ...
            'FontSize',9,'HorizontalAlignment','left', ...
            'ForegroundColor',[0.45 0.45 0.45],'BackgroundColor',BG);

        % 尺寸预设快捷按钮
        preset_sizes = {10,20,30,50};
        preset_labels = {'10','20','30','50'};
        px = 478;
        uicontrol('Parent',fig,'Style','text','Units','pixels', ...
            'Position',[px-30,row2_y,28,22],'String','预设:', ...
            'FontSize',8,'HorizontalAlignment','right','BackgroundColor',BG);
        for pi_ = 1:4
            sz = preset_sizes{pi_};
            uicontrol('Parent',fig,'Style','pushbutton','Units','pixels', ...
                'Position',[px+(pi_-1)*40, row2_y, 36, 24], ...
                'String',preset_labels{pi_},'FontSize',9, ...
                'Callback',@(~,~) apply_preset(sz,sz));
        end

        % ══════════════════════════════════════════════════════════════
        %  第3行：随机障碍 + 功能按钮
        % ══════════════════════════════════════════════════════════════
        row3_y = top_y + 14;

        % 随机障碍概率
        uicontrol('Parent',fig,'Style','text','Units','pixels', ...
            'Position',[8,row3_y,68,22],'String','障碍概率:', ...
            'FontSize',9,'HorizontalAlignment','left','BackgroundColor',BG);
        edit_prob = uicontrol('Parent',fig,'Style','edit','Units','pixels', ...
            'Position',[76,row3_y,44,22],'String','0.30','FontSize',9);
        uicontrol('Parent',fig,'Style','pushbutton','Units','pixels', ...
            'Position',[128,row3_y,80,24],'String','🎲 随机障碍', ...
            'FontSize',9,'FontWeight','bold', ...
            'ForegroundColor',[0.1 0.3 0.7],'Callback',@btn_random_obstacle);

        % 功能按钮组
        btn_defs = { ...
            '撤  销 ↩',  @btn_undo,  [0.3 0.3 0.3]; ...
            '清除障碍',  @btn_clear, [0.1 0.45 0.1]; ...
            '填满障碍',  @btn_fill,  [0.55 0.18 0.08]; ...
            '恢复初始',  @btn_reset, [0.2 0.2 0.5]; ...
            '完成编辑 ✓',@btn_done,  [0 0.48 0.18]; ...
        };
        bw = 86; bh = 26; bx = win_w - size(btn_defs,1)*(bw+6) - 6;
        for bi = 1:size(btn_defs,1)
            uicontrol('Parent',fig,'Style','pushbutton','Units','pixels', ...
                'Position',[bx+(bi-1)*(bw+6), row3_y, bw, bh], ...
                'String',btn_defs{bi,1},'FontSize',9,'FontWeight','bold', ...
                'ForegroundColor',btn_defs{bi,3},'Callback',btn_defs{bi,2});
        end

        % ══════════════════════════════════════════════════════════════
        %  底部状态栏
        % ══════════════════════════════════════════════════════════════
        lbl_status = uicontrol('Parent',fig,'Style','text','Units','pixels', ...
            'Position',[8,14,win_w-16,24], ...
            'String','当前模式：切换障碍  |  提示：单击或拖拽鼠标绘制/擦除障碍格', ...
            'FontSize',9,'HorizontalAlignment','left', ...
            'ForegroundColor',[0.1 0.15 0.5],'BackgroundColor',BG);

        % 把需要跨函数访问的 handle 存到 fig UserData
        ud.lbl_status    = lbl_status;
        ud.lbl_size_hint = lbl_size_hint;
        ud.edit_rows     = edit_rows;
        ud.edit_cols     = edit_cols;
        ud.edit_prob     = edit_prob;
        ud.rb1 = rb1; ud.rb2 = rb2; ud.rb3 = rb3;
        ud.bg_mode       = bg_mode;
        set(fig,'UserData',ud);
    end

%% ═════════════════════════════════════════════════════════════════════════
%%  辅助：从 UserData 取 handle
%% ═════════════════════════════════════════════════════════════════════════
    function h = ud_get(field)
        h = get(fig,'UserData').(field);
    end

%% ═════════════════════════════════════════════════════════════════════════
%%  模式切换
%% ═════════════════════════════════════════════════════════════════════════
    function on_mode_change(~, event)
        ud   = get(fig,'UserData');
        src  = event.NewValue;
        if     src == ud.rb1, cur_mode=1; msg='切换障碍  |  单击或拖拽绘制/擦除障碍格';  col=[0.1 0.15 0.5];
        elseif src == ud.rb2, cur_mode=2; msg='设置起点 S  |  点击自由格即设为起点';       col=[0 0.45 0.15];
        elseif src == ud.rb3, cur_mode=3; msg='设置终点 G  |  点击自由格即设为终点';       col=[0.75 0.08 0.08];
        end
        set(ud.lbl_status,'String',sprintf('当前模式：%s', msg),'ForegroundColor',col);
    end

%% ═════════════════════════════════════════════════════════════════════════
%%  鼠标事件（支持拖拽连续绘制）
%% ═════════════════════════════════════════════════════════════════════════
    function on_btn_down(~,~)
        if ~isvalid(fig), return; end
        [row, col] = get_grid_pos();
        if row<1, return; end
        switch cur_mode
            case 1
                if isequal([row,col],cur_S)||isequal([row,col],cur_E)
                    print_hint('起/终点格不能设为障碍'); return; end
                push_undo();
                drag_val = 1 - cur_map(row,col);   % 第一格决定本次拖拽是加还是擦
                cur_map(row,col) = drag_val;
                is_dragging = true;
                draw_map();
            case 2
                if cur_map(row,col)==1, print_hint('起点不能在障碍上'); return; end
                if isequal([row,col],cur_E), print_hint('起点不能与终点重合'); return; end
                cur_S=[row,col]; fprintf('  起点→[%d,%d]\n',row,col); draw_map();
            case 3
                if cur_map(row,col)==1, print_hint('终点不能在障碍上'); return; end
                if isequal([row,col],cur_S), print_hint('终点不能与起点重合'); return; end
                cur_E=[row,col]; fprintf('  终点→[%d,%d]\n',row,col); draw_map();
        end
    end

    function on_btn_motion(~,~)
        if ~is_dragging || cur_mode~=1, return; end
        if ~isvalid(fig), return; end
        [row, col] = get_grid_pos();
        if row<1, return; end
        if isequal([row,col],cur_S)||isequal([row,col],cur_E), return; end
        if cur_map(row,col) ~= drag_val
            cur_map(row,col) = drag_val;
            draw_map();
        end
    end

    function on_btn_up(~,~)
        is_dragging = false;
    end

    function [row, col] = get_grid_pos()
        % 将鼠标位置转为行列索引；越界返回 row=0
        try
            cp  = get(ax,'CurrentPoint');
            col = floor(cp(1,1)) + 1;
            row = floor(cp(1,2)) + 1;
            if row<1||row>nR||col<1||col>nC, row=0; col=0; end
        catch
            row=0; col=0;
        end
    end

%% ═════════════════════════════════════════════════════════════════════════
%%  按钮回调
%% ═════════════════════════════════════════════════════════════════════════
    function btn_resize_map(~,~)
        ud = get(fig,'UserData');
        nr2 = round(str2double(get(ud.edit_rows,'String')));
        nc2 = round(str2double(get(ud.edit_cols,'String')));
        if isnan(nr2)||isnan(nc2)||nr2<2||nc2<2||nr2>200||nc2>200
            set(ud.lbl_status,'String','[错误] 行列数需在 2~200 之间','ForegroundColor',[0.8 0 0]); return; end
        apply_resize(nr2, nc2);
    end

    function apply_preset(nr2, nc2)
        ud = get(fig,'UserData');
        set(ud.edit_rows,'String',num2str(nr2));
        set(ud.edit_cols,'String',num2str(nc2));
        apply_resize(nr2, nc2);
    end

    function apply_resize(nr2, nc2)
        % [R2] 调整尺寸时全部清零，不保留旧障碍
        nR = nr2; nC = nc2;
        cur_map  = zeros(nR, nC);
        orig_map = zeros(nR, nC);
        undo_stack = {};
        % 起终点若越界则移到角落
        cur_S = [min(cur_S(1),nR), min(cur_S(2),nC)];
        cur_E = [min(cur_E(1),nR), min(cur_E(2),nC)];
        if isequal(cur_S, cur_E)
            if nR>1, cur_E=[nR,cur_E(2)]; else, cur_E=[1,min(nC,cur_E(2)+1)]; end
        end
        % [R1] 重新计算自适应格子大小
        [cell_px, win_w, win_h] = calc_layout(nR, nC);
        old_pos = get(fig,'Position');
        set(fig,'Position',[old_pos(1), old_pos(2), win_w, win_h]);
        set(ax,'Position',[PAD_L, BOT_H+PAD_B, nC*cell_px, nR*cell_px]);
        % 更新提示
        ud = get(fig,'UserData');
        set(ud.lbl_size_hint,'String',sprintf('当前：%d×%d', nR, nC));
        set(ud.edit_rows,'String',num2str(nR));
        set(ud.edit_cols,'String',num2str(nC));
        fprintf('  地图已重置为 %d×%d（全部清零）\n', nR, nC);
        draw_map();
    end

    function btn_random_obstacle(~,~)
        ud   = get(fig,'UserData');
        prob = str2double(get(ud.edit_prob,'String'));
        if isnan(prob)||prob<0||prob>0.8
            set(ud.lbl_status,'String','[错误] 障碍概率需在 0.00 ~ 0.80 之间', ...
                'ForegroundColor',[0.8 0 0]); return; end
        push_undo();
        rand_mat = rand(nR, nC) < prob;
        rand_mat(cur_S(1), cur_S(2)) = 0;   % 保护起点
        rand_mat(cur_E(1), cur_E(2)) = 0;   % 保护终点
        cur_map = double(rand_mat);
        fprintf('  随机障碍已生成，概率=%.2f，障碍格=%d\n', prob, sum(cur_map(:)));
        draw_map();
    end

    function btn_undo(~,~)
        if isempty(undo_stack)
            ud = get(fig,'UserData');
            set(ud.lbl_status,'String','没有可撤销的操作','ForegroundColor',[0.5 0.5 0.5]); return; end
        cur_map    = undo_stack{end};
        undo_stack = undo_stack(1:end-1);
        fprintf('  撤销完成（剩余 %d 步）\n', numel(undo_stack));
        draw_map();
    end

    function btn_clear(~,~)
        push_undo();
        cur_map = zeros(nR, nC);
        fprintf('  已清除所有障碍\n'); draw_map();
    end

    function btn_fill(~,~)
        push_undo();
        cur_map = ones(nR, nC);
        cur_map(cur_S(1),cur_S(2)) = 0;
        cur_map(cur_E(1),cur_E(2)) = 0;
        fprintf('  已填满障碍（起终点保留）\n'); draw_map();
    end

    function btn_reset(~,~)
        push_undo();
        cur_map = orig_map; cur_S = S_in; cur_E = E_in;
        fprintf('  已恢复初始地图\n'); draw_map();
    end

    function btn_done(~,~)
        fprintf('  完成：%d×%d  障碍%d格  S[%d,%d]  G[%d,%d]\n', ...
            nR, nC, sum(cur_map(:)), cur_S(1), cur_S(2), cur_E(1), cur_E(2));
        delete(fig);
    end

    function on_close(~,~), fprintf('  窗口已关闭\n'); delete(fig); end

%% ═════════════════════════════════════════════════════════════════════════
%%  撤销栈
%% ═════════════════════════════════════════════════════════════════════════
    function push_undo()
        undo_stack{end+1} = cur_map;
        if numel(undo_stack) > UNDO_MAX
            undo_stack = undo_stack(end-UNDO_MAX+1:end);
        end
    end

%% ═════════════════════════════════════════════════════════════════════════
%%  地图绘制（自适应格子大小）
%% ═════════════════════════════════════════════════════════════════════════
    function draw_map()
        cla(ax); hold(ax,'on');

        % ── 绘制格子 ──────────────────────────────────────────────────
        % 性能优化：用 image() 代替逐格 fill()，速度提升数十倍
        img = zeros(nR, nC, 3);
        for i = 1:nR
            for j = 1:nC
                if cur_map(i,j)==1
                    img(i,j,:) = [77 144 138]/255;   % 障碍色（深青绿）
                else
                    img(i,j,:) = [233 248 246]/255;  % 自由色（淡青）
                end
            end
        end
        % 用 image 显示（x: 0~nC, y: 0~nR，需翻转y）
        image(ax, [0.5,nC-0.5], [nR-0.5, 0.5], flipud(img));

        % ── 网格线 ────────────────────────────────────────────────────
        % 只在 cell_px >= 8 时画格线，太小时省略以减少视觉噪声
        if cell_px >= 8
            lc = [180 210 207]/255;
            lw = max(0.3, min(0.8, cell_px/40));
            for j = 0:nC
                plot(ax,[j,j],[0,nR],'Color',lc,'LineWidth',lw);
            end
            for i = 0:nR
                plot(ax,[0,nC],[i,i],'Color',lc,'LineWidth',lw);
            end
        end

        % ── 起点标记 ──────────────────────────────────────────────────
        mk = max(6, min(16, cell_px*0.6));
        plot(ax, cur_S(2)-0.5, cur_S(1)-0.5, 'o', ...
            'Color',[0 0.72 0],'MarkerFaceColor',[0 0.72 0], ...
            'MarkerSize',mk,'LineWidth',1.5);
        if cell_px >= 10
            text(ax, cur_S(2)-0.5, cur_S(1)-0.5, 'S', ...
                'FontSize',max(7,round(cell_px*0.38)), ...
                'FontWeight','bold','Color','w', ...
                'HorizontalAlignment','center','VerticalAlignment','middle');
        end

        % ── 终点标记 ──────────────────────────────────────────────────
        plot(ax, cur_E(2)-0.5, cur_E(1)-0.5, 's', ...
            'Color',[0.88 0.05 0.05],'MarkerFaceColor',[0.88 0.05 0.05], ...
            'MarkerSize',mk,'LineWidth',1.5);
        if cell_px >= 10
            text(ax, cur_E(2)-0.5, cur_E(1)-0.5, 'G', ...
                'FontSize',max(7,round(cell_px*0.38)), ...
                'FontWeight','bold','Color','w', ...
                'HorizontalAlignment','center','VerticalAlignment','middle');
        end

        % ── 轴设置 ────────────────────────────────────────────────────
        xlim(ax,[0 nC]); ylim(ax,[0 nR]);
        xlabel(ax,'列','FontSize',8); ylabel(ax,'行','FontSize',8);
        n_obs = sum(cur_map(:));
        title(ax, sprintf('障碍:%d格 (%.0f%%)   S[%d,%d]   G[%d,%d]', ...
            n_obs, n_obs/(nR*nC)*100, cur_S(1),cur_S(2), cur_E(1),cur_E(2)), ...
            'FontSize',9);

        % 刻度：格子小时减少刻度密度
        step = max(1, ceil(max(nR,nC)/25));
        set(ax, 'XTick',0:step:nC, 'YTick',0:step:nR, ...
            'TickLength',[0 0], 'FontSize', max(6, min(8, cell_px-2)));
        set(ax,'YDir','normal');
        grid(ax,'off'); box(ax,'on');
        axis(ax,'equal'); xlim(ax,[0 nC]); ylim(ax,[0 nR]);
        drawnow limitrate;
    end

%% ═════════════════════════════════════════════════════════════════════════
%%  工具
%% ═════════════════════════════════════════════════════════════════════════
    function print_hint(msg)
        ud = get(fig,'UserData');
        set(ud.lbl_status,'String',['[提示] ' msg],'ForegroundColor',[0.6 0.35 0]);
    end

end
