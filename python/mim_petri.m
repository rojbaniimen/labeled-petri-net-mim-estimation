%% Minimal Initial Marking (MIM) Estimation in Labeled Petri Nets
% Exact (Pareto-front) & Heuristic (Periodic Pruning) Implementations
% Author: Imen Rojbani (Electrical Engineering Student, ENSIT)
% Academic Module: Modeling of Complex Systems (2025-2026)

clear; clc;

%% 1. Benchmark Network Definition (Figure 1)
Pre = [1 1 0 0;
       0 0 1 0;
       0 0 1 0;
       0 0 0 1;
       0 0 0 0];

Post = [0 0 0 0;
        0 1 0 0;
        0 1 0 0;
        0 0 1 0;
        1 0 0 1];

C = Post - Pre;
n_places = 5;
n_transitions = 4;

labels = {'a', 'b', 'a', 'c'};
word = {'a', 'b', 'a', 'c'};

fprintf('=== BENCHMARK NETWORK (Figure 1: w = abac) ===\n');

% Algorithm 1: Inverse Observation
M0_1 = inverse_observation([1, 2, 3, 4], Post, C, n_places);
fprintf('Alg 1 (Inverse Obs): M0 = [%s] | tokens = %d\n', num2str(M0_1'), sum(M0_1));

% Algorithm 2: Exact Pareto Method
tic;
[b2, n2] = exact_algorithm(word, Pre, Post, C, labels, n_places, n_transitions);
fprintf('Alg 2 (Exact Pareto): M0 = [%s] | tokens = %d (%d sol, %.2f ms)\n', ...
    num2str(b2{1}'), sum(b2{1}), n2, toc*1000);

% Algorithm 3: Heuristic Approach
[b3, n3] = heuristic_algorithm(word, Pre, Post, C, labels, n_places, n_transitions, 1, 2);
fprintf('Alg 3 (Heuristic c=1,d=2): M0 = [%s] | tokens = %d (%d sol)\n', ...
    num2str(b3{1}'), sum(b3{1}), n3);

% Sensitivity Analysis
params = [0 1; 0 2; 1 1; 1 2; 1 4; 2 2];
fprintf('\n  c   d   Sb  #sol  MIM\n');
for r = 1:size(params, 1)
    c_v = params(r, 1);
    d_v = params(r, 2);
    Sb = max(1, floor(numel(word) / d_v));
    [bh, nh] = heuristic_algorithm(word, Pre, Post, C, labels, n_places, n_transitions, c_v, d_v);
    fprintf('%3d %3d %4d %5d %4d\n', c_v, d_v, Sb, nh, sum(bh{1}));
end

%% 2. Robotic Surveillance Application (3x3 Grid)
sub_labels = {'f','e','b','c','a','f','d','e','d'};
adj = [1 2; 1 4; 2 3; 2 5; 3 6; 4 5; 4 7; 5 6; 5 8; 6 9; 7 8; 8 9];
transitions3 = {};
Pre3 = zeros(9, 24);
Post3 = zeros(9, 24);
labels3 = cell(1, 24);

for r = 1:size(adj, 1)
    i = adj(r, 1); j = adj(r, 2);
    for pair = {[i, j], [j, i]}
        src = pair{1}(1); dst = pair{1}(2);
        k = numel(transitions3) + 1;
        transitions3{k} = struct('src', src, 'dst', dst, 'label', sub_labels{dst});
        labels3{k} = sub_labels{dst};
        Pre3(src, k) = 1;
        Post3(dst, k) = 1;
    end
end

C3 = Post3 - Pre3;
w3 = {'f','b','e','b','a','e','d','f','c','b','d','b','c','b','a','b','c','b','c','d'};

tic;
[b_rob, n_rob] = heuristic_algorithm(w3, Pre3, Post3, C3, labels3, 9, 24, 1, 2);
fprintf('\n[Robotic Surveillance 3x3] w length = %d\n', numel(w3));
fprintf('Heuristic (c=1, d=2): MIM = %d token(s) | M0 = [%s] | Time = %.2f ms\n', ...
    sum(b_rob{1}), num2str(b_rob{1}'), toc*1000);

%% =========================================================================
%  LOCAL FUNCTIONS
%  =========================================================================
function MO = inverse_observation(sigma, Post, C, np)
    n = numel(sigma);
    M_p = zeros(np, 1);
    for i = 1:n
        tj = sigma(n - i + 1);
        M_p = max(M_p, Post(:, tj)) - C(:, tj);
    end
    MO = M_p;
end

function [best, ns] = exact_algorithm(word, ~, Post, C, labels, np, nt)
    store = containers.Map('KeyType', 'char', 'ValueType', 'any');
    store(mat2str(zeros(1, nt))) = {zeros(np, 1)};
    for step = 1:numel(word)
        lbl = word{numel(word) - step + 1};
        ns2 = containers.Map('KeyType', 'char', 'ValueType', 'any');
        for ki = keys(store)
            y = str2num(ki{1}); %#ok<ST2NM>
            ml = store(ki{1});
            for t = find(strcmp(labels, lbl))
                yn = y; yn(t) = yn(t) + 1;
                for mi = 1:numel(ml)
                    mn = max(ml{mi}, Post(:, t)) - C(:, t);
                    ns2 = insert_pareto(ns2, mat2str(yn), mn);
                end
            end
        end
        store = ns2;
    end
    [best, ns] = extract_mim(store);
end

function [best, ns] = heuristic_algorithm(word, ~, Post, C, labels, np, nt, c_p, d_p)
    Sb = max(1, floor(numel(word) / d_p));
    store = containers.Map('KeyType', 'char', 'ValueType', 'any');
    store(mat2str(zeros(1, nt))) = {zeros(np, 1)};
    for step = 1:numel(word)
        lbl = word{numel(word) - step + 1};
        ns2 = containers.Map('KeyType', 'char', 'ValueType', 'any');
        for ki = keys(store)
            y = str2num(ki{1}); %#ok<ST2NM>
            ml = store(ki{1});
            for t = find(strcmp(labels, lbl))
                yn = y; yn(t) = yn(t) + 1;
                for mi = 1:numel(ml)
                    mn = max(ml{mi}, Post(:, t)) - C(:, t);
                    ns2 = insert_pareto(ns2, mat2str(yn), mn);
                end
            end
        end
        store = ns2;
        if mod(step, Sb) == 0 && store.Count > 0
            all_m = collect_all(store);
            sums = cellfun(@sum, all_m);
            thr = min(sums) + c_p;
            for ki = keys(store)
                bkt = store(ki{1});
                kept = bkt(cellfun(@(m) sum(m) <= thr, bkt));
                if isempty(kept)
                    remove(store, ki{1});
                else
                    store(ki{1}) = kept;
                end
            end
        end
    end
    [best, ns] = extract_mim(store);
end

function store = insert_pareto(store, key, mn)
    if ~isKey(store, key)
        store(key) = {mn};
        return;
    end
    bkt = store(key);
    dom = false;
    rm = [];
    for i = 1:numel(bkt)
        if all(bkt{i} <= mn)
            dom = true;
            break;
        elseif all(mn <= bkt{i})
            rm(end+1) = i; %#ok<AGROW>
        end
    end
    if ~dom
        bkt(rm) = [];
        bkt{end+1} = mn;
        store(key) = bkt;
    end
end

function [best, n] = extract_mim(store)
    all_m = collect_all(store);
    if isempty(all_m)
        best = {}; n = 0; return;
    end
    s = cellfun(@sum, all_m);
    mt = min(s);
    best = all_m(s == mt);
    n = numel(best);
end

function all_m = collect_all(store)
    all_m = {};
    for ki = keys(store)
        bkt = store(ki{1});
        for i = 1:numel(bkt)
            all_m{end+1} = bkt{i}; %#ok<AGROW>
        end
    end
end
