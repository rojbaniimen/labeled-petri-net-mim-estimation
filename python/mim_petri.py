"""
Minimal Initial Marking (MIM) Estimation in Labeled Petri Nets
Exact (Pareto-front) & Heuristic (Periodic Pruning) Algorithms with Robotic Surveillance Application

Author: Imen Rojbani (Electrical Engineering Student, ENSIT)
Academic Module: Modeling of Complex Systems (2025-2026)
"""

import time
import numpy as np

# ==============================================================================
# 1. BENCHMARK NETWORK DEFINITION (Figure 1)
# ==============================================================================
# Transitions: t1(a), t2(b), t3(a), t4(c)
Pre = np.array([
    [1, 1, 0, 0],  # P1
    [0, 0, 1, 0],  # P2
    [0, 0, 1, 0],  # P3
    [0, 0, 0, 1],  # P4
    [0, 0, 0, 0],  # P5
], dtype=int)

Post = np.array([
    [0, 0, 0, 0],  # P1
    [0, 1, 0, 0],  # P2
    [0, 1, 0, 0],  # P3
    [0, 0, 1, 0],  # P4
    [1, 0, 0, 1],  # P5
], dtype=int)

C = Post - Pre
n_places, n_transitions = Pre.shape

labels = {0: 'a', 1: 'b', 2: 'a', 3: 'c'}
word = ['a', 'b', 'a', 'c']


def transitions_for_label(lbl, lbl_map=labels):
    """Returns all transition indices matching the observed label."""
    return [t for t, l in lbl_map.items() if l == lbl]


def _insert_pareto(store, y_key, m0_new):
    """Inserts m0_new into store[y_key] while maintaining the non-dominated Pareto front."""
    if y_key not in store:
        store[y_key] = [list(m0_new)]
        return

    bucket = store[y_key]
    dominated = False
    to_remove = []

    for i, m_ex in enumerate(bucket):
        if all(e <= n for e, n in zip(m_ex, m0_new)):
            dominated = True
            break
        elif all(n <= e for n, e in zip(m0_new, m_ex)):
            to_remove.append(i)

    if not dominated:
        for i in reversed(to_remove):
            bucket.pop(i)
        bucket.append(list(m0_new))


# ==============================================================================
# 2. ALGORITHM 1: INVERSE OBSERVATION (Exact Transition Sequence Known)
# ==============================================================================
def inverse_observation(sigma, post_mat=Post, c_mat=C, np_count=n_places):
    """Reconstructs the minimal initial marking M0 for an EXACT transition sequence sigma."""
    n = len(sigma)
    m_p = np.zeros(np_count, dtype=int)
    for j in range(n - 1, -1, -1):
        tj = sigma[j]
        m_pp = np.maximum(m_p, post_mat[:, tj])
        m_p = m_pp - c_mat[:, tj]
    return m_p


# ==============================================================================
# 3. ALGORITHM 2: EXACT METHOD (Pareto-Front Backward Exploration)
# ==============================================================================
def exact_algorithm(obs_word, post_mat=Post, c_mat=C, lbl_map=labels):
    """Computes all Minimal Initial Markings (MIMs) compatible with label sequence obs_word."""
    k = len(obs_word)
    np_count, nt_count = post_mat.shape
    store = {tuple([0] * nt_count): [list([0] * np_count)]}

    for step in range(k):
        lbl = obs_word[k - 1 - step]
        new_store = {}
        for y, m0_list in store.items():
            for t in transitions_for_label(lbl, lbl_map):
                yn = list(y)
                yn[t] += 1
                yn = tuple(yn)
                for m0 in m0_list:
                    mn = tuple(
                        int(v)
                        for v in (np.maximum(np.array(m0, dtype=int), post_mat[:, t]) - c_mat[:, t])
                    )
                    _insert_pareto(new_store, yn, mn)
        store = new_store

    sols = [m0 for lst in store.values() for m0 in lst]
    mt = min(sum(m) for m in sols)
    return [m for m in sols if sum(m) == mt], sols


# ==============================================================================
# 4. ALGORITHM 3: HEURISTIC APPROACH (Periodic State-Space Pruning)
# ==============================================================================
def heuristic_algorithm(obs_word, post_mat=Post, c_mat=C, lbl_map=labels, c_param=1, d_param=2):
    """Approximates MIMs using periodic pruning every Sb = floor(k / d) steps."""
    k = len(obs_word)
    sb = max(1, k // d_param)
    np_count, nt_count = post_mat.shape
    store = {tuple([0] * nt_count): [list([0] * np_count)]}

    for step in range(k):
        lbl = obs_word[k - 1 - step]
        new_store = {}
        for y, m0_list in store.items():
            for t in transitions_for_label(lbl, lbl_map):
                yn = list(y)
                yn[t] += 1
                yn = tuple(yn)
                for m0 in m0_list:
                    mn = tuple(
                        int(v)
                        for v in (np.maximum(np.array(m0, dtype=int), post_mat[:, t]) - c_mat[:, t])
                    )
                    _insert_pareto(new_store, yn, mn)
        store = new_store

        # Periodic pruning
        if (step + 1) % sb == 0 and store:
            all_s = [sum(m) for lst in store.values() for m in lst]
            thr = min(all_s) + c_param
            store = {y: [m for m in lst if sum(m) <= thr] for y, lst in store.items()}
            store = {y: lst for y, lst in store.items() if lst}

    sols = [m0 for lst in store.values() for m0 in lst]
    mt = min(sum(m) for m in sols)
    return [m for m in sols if sum(m) == mt], sols


# ==============================================================================
# 5. ROBOTIC SURVEILLANCE APPLICATION (3x3 Grid, 9 Places, 24 Transitions)
# ==============================================================================
def run_robotic_surveillance():
    sub_labels = {0: 'f', 1: 'e', 2: 'b', 3: 'c', 4: 'a', 5: 'f', 6: 'd', 7: 'e', 8: 'd'}
    adj = [
        (0, 1), (0, 3), (1, 2), (1, 4), (2, 5), (3, 4),
        (3, 6), (4, 5), (4, 7), (5, 8), (6, 7), (7, 8)
    ]
    transitions3 = []
    pre3 = np.zeros((9, 24), dtype=int)
    post3 = np.zeros((9, 24), dtype=int)

    for i, j in adj:
        for src, dst in [(i, j), (j, i)]:
            idx = len(transitions3)
            transitions3.append({'src': src, 'dst': dst, 'label': sub_labels[dst]})
            pre3[src, idx] = 1
            post3[dst, idx] = 1

    c3 = post3 - pre3
    lbl_map3 = {idx: tr['label'] for idx, tr in enumerate(transitions3)}
    w3 = list('fbebaedfcbdbcbabcbcd')

    t0 = time.perf_counter()
    best_rob, all_rob = heuristic_algorithm(w3, post_mat=post3, c_mat=c3, lbl_map=lbl_map3, c_param=1, d_param=2)
    dt = (time.perf_counter() - t0) * 1000
    print(f"\n[Robotic Surveillance 3x3] w length={len(w3)}")
    print(f"Heuristic (c=1, d=2): MIM = {sum(best_rob[0])} token(s) | M0 = {best_rob[0]} | Time = {dt:.2f} ms")


if __name__ == "__main__":
    print("=== BENCHMARK NETWORK (Figure 1: w = 'abac') ===")
    m0_1 = inverse_observation([0, 1, 2, 3])
    print(f"Alg 1 (Inverse Obs): M0 = {list(m0_1)} | tokens = {sum(m0_1)}")

    t_start = time.perf_counter()
    best2, all2 = exact_algorithm(word)
    dt2 = (time.perf_counter() - t_start) * 1000
    print(f"Alg 2 (Exact Pareto): M0 = {best2[0]} | tokens = {sum(best2[0])} ({len(all2)} Pareto sol, {dt2:.2f} ms)")

    best3, all3 = heuristic_algorithm(word, c_param=1, d_param=2)
    print(f"Alg 3 (Heuristic c=1,d=2): M0 = {best3[0]} | tokens = {sum(best3[0])} ({len(all3)} Pareto sol)")

    print("\n--- Heuristic Parameter Sensitivity (c, d) ---")
    print(f"{'c':>3} {'d':>3} {'Sb':>4} {'#sol':>6} {'MIM':>5}")
    for c_v, d_v in [(0, 1), (0, 2), (1, 1), (1, 2), (1, 4), (2, 2)]:
        bh, ah = heuristic_algorithm(word, c_param=c_v, d_param=d_v)
        sb = max(1, len(word) // d_v)
        print(f"{c_v:>3} {d_v:>3} {sb:>4} {len(ah):>6} {sum(bh[0]):>5}")

    run_robotic_surveillance()
