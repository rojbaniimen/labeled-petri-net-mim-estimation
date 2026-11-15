<div align="center">

# 🤖 Minimal Initial Marking (MIM) Estimation in Labeled Petri Nets
### *Exact (Pareto-Front) & Heuristic State-Space Pruning Algorithms with Robotic Surveillance Application*

[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2024a+-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/)
[![Discrete Event Systems](https://img.shields.io/badge/Domain-Labeled%20Petri%20Nets-2E7D32?style=for-the-badge)]()
[![Robotics](https://img.shields.io/badge/Application-Robotic%20Surveillance-FF6F00?style=for-the-badge)]()

**Author:** **Imen Rojbani** — *Electrical Engineering Student (ENSIT)*  
*Academic Module: Modeling of Complex Systems (2025–2026)*[cite: 9]

</div>

---

## 🎯 Problem Statement & Engineering Motivation

In partially observable discrete-event systems—such as **autonomous mobile robot navigation**, industrial manufacturing lines, and fault diagnosis—sensors generate sequences of observable labels $w = l_1 l_2 \dots l_k$ without revealing the exact initial state $M_0$ or distinguishing transitions sharing the same label ($L(t_i) = L(t_j)$)[cite: 11, 28].

> **Core Algorithmic Challenge:** *How can we compute the Minimal Initial Marking ($M_0 \in \mathbb{N}^{\vert{}P\vert{}}$) capable of firing a sequence compatible with $w$, while preventing combinatorial state-space explosion on large robotic networks?*[cite: 11, 27, 28]

This repository implements, benchmarks, and cross-validates **three complementary algorithms** in both **Python (NumPy)** and **MATLAB**[cite: 11, 13, 28]:
1. **Inverse Observation ($O(\vert{}\sigma\vert{} \cdot m)$):** Exact backward reconstruction when the fired transition sequence $\sigma$ is unambiguously known[cite: 11, 13, 28].
2. **Exact Backward Exploration (Pareto Front):** Exhaustive exploration of all label-compatible firing vectors $\vec{y}$, retaining only Pareto-non-dominated candidate markings[cite: 11, 16].
3. **Periodic Pruning Heuristic ($S_b = \lfloor k/d \rfloor$):** Bounded-memory approximation discarding candidate markings exceeding $s_{\min} + c$ every $S_b$ steps[cite: 11, 20, 21].

---

## 🔬 Benchmark Network & Experimental Results ($w = \text{abac}$)

Evaluated on a 5-place, 4-transition labeled Petri net where $L(t_1) = L(t_3) = a$, $L(t_2) = b$, and $L(t_4) = c$[cite: 12]:

| Algorithm | Input Assumption | Optimality Guarantee | Computed $M_0$ | Minimal Tokens | Scalability |
| :--- | :--- | :--- | :---: | :---: | :--- |
| **1. Inverse Observation**[cite: 25, 28] | Exact sequence $\sigma$[cite: 28] | Exact (for fixed $\sigma$)[cite: 28] | `[2, 0, 0, 0, 0]`[cite: 25] | **2**[cite: 25] | High[cite: 28] |
| **2. Exact Pareto Method**[cite: 25, 28] | Label sequence $w$[cite: 28] | **Mathematically Guaranteed**[cite: 28] | `[2, 0, 0, 0, 0]`[cite: 25] | **2**[cite: 25] | Exponential (small nets)[cite: 28] |
| **3. Heuristic ($c=1, d=2$)**[cite: 25, 28] | Label sequence $w$[cite: 28] | Tunable via $(c, d)$[cite: 28] | `[2, 0, 0, 0, 0]`[cite: 25] | **2**[cite: 25] | **High (large nets)**[cite: 28] |

---

## 🤖 Robotic Surveillance Case Study ($3 \times 3$ Grid)

To validate real-world scalability, the framework models an autonomous surveillance robot patrolling a **$3 \times 3$ environment** (**9 places**, **24 bidirectional transitions**, observable sequence length $\vert{}w\vert{} = 20$)[cite: 26]:
* **Why Exact Methods Fail:** High label ambiguity (e.g., label `e` maps to 6 distinct transitions, `d` to 5) triggers combinatorial memory explosion during exhaustive backward search[cite: 27].
* **Heuristic Breakthrough:** With parameters $c=1, d=2$, the periodic pruning heuristic resolves the Minimal Initial Marking (**1 token** — single robot localization) in milliseconds with identical numerical outputs across Python and MATLAB[cite: 28, 33].

---

## 📂 Repository Structure

```text
labeled-petri-net-mim-estimation/
├── python/
│   └── mim_petri.py                  # Full Python/NumPy implementation & 3x3 robotic benchmark
├── matlab/
│   └── mim_petri.m                   # Full MATLAB implementation & sensitivity analysis
├── Rapport_RdP_Etiquetes_MIM.pdf     # Complete mathematical report & complexity study
└── README.md
