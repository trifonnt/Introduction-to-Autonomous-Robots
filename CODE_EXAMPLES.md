# Code Examples in the Book Repository

## Conclusion

Yes. The repository contains two explicit code-style examples inside the book text itself, and it also includes companion MATLAB/Mathematica source files that are more directly reusable for development work.

## Extracted in-book examples

### 1. Rapidly Exploring Random Trees (RRT)
Source: `/home/runner/work/Introduction-to-Autonomous-Robots/Introduction-to-Autonomous-Robots/trifonnt/Introduction-to-Autonomous-Robots/chapters/pathplanning.tex`

```text
Tree=Init(X, G, start, max_dist, t, k, goal_bias);
iteration = 0
WHILE (ElapsedTime() < t AND iteration < k
AND NoGoalFound(Tree,G)) DO:
 iteration = iteration + 1
 IF RandomPercentage() < goal_bias THEN
  q_rand = SampleRandomGoal(G);
 ELSE
  q_rand = SampleRandomState(X);
 ENDIF
 q_nearest = NearestVertex(q_rand)
 q_new = Extend(q_nearest, q_rand, max_dist)
 edge = CreatePath(q_nearest, q_new);
 IF IsAllowablePath(edge) THEN
  Tree.addVertex(q_new);
  Tree.addEdge(edge);
 ENDIF
ENDWHILE
return Tree
```

Assessment: useful as algorithmic pseudocode, but not directly executable without implementing the helper functions and data structures.

### 2. Bayes Filter / Markov Localization update loop
Source: `/home/runner/work/Introduction-to-Autonomous-Robots/Introduction-to-Autonomous-Robots/trifonnt/Introduction-to-Autonomous-Robots/chapters/localization.tex`

```text
BayesFilter(Belief Bel, Data d, Set of States X):
  while d is not empty:
    c = 0
    if (d[0] is a sensor measurement):
      z = d.pop(0)
      for all x ∈ X:
        Bel'(x) = P(z|x)Bel(x)
        c += Bel'(x)
      for all x ∈ X:
        Bel'(x) = c^-1*Bel'(x)
    elif (d[0] is an action):
      u = d.pop(0)
      for all x ∈ X:
        Bel'(x) = Σ_x_(t-1) P(x|u,x_(t-1))*Bel(x_(t-1))
    Bel = Bel'
```

Assessment: useful as algorithmic pseudocode, but not directly executable without choosing concrete state, motion, and sensor models.

## Development-usable companion code

The repository also contains reusable source files outside the book chapters:

- `matlab/markovloc.m` — interactive grid-based Markov localization example
- `matlab/pf.m` — particle filter example
- `matlab/RANSAC.m` — RANSAC-based line fitting over sample sensor data
- `matlab/line_fitting.m`, `matlab/line_fitting_US.m`, `matlab/line_fitting_opt.m` — line fitting examples
- `matlab/planar_arm.m`, `matlab/hw3_ik.m`, `matlab/feedbackcontrol.m` — manipulator kinematics and inverse-kinematics examples
- `matlab/Gauss2D.m`, `matlab/linear_combinations_of_Gaussians.m` — Gaussian modeling examples
- `mathematica/inversekinematics.nb` — inverse-kinematics notebook

## Practical answer

- If you need directly runnable examples for development, use the files in `matlab/` and `mathematica/`.
- If you need examples from the book text itself, the repository mainly provides pseudocode rather than production-ready source code.
