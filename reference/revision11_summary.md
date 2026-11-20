# Revision 11: replaces ground-seeking distal-first rest

- Removes revision 10 distal-first strategy and low direct-fold fallback.
- Whole-mesh clearance (actual collision STL bounds for forearm/wrist/tool/fingers).
- Cartesian up/in retreat before folding; continuous seeded IK and branch checks.
- >=0.10 m protected geometry clearance; >=0.20 m outer-workspace tool clearance;
  only allow lower tool geometry inside the 0.22 m base cylinder and away from
  protected target XY (0.12 m radius). Initially low poses may only recover upward.
- Checks complete path at <=0.005 rad spacing and checks measured geometry during
  motion. Maintains existing joint, health, stall and tracking checks.
- Folded joint goal uses 0.15 rad inward limit margin; no forced motion to limits.
- No feasible clearance path => no motor command. Already-colliding start refused.
- Adds actual rest preview in execute_motion=false mode, including Ctrl+C.
- Tracking Z remains 0.300 m; rest follows the checked varying-height path.
- Model regressions reject the exact recorded ground-seeking segment; replacement
  paths verified against URDF/SRDF/STL/table for both arms. Physical trial required.
