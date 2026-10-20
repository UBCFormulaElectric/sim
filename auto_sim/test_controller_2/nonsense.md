Why it hangs
1. The zero warm start produces NaN Jacobians. controller.cpp:219

last_result is a zero-initialized static, so every linearization point has v = 0.
autodiff's atan2 derivative is (x·dy − y·dx)/(x² + y²), which is 0/0 = NaN at the origin (controller.cpp:115).
The harness showed rows 3–4 of both A_c and B_c as NaN. ee.exp() then makes all 144 entries NaN, so every dynamics row of A and b is NaN.
Fix:
Seed last_result by rolling out the current state.
Put a floor under v_x in the slip angle. SLIP_REGULARIZATION_SPEED_MPS already exists in constants.hpp for this.
Set qp.settings.max_iter to something small and check qp.results.info.status after controller.cpp:331, so it fails fast instead of hanging.
Bugs in the dynamics (still broken after the NaN fix)
2. F_y is given position instead of velocity. At controller.cpp:131, F_y(u(2), x(0), x(1)) passes x, y; it should be x(2), x(3). The harness shows ∂v̇_y/∂y = −124,473, meaning lateral force depends on where the car is.

3. xi_prime writes 6 values into a 7-element state. controller.cpp:133-138

psi_dot = r is missing, so every row after it shifts by one:
ψ̇ gets the yaw acceleration.
ṙ gets v_theta.
θ̇ is always 0.
So the progress reward -w_3·theta can't be affected by any control. The optimizer can't make progress, and v_theta ends up steering yaw rate.
In a Release build, Eigen's "too few coefficients" assert is compiled out, so this fails silently.
Separately, ẋ and ẏ use body-frame velocities without rotating by ψ.
4. M_z is always zero. F_y returns the same force for front-left and front-right, and for rear-left and rear-right. controller.cpp:126 computes f0 − f1 + f2 − f3 with every lever arm set to 1, so the terms cancel. It should be a·(F_fl+F_fr) − b·(F_rl+F_rr), using DIST_FRONT_AXLE_CG_m and DIST_REAR_AXLE_CG_m.

5. m = 1 and izz = 1 make the problem very badly scaled. controller.cpp:20-21 The tires produce forces of hundreds to thousands of newtons. With a 1 kg car, B_c has an entry of 62,236 m/s² per radian of steering. Use CAR_MASS_AT_CG_KG and CAR_YAW_MOMENT_INERTIA_KGM2.

Bugs in the QP setup
6. The linearization is off by one step. The transition from step i−1 to step i is written into column block i-1. But it's linearized at x_i, u_i (controller.cpp:235, :269) when it should use x_{i-1}, u_{i-1}.

7. There's no cost on the controls. H_mask only covers the state blocks (controller.cpp:158-164).

Controls appear only linearly, so H is only positive semi-definite and the solution is bang-bang. That makes convergence slow for a proximal solver like proxsuite.
u_{T-1} appears in no constraint or cost at all.
Add a small weight on the controls; this needs mask entries for the control blocks.
8. The path parameter θ is never set or wrapped.

controller.cpp:58 always sets θ to 0 instead of calling project(ve.x, ve.y).
θ is never wrapped around center_line_length, so spline_t extrapolates past the end of the spline.
9. The preconditioner is never refreshed. qp.update defaults to update_preconditioner=false. The scaling from the first call is reused even though the linearization changes every time. Minor.

Unrelated bugs I noticed
get_trajectory passes fields in the wrong order. VehicleState's constructor takes (x, y, psi, v_x, v_y, r), but it gets v_x as psi, v_y as v_x, and so on.
The slip angles leave out the yaw-rate terms (a·r/v_x and b·r/v_x).