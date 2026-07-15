#!/usr/bin/env python3
"""
analyze_pendulum_sysid.py

Offline system identification of dry (Coulomb/static) friction and viscous
damping from the automated free-release angle sweep produced by
pendulum_release_experiment.py (angle_target_deg, trial, t_release_s,
timestamp_unix, pos_rad, pos_deg, vel_rad_s, torque_Nm, temp_C).

WHY THIS METHOD (read this before trusting the numbers)
---------------------------------------------------------------------------
The classic approach for this kind of experiment is "peak-envelope decay":
find successive oscillation peaks after release and see how fast their
amplitude shrinks (exponential decay -> viscous; constant absolute
decrement each half-swing -> Coulomb). That method needs the pendulum to
actually oscillate through several cycles.

This particular dataset does not do that. Releases below ~30 degrees barely
move at all (static friction alone holds them against gravity), and *every*
trial -- even the big releases -- comes to rest at a nonzero angle, not back
at the calibrated zero. That is itself a real, useful finding (a sizeable
static-friction "dead zone"), but it also means peak-counting has almost no
data to work with: most trials never complete even one full swing back past
vertical.

So the primary method here is an ENERGY-DISSIPATION regression instead:
every trial starts at rest (theta_dot ~ 0, confirmed by the experiment
script's settle check) and ends at rest, so ALL of the potential energy
given up between release and final rest was dissipated by friction --
no oscillation counting required, and it uses every trial, including the
ones that don't move. See fit_energy_dissipation() docstring for the math.
A secondary ODE trajectory-fit method (fit_ode_trajectories()) is used as a
cross-check on the subset of trials that do swing through vertical.

Because the actual rod mass/length were never independently re-measured for
this rig (only carried over as an unverified "reference" from
src/soft_balance.py), the headline outputs are the DIMENSIONLESS ratios
mu_c = c/k_g (Coulomb friction torque / peak gravity torque) and
mu_b = b/k_g (viscous coefficient / peak gravity torque, units of seconds)
-- these don't depend on trusting the reference mass at all. Nm-scale
(c, b) and an inertia estimate are also reported, clearly marked as
depending on that unverified reference.

Usage:
    python analyze_pendulum_sysid.py --csv data/pendulum_release_sysid.csv \
        --meta data/pendulum_release_sysid_meta.json --out-dir out/
"""
import argparse
import json
import os

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import integrate, optimize

G = 9.81

# numpy >=2.0 renamed trapz -> trapezoid; support either so this runs on
# whatever numpy the user has installed.
_trapz = getattr(np, 'trapezoid', None) or np.trapz


# ---------------------------------------------------------------------------
# Loading & per-trial feature extraction
# ---------------------------------------------------------------------------

def load(csv_path, meta_path):
    df = pd.read_csv(csv_path)
    df = df.sort_values(['angle_target_deg', 'trial', 't_release_s']).reset_index(drop=True)
    with open(meta_path) as f:
        meta = json.load(f)
    return df, meta


def trial_features(df):
    """One row per (angle, trial) with the quantities the fits below need.
    Everything here comes straight from measured position/velocity -- no
    differentiation, no assumed physical parameters."""
    rows = []
    for (angle, trial), g in df.groupby(['angle_target_deg', 'trial']):
        g = g.sort_values('t_release_s')
        t = g['t_release_s'].to_numpy()
        theta = g['pos_rad'].to_numpy()
        omega = g['vel_rad_s'].to_numpy()

        theta0 = theta[0]
        # final rest position/velocity: mean of the last 15 samples (or fewer
        # if a trial logged less than that)
        tail_n = min(15, len(theta))
        theta_final = float(np.mean(theta[-tail_n:]))
        omega_tail_std = float(np.std(omega[-tail_n:]))

        # total arc length traveled (robust to oscillation, no assumption of
        # a fixed period): sum of |consecutive position deltas|
        arc_length = float(np.sum(np.abs(np.diff(theta))))

        # viscous dissipation integral: trapezoidal integral of omega^2 dt,
        # using the *measured* velocity (not a finite-difference of theta)
        visc_integral = float(_trapz(omega ** 2, t))

        crossed_zero = bool(np.any(np.sign(theta - 0.0) != np.sign(theta0)) and
                             np.any(np.abs(theta) > 0.5 * np.pi / 180))  # ignore sub-0.5deg noise

        # count velocity zero-crossings (oscillation half-cycles) as a
        # secondary diagnostic
        sign_changes = np.sum(np.diff(np.sign(omega)) != 0)

        rows.append(dict(
            angle_target_deg=angle, trial=int(trial),
            n_samples=len(g), duration_s=float(t[-1] - t[0]) if len(t) > 1 else 0.0,
            theta0_rad=float(theta0), theta0_deg=float(np.degrees(theta0)),
            theta_final_rad=theta_final, theta_final_deg=float(np.degrees(theta_final)),
            omega_tail_std=omega_tail_std,
            min_theta_deg=float(np.degrees(theta.min())), max_theta_deg=float(np.degrees(theta.max())),
            arc_length_rad=arc_length, visc_integral=visc_integral,
            crossed_zero=crossed_zero, n_vel_sign_changes=int(sign_changes),
        ))
    return pd.DataFrame(rows).sort_values(['angle_target_deg', 'trial']).reset_index(drop=True)


# ---------------------------------------------------------------------------
# Finding 1: static-friction dead zone
# ---------------------------------------------------------------------------

def analyze_dead_zone(feat):
    """Where does the pendulum end up stuck, and at what release angle does
    it first manage to swing all the way through vertical?"""
    never_cross = feat[~feat['crossed_zero']]
    always_cross_angles = sorted(feat.loc[feat['crossed_zero'], 'angle_target_deg'].unique())
    never_cross_angles = sorted(feat.loc[~feat['crossed_zero'], 'angle_target_deg'].unique())

    threshold_lo = max(never_cross_angles) if never_cross_angles else None
    threshold_hi = min(always_cross_angles) if always_cross_angles else None

    dead_zone_half_width_deg = float(feat['theta_final_deg'].abs().max())

    return dict(
        max_release_angle_deg_with_no_crossing=threshold_lo,
        min_release_angle_deg_that_always_crosses=threshold_hi,
        dead_zone_half_width_deg_lower_bound=dead_zone_half_width_deg,
        n_trials_never_crossing=int(len(never_cross)),
        n_trials_total=int(len(feat)),
    )


# ---------------------------------------------------------------------------
# Finding 2 (primary): energy-dissipation regression
# ---------------------------------------------------------------------------

def fit_energy_dissipation(feat, k_g_reference):
    """
    Every trial starts at rest at theta0 and ends at rest at theta_final.
    Mechanical energy lost between those two rest states must equal the
    work done against friction along the path:

        k_g * [cos(theta_final) - cos(theta0)]  =  c * S  +  b * V

    where k_g = m*g*(L/2) (peak gravity torque), S = total arc length
    traveled (rad), V = integral of omega^2 dt (rad^2/s), c = Coulomb
    friction torque (Nm), b = viscous coefficient (Nm*s/rad).

    Dividing by k_g gives a regression with NO dependence on k_g at all:

        X  =  mu_c * S  +  mu_b * V,   X := cos(theta_final) - cos(theta0)

    mu_c = c/k_g (dimensionless), mu_b = b/k_g (seconds). Fit by ordinary
    least squares (with and without a free intercept, as a check that the
    intercept is ~0 as the physics predicts).
    """
    X = np.cos(feat['theta_final_rad']) - np.cos(feat['theta0_rad'])
    S = feat['arc_length_rad'].to_numpy()
    V = feat['visc_integral'].to_numpy()

    # no-intercept fit (the physically-motivated model)
    A = np.column_stack([S, V])
    coef, residuals, rank, sv = np.linalg.lstsq(A, X, rcond=None)
    mu_c, mu_b = coef
    X_pred = A @ coef
    ss_res = float(np.sum((X - X_pred) ** 2))
    ss_tot = float(np.sum((X - X.mean()) ** 2))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')

    # bootstrap CI (resample trials with replacement)
    rng = np.random.default_rng(0)
    n = len(X)
    boots = []
    for _ in range(2000):
        idx = rng.integers(0, n, n)
        Ab, Xb = A[idx], X.to_numpy()[idx]
        try:
            c_b, _, _, _ = np.linalg.lstsq(Ab, Xb, rcond=None)
            boots.append(c_b)
        except np.linalg.LinAlgError:
            continue
    boots = np.array(boots)
    mu_c_ci = np.percentile(boots[:, 0], [2.5, 97.5]).tolist()
    mu_b_ci = np.percentile(boots[:, 1], [2.5, 97.5]).tolist()

    # with-intercept fit, as a physical sanity check
    A2 = np.column_stack([np.ones(n), S, V])
    coef2, *_ = np.linalg.lstsq(A2, X, rcond=None)

    result = dict(
        mu_c=float(mu_c), mu_b=float(mu_b), r2=float(r2),
        mu_c_95ci=mu_c_ci, mu_b_95ci=mu_b_ci,
        intercept_check=float(coef2[0]),  # should be close to 0
        n_trials_used=int(n),
        k_g_reference_Nm=k_g_reference,
        c_Nm_using_reference=float(mu_c * k_g_reference),
        b_Nm_s_per_rad_using_reference=float(mu_b * k_g_reference),
    )
    return result, X.to_numpy(), X_pred


# ---------------------------------------------------------------------------
# Finding 3 (cross-check): nonlinear ODE trajectory fit on swinging trials
# ---------------------------------------------------------------------------

def _ode_rhs(t, y, omega0_sq, mu_b_over_I_dummy, zeta_v, a_c, eps):
    theta, omega = y
    friction = a_c * np.tanh(omega / eps)  # smoothed Coulomb term
    domega = -omega0_sq * np.sin(theta) - zeta_v * omega - friction
    return [omega, domega]


def _simulate(params, t_eval, theta0, omega0_ic, method='RK23', rtol=1e-4, atol=1e-6):
    omega0_sq, zeta_v, a_c = params
    sol = integrate.solve_ivp(
        _ode_rhs, (t_eval[0], t_eval[-1]), [theta0, omega0_ic],
        t_eval=t_eval, args=(omega0_sq, None, zeta_v, a_c, 0.02),
        method=method, rtol=rtol, atol=atol,
    )
    if not sol.success or sol.y.shape[1] != len(t_eval):
        return None
    return sol.y[0], sol.y[1]


def fit_ode_trajectories(df, feat, min_angle_for_swing=33.0, trials_per_angle=1,
                          downsample_stride=3, max_nfev=150):
    """Fit theta''=-omega0^2 sin(theta) - zeta_v*theta' - a_c*tanh(theta'/eps)
    to a sample of trials that actually swing through vertical, by matching
    the simulated trajectory to the measured one (least squares on
    position). Only meaningful for trials with enough energy to swing --
    see the dead zone analysis for why smaller releases are excluded here.

    This is a cross-check on the (fast, closed-form) energy-dissipation fit,
    not the primary method -- each per-trial fit requires a numerical ODE
    solve inside a nonlinear least-squares loop, which is orders of
    magnitude slower than the energy method's plain linear algebra. Default
    settings (1 trial/angle, downsampled 3x, loose-but-adequate ODE
    tolerances) keep a ~20-angle sweep to well under a minute; increase
    trials_per_angle for a more thorough (slower) cross-check.
    """
    swinging = feat[feat['angle_target_deg'] >= min_angle_for_swing].copy()
    swinging = (swinging.sort_values(['angle_target_deg', 'trial'])
                .groupby('angle_target_deg', group_keys=False).head(trials_per_angle))

    fits = []
    for _, row in swinging.iterrows():
        g = df[(df.angle_target_deg == row['angle_target_deg']) & (df.trial == row['trial'])]
        g = g.sort_values('t_release_s')
        t = g['t_release_s'].to_numpy()[::downsample_stride]
        theta = g['pos_rad'].to_numpy()[::downsample_stride]
        omega = g['vel_rad_s'].to_numpy()[::downsample_stride]
        if len(t) < 15:
            continue

        def residuals(params):
            sim = _simulate(params, t, theta[0], omega[0])
            if sim is None:
                return np.full_like(theta, 1e3)
            theta_sim, _ = sim
            return theta_sim - theta

        x0 = [15.0, 0.5, 2.0]  # initial guesses: omega0^2, zeta_v, a_c
        bounds = ([0.1, 0.0, 0.0], [200.0, 20.0, 50.0])
        try:
            res = optimize.least_squares(residuals, x0, bounds=bounds, max_nfev=max_nfev)
        except Exception:
            continue
        if not res.success:
            continue
        omega0_sq, zeta_v, a_c = res.x
        rmse_deg = float(np.degrees(np.sqrt(np.mean(res.fun ** 2))))
        fits.append(dict(
            angle_target_deg=row['angle_target_deg'], trial=row['trial'],
            omega0_rad_s=float(np.sqrt(max(omega0_sq, 0))), omega0_sq=float(omega0_sq),
            zeta_v_per_s=float(zeta_v), a_c_rad_s2=float(a_c),
            rmse_deg=rmse_deg,
        ))
    return pd.DataFrame(fits)


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def make_plots(df, feat, dead_zone, energy_fit_result, X_meas, X_pred, ode_fits, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    cmap = plt.get_cmap('viridis')
    angles = sorted(df['angle_target_deg'].unique())
    norm = matplotlib.colors.Normalize(vmin=min(angles), vmax=max(angles))

    # (a) trajectory overlay, trial 1 of every 3rd angle, colored by release angle
    fig, ax = plt.subplots(figsize=(9, 5.5))
    for angle in angles[::3]:
        g = df[(df.angle_target_deg == angle) & (df.trial == 1)].sort_values('t_release_s')
        ax.plot(g['t_release_s'], g['pos_deg'], color=cmap(norm(angle)), lw=1.4)
    ax.axhline(0, color='#888888', lw=1, ls='--', zorder=0)
    ax.set_xlabel('time since release (s)')
    ax.set_ylabel('position (deg)')
    ax.set_title('Free-release trajectories (trial 1, every 3rd release angle)')
    sm = matplotlib.cm.ScalarMappable(cmap=cmap, norm=norm)
    cbar = fig.colorbar(sm, ax=ax)
    cbar.set_label('release angle (deg)')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '01_trajectories.png'), dpi=150)
    plt.close(fig)

    # (b) final rest angle vs release angle -- the dead-zone signature
    fig, ax = plt.subplots(figsize=(8, 5))
    colors = feat['crossed_zero'].map({True: cmap(0.85), False: cmap(0.15)})
    ax.scatter(feat['angle_target_deg'], feat['theta_final_deg'], c=colors, s=28, alpha=0.85,
               edgecolors='white', linewidths=0.4)
    ax.axhline(0, color='#888888', lw=1, ls='--', zorder=0)
    ax.axhspan(-dead_zone['dead_zone_half_width_deg_lower_bound'],
               dead_zone['dead_zone_half_width_deg_lower_bound'],
               color=cmap(0.15), alpha=0.08, zorder=0,
               label=f"observed dead-zone (±{dead_zone['dead_zone_half_width_deg_lower_bound']:.1f}°)")
    ax.set_xlabel('release (target) angle (deg)')
    ax.set_ylabel('final rest angle (deg)')
    ax.set_title('Where each trial comes to rest -- dark = never swung through vertical')
    ax.legend(loc='upper left', frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '02_rest_angle_vs_release.png'), dpi=150)
    plt.close(fig)

    # (c) energy regression parity plot
    fig, ax = plt.subplots(figsize=(6.5, 6))
    c = [cmap(norm(a)) for a in feat['angle_target_deg']]
    ax.scatter(X_meas, X_pred, c=c, s=26, alpha=0.85, edgecolors='white', linewidths=0.4)
    lims = [min(X_meas.min(), X_pred.min()), max(X_meas.max(), X_pred.max())]
    ax.plot(lims, lims, color='#888888', lw=1, ls='--', zorder=0, label='y = x')
    ax.set_xlabel('measured  cos(θ_final) − cos(θ₀)')
    ax.set_ylabel('predicted  μ_c·S + μ_b·V')
    ax.set_title(f"Energy-dissipation regression fit  (R² = {energy_fit_result['r2']:.4f})")
    ax.legend(loc='upper left', frameon=False)
    sm = matplotlib.cm.ScalarMappable(cmap=cmap, norm=norm)
    cbar = fig.colorbar(sm, ax=ax)
    cbar.set_label('release angle (deg)')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '03_energy_fit_parity.png'), dpi=150)
    plt.close(fig)

    # (d) phase portrait for a few large-angle swinging trials
    fig, ax = plt.subplots(figsize=(7, 6))
    for angle in [45.0, 60.0, 75.0, 90.0]:
        if angle not in angles:
            continue
        g = df[(df.angle_target_deg == angle) & (df.trial == 1)].sort_values('t_release_s')
        ax.plot(g['pos_deg'], g['vel_rad_s'], color=cmap(norm(angle)), lw=1.2, label=f'{angle:g}°')
    ax.axvline(0, color='#888888', lw=1, ls='--', zorder=0)
    ax.axhline(0, color='#888888', lw=1, ls='--', zorder=0)
    ax.set_xlabel('position (deg)')
    ax.set_ylabel('velocity (rad/s)')
    ax.set_title('Phase portraits -- spiral settles off-origin (dead zone), not at 0°')
    ax.legend(frameon=False, title='release angle')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '04_phase_portraits.png'), dpi=150)
    plt.close(fig)

    # (e) ODE trajectory fit examples
    if len(ode_fits) > 0:
        fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)
        example_angles = [a for a in [36.0, 54.0, 72.0, 90.0] if a in ode_fits['angle_target_deg'].values]
        for ax, angle in zip(axes.flat, example_angles):
            row = ode_fits[(ode_fits.angle_target_deg == angle)].iloc[0]
            g = df[(df.angle_target_deg == angle) & (df.trial == row['trial'])].sort_values('t_release_s')
            t = g['t_release_s'].to_numpy()
            theta = g['pos_rad'].to_numpy()
            omega = g['vel_rad_s'].to_numpy()
            sim = _simulate([row['omega0_sq'], row['zeta_v_per_s'], row['a_c_rad_s2']], t, theta[0], omega[0])
            ax.plot(t, np.degrees(theta), color='#333333', lw=1.6, label='measured')
            if sim is not None:
                ax.plot(t, np.degrees(sim[0]), color=cmap(0.6), lw=1.6, ls='--', label='ODE fit')
            ax.set_title(f'{angle:g}° release, trial {int(row["trial"])}  (rmse={row["rmse_deg"]:.2f}°)')
            ax.set_xlabel('t (s)')
            ax.set_ylabel('deg')
            ax.legend(frameon=False, fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, '05_ode_fit_examples.png'), dpi=150)
        plt.close(fig)

        # (f) ODE-fit parameter stability vs release angle
        fig, axes = plt.subplots(1, 3, figsize=(13, 4))
        for ax, col, label in zip(
            axes, ['omega0_rad_s', 'zeta_v_per_s', 'a_c_rad_s2'],
            ['natural frequency ω₀ (rad/s)', 'viscous rate ζ (1/s)', 'Coulomb decel. a_c (rad/s²)'],
        ):
            ax.scatter(ode_fits['angle_target_deg'], ode_fits[col], color=cmap(0.4), s=26,
                       edgecolors='white', linewidths=0.4)
            ax.set_xlabel('release angle (deg)')
            ax.set_ylabel(label)
        fig.suptitle('ODE-fit parameters vs release angle (should be roughly flat if the model fits well)')
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, '06_ode_param_stability.png'), dpi=150)
        plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Analyze pendulum_release_experiment.py output for dry friction + damping.")
    ap.add_argument('--csv', default='data/pendulum_release_sysid.csv')
    ap.add_argument('--meta', default='data/pendulum_release_sysid_meta.json')
    ap.add_argument('--out-dir', default='out')
    ap.add_argument('--swing-threshold-deg', type=float, default=None,
                     help="release angle above which trials are used for the ODE cross-check fit "
                          "(default: auto-detected from the data as the smallest angle where "
                          "all trials swing through vertical)")
    ap.add_argument('--ode-trials-per-angle', type=int, default=1,
                     help="how many trials per angle to include in the (slow) ODE trajectory-fit "
                          "cross-check; the primary energy-dissipation fit always uses all trials "
                          "regardless of this setting (default: 1)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    df, meta = load(args.csv, args.meta)
    feat = trial_features(df)
    feat.to_csv(os.path.join(args.out_dir, 'trial_features.csv'), index=False)

    dead_zone = analyze_dead_zone(feat)
    swing_threshold = args.swing_threshold_deg or dead_zone['min_release_angle_deg_that_always_crosses'] or 30.0

    k_g_ref = meta['reference_rod_mass_kg'] * G * (meta['reference_rod_length_m'] / 2.0)
    energy_result, X_meas, X_pred = fit_energy_dissipation(feat, k_g_ref)

    ode_fits = fit_ode_trajectories(df, feat, min_angle_for_swing=swing_threshold,
                                     trials_per_angle=args.ode_trials_per_angle)
    ode_fits.to_csv(os.path.join(args.out_dir, 'ode_trajectory_fits.csv'), index=False)

    # independent inertia estimate from the ODE fit's omega0 + reference k_g
    if len(ode_fits) > 0:
        omega0_med = float(ode_fits['omega0_rad_s'].median())
        I_estimate = k_g_ref / (omega0_med ** 2) if omega0_med > 0 else float('nan')
        zeta_v_med = float(ode_fits['zeta_v_per_s'].median())
        a_c_med = float(ode_fits['a_c_rad_s2'].median())
        # per-trial zeta_v/a_c trade-off diagnostic: a single ~1-swing trial
        # often can't separate viscous vs Coulomb dissipation well on its
        # own (both reduce swing amplitude similarly), so a strong negative
        # correlation here is a real identifiability warning, not noise --
        # the pooled energy-dissipation fit (Finding 2) is the one built to
        # avoid this, by combining many amplitudes where the two dissipation
        # mechanisms scale differently.
        zeta_ac_corr = float(ode_fits['zeta_v_per_s'].corr(ode_fits['a_c_rad_s2'])) if len(ode_fits) > 2 else float('nan')
        ode_summary = dict(
            n_trials_fit=int(len(ode_fits)),
            omega0_rad_s_median=omega0_med,
            zeta_v_per_s_median=zeta_v_med,
            a_c_rad_s2_median=a_c_med,
            I_estimate_kg_m2_using_reference_kg=I_estimate,
            b_Nm_s_per_rad_using_I_estimate=zeta_v_med * I_estimate,
            c_Nm_using_I_estimate=a_c_med * I_estimate,
            rmse_deg_median=float(ode_fits['rmse_deg'].median()),
            zeta_v_vs_a_c_correlation_across_trials=zeta_ac_corr,
            identifiability_caveat=(
                "Per-trial zeta_v and a_c are only weakly separable from a single "
                "~1-swing trajectory (see the correlation above and 06_ode_param_stability.png) "
                "-- trust the MEDIAN/omega0 here more than any individual trial's (zeta_v, a_c) split, "
                "and prefer Finding 2 (energy-dissipation fit) for the b vs c split itself."
            ),
        )
    else:
        ode_summary = {"note": "no trials fit successfully"}

    make_plots(df, feat, dead_zone, energy_result, X_meas, X_pred, ode_fits, args.out_dir)

    summary = dict(
        source_csv=os.path.basename(args.csv),
        source_meta=os.path.basename(args.meta),
        n_trials=len(feat),
        reference_rod_mass_kg=meta['reference_rod_mass_kg'],
        reference_rod_length_m=meta['reference_rod_length_m'],
        reference_k_g_Nm=k_g_ref,
        reference_params_caveat=meta.get('reference_params_note', ''),
        dead_zone_finding=dead_zone,
        energy_dissipation_fit=energy_result,
        ode_trajectory_fit_summary=ode_summary,
        swing_threshold_deg_used_for_ode_fit=swing_threshold,
    )
    with open(os.path.join(args.out_dir, 'sysid_summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)

    # console report
    print("=" * 78)
    print("PENDULUM DRY-FRICTION / DAMPING SYSID SUMMARY")
    print("=" * 78)
    print(f"{len(feat)} trials analyzed, release angles {feat.angle_target_deg.min():g}-"
          f"{feat.angle_target_deg.max():g} deg")
    print()
    print("FINDING 1 -- static friction dead zone")
    print(f"  Releases <= {dead_zone['max_release_angle_deg_with_no_crossing']:g} deg NEVER swing "
          f"through vertical within 5s ({dead_zone['n_trials_never_crossing']}/{dead_zone['n_trials_total']} "
          f"trials total never cross).")
    print(f"  Releases >= {dead_zone['min_release_angle_deg_that_always_crosses']:g} deg ALWAYS swing through.")
    print(f"  Observed final rest angles reach up to {dead_zone['dead_zone_half_width_deg_lower_bound']:.2f} deg "
          f"from vertical -- a lower bound on the static-friction dead-zone half-width.")
    print()
    print("FINDING 2 -- energy-dissipation regression (primary method, uses all trials)")
    print(f"  mu_c (Coulomb torque / peak gravity torque)  = {energy_result['mu_c']:.4f}  "
          f"(95% CI {energy_result['mu_c_95ci'][0]:.4f} .. {energy_result['mu_c_95ci'][1]:.4f})")
    print(f"  mu_b (viscous coeff / peak gravity torque)   = {energy_result['mu_b']:.4f} s  "
          f"(95% CI {energy_result['mu_b_95ci'][0]:.4f} .. {energy_result['mu_b_95ci'][1]:.4f})")
    print(f"  R^2 = {energy_result['r2']:.4f}   intercept check (should be ~0) = {energy_result['intercept_check']:.4f}")
    print(f"  Using the REFERENCE rod params (mass={meta['reference_rod_mass_kg']}kg, "
          f"length={meta['reference_rod_length_m']}m, k_g={k_g_ref:.4f} Nm) -- UNVERIFIED, see caveat:")
    print(f"    c (Coulomb friction torque) ~ {energy_result['c_Nm_using_reference']:.4f} Nm")
    print(f"    b (viscous coefficient)     ~ {energy_result['b_Nm_s_per_rad_using_reference']:.4f} Nm*s/rad")
    print()
    print(f"FINDING 3 -- ODE trajectory-fit cross-check ({ode_summary.get('n_trials_fit', 0)} swinging trials, "
          f">= {swing_threshold:g} deg release)")
    if 'omega0_rad_s_median' in ode_summary:
        print(f"  omega0 (natural frequency, no mass assumption) = {ode_summary['omega0_rad_s_median']:.3f} rad/s")
        print(f"  zeta_v (viscous rate)  = {ode_summary['zeta_v_per_s_median']:.3f} 1/s")
        print(f"  a_c (Coulomb decel.)   = {ode_summary['a_c_rad_s2_median']:.3f} rad/s^2")
        print(f"  median trajectory fit RMSE = {ode_summary['rmse_deg_median']:.2f} deg")
        print(f"  Implied inertia (using reference k_g) I ~ {ode_summary['I_estimate_kg_m2_using_reference_kg']:.5f} kg*m^2")
        print(f"    -> b ~ {ode_summary['b_Nm_s_per_rad_using_I_estimate']:.4f} Nm*s/rad, "
              f"c ~ {ode_summary['c_Nm_using_I_estimate']:.4f} Nm  (cross-check vs Finding 2)")
    print()
    print(f"Outputs written to: {os.path.abspath(args.out_dir)}/")
    print("  trial_features.csv, ode_trajectory_fits.csv, sysid_summary.json, 01..06_*.png")
    print("=" * 78)


if __name__ == '__main__':
    main()
