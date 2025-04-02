from scipy.interpolate import make_interp_spline, make_smoothing_spline, make_splrep, make_splprep
from enum import Enum, auto
import numpy as np


# See https://docs.scipy.org/doc/scipy/tutorial/interpolate/smoothing_splines.html for full details
class SplineType(Enum):
    interp = auto()  # Compute the (coefficients of) interpolating B-spline https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_interp_spline.html#make-interp-spline
    smoothing = auto()  # Compute the (coefficients of) smoothing cubic spline function using lam to control the tradeoff between the amount of smoothness of the curve and its proximity to the data. In case lam is None, using the GCV criteria [1] to find it. https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_smoothing_spline.html#scipy.interpolate.make_smoothing_spline
    splrep = auto()  # Find the B-spline representation of a 1D function https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_splrep.html#scipy.interpolate.make_splrep
    splprep = auto()  # Find a smoothed B-spline representation of a parametric N-D curve https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_splprep.html#scipy.interpolate.make_splprep


def plot_spline(
    ax,
    current_t,
    control_points,
    k,
    num_fine_t_points=100,
    label=None,
    color=None,
    derivatives=0,
    bc_type=None,
    spline_type=SplineType.interp,
    derivatives_ax=None,
    mix=1,
):
    """
    Plot control points and current position at t.

    Parameters:
    -----------
    ax: Axis to plot on
    current_t: current time point
    control_points: Array of control points [x, y, t]
    k: degree of BSpline to plot

    """
    if len(control_points) < k:
        return

    if derivatives_ax is None:
        derivatives_ax = ax

    if not label:
        if spline_type == SplineType.interp:
            label = f'InterpSpline {k=}'
        elif spline_type == SplineType.smoothing:
            lam = k
            k = 3
            label = f'SmoothingSpline {lam=} {k=}'
        elif spline_type == SplineType.splrep:
            s = k
            k = 3
            label = f'Splrep {s=} {k=}'
        elif spline_type == SplineType.splprep:
            s = k
            k = 3
            label = f'Splprep {s=} {k=}'
        else:
            raise ValueError(f'Unknown spline type {spline_type}')
    if mix < 1:
        label += f' {mix=}'

    if bc_type:
        label += f' {bc_type=}'

    x = control_points[::, 0]
    y = control_points[::, 1]
    t = control_points[::, 2]

    t_fine_x = np.linspace(t[0], t[-1], num_fine_t_points)
    t_fine_y = t_fine_x

    if spline_type == SplineType.interp:
        spline_x = make_interp_spline(t, x, k=k, bc_type=bc_type)
        spline_y = make_interp_spline(t, y, k=k, bc_type=bc_type)
    elif spline_type == SplineType.smoothing:
        spline_x = make_smoothing_spline(t, x, lam=lam)
        spline_y = make_smoothing_spline(t, y, lam=lam)
    elif spline_type == SplineType.splrep:
        spline_x = make_splrep(t, x, s=s, k=k)
        spline_y = make_splrep(t, y, s=s, k=k)
    if spline_type == SplineType.splprep:
        _spline_x, tx = make_splprep([x, t], s=s, k=k)
        _spline_y, ty = make_splprep([y, t], s=s, k=k)

        t_fine_x = np.linspace(tx[0], tx[-1], num_fine_t_points)
        t_fine_y = np.linspace(ty[0], ty[-1], num_fine_t_points)

        class Spline:
            def __init__(self, spline):
                self.spline = spline

            def __call__(self, t):
                return self.spline(t)[0]

            def derivative(self, n):
                return Spline(self.spline.derivative(n))

        spline_x = Spline(_spline_x)
        spline_y = Spline(_spline_y)

    if mix < 1:
        _spline_x = spline_x
        _spline_y = spline_y
        line_x = make_interp_spline(t, x, k=1)
        line_y = make_interp_spline(t, y, k=1)

        def spline_x(t_fine):
            mixed = mix * _spline_x(t_fine) + (1 - mix) * line_x(t_fine)
            # print(np.array([_spline_x(t_fine), line_x(t_fine), mixed]).T)
            return mixed

        def spline_y(t_fine):
            mixed = mix * _spline_y(t_fine) + (1 - mix) * line_y(t_fine)
            # print(np.array([_spline_y(t_fine), line_y(t_fine), mixed]).T)
            return mixed

    ax.plot(spline_x(t_fine_x), spline_y(t_fine_y), label=label, color=color)

    ax.scatter(
        spline_x(current_t),
        spline_y(current_t),
        label=f'{label} (current point)',
        color=color,
    )
    if derivatives > 0 and mix == 1:
        names = {
            1: ('Velocity', 'orange'),
            2: ('Acceleration', 'blue'),
            3: ('Jerk (3rd)', 'green'),
            4: ('Snap (4th)', 'red'),
            5: ('Crackle (5th)', 'cyan'),
            6: ('Pop (6th)', 'purple'),
            7: ('Lock (7th)', 'magenta'),
            8: ('Drop (8th)', 'black'),
        }
        derivatives = min(derivatives, k)
        for d in range(1, derivatives + 1):
            name, color = names[d]
            deriv_x = spline_x.derivative(d)
            deriv_y = spline_y.derivative(d)
            _t_deriv_plot_x = t_fine_x
            _t_deriv_plot_y = t_fine_y
            if spline_type == SplineType.splprep:
                _t_deriv_plot_x = np.linspace(t[0], t[-1], num_fine_t_points)
                _t_deriv_plot_y = _t_deriv_plot_x
                raise NotImplementedError('splprep derivatives not implemented')

            derivatives_ax.plot(
                _t_deriv_plot_x,
                deriv_x(t_fine_x),
                label=f'{name} (x) {label}',
                color=color,
                linestyle='--',
            )
            derivatives_ax.plot(
                _t_deriv_plot_y,
                deriv_y(t_fine_y),
                label=f'{name} (y) {label}',
                color=color,
                linestyle=':',
            )

    return spline_x, spline_y
