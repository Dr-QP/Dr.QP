from enum import auto, Enum
from typing import Callable

import numpy as np
from scipy.interpolate import make_interp_spline, make_smoothing_spline, make_splprep, make_splrep


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

    Parameters
    ----------
    ax: Axis to plot on
    current_t: current time point
    control_points: Array of control points [x, y, t]
    k: degree of BSpline to plot

    """
    if derivatives_ax is None:
        derivatives_ax = ax

    lam = k
    s = k
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

    # Create a wrapper class to ensure consistent callable interface
    class SplineWrapper:
        def __init__(self, spline):
            self._spline = spline

        def __call__(self, t_fine):
            return self._spline(t_fine)

        def derivative(self, n):
            return SplineWrapper(self._spline.derivative(n))

    if spline_type == SplineType.interp:
        spline_x = SplineWrapper(make_interp_spline(t, x, k=k, bc_type=bc_type))
        spline_y = SplineWrapper(make_interp_spline(t, y, k=k, bc_type=bc_type))
    elif spline_type == SplineType.smoothing:
        spline_x = SplineWrapper(make_smoothing_spline(t, x, lam=lam))
        spline_y = SplineWrapper(make_smoothing_spline(t, y, lam=lam))
    elif spline_type == SplineType.splrep:
        spline_x = SplineWrapper(make_splrep(t, x, s=s, k=k))
        spline_y = SplineWrapper(make_splrep(t, y, s=s, k=k))
    elif spline_type == SplineType.splprep:
        _spline_x, tx = make_splprep([x, t], s=s, k=k)
        _spline_y, ty = make_splprep([y, t], s=s, k=k)

        t_fine_x = np.linspace(tx[0], tx[-1], num_fine_t_points)
        t_fine_y = np.linspace(ty[0], ty[-1], num_fine_t_points)

        spline_x = SplineWrapper(_spline_x)
        spline_y = SplineWrapper(_spline_y)
    else:
        raise ValueError(f'Unknown spline type {spline_type}')

    if mix < 1:
        line_x = SplineWrapper(make_interp_spline(t, x, k=1))
        line_y = SplineWrapper(make_interp_spline(t, y, k=1))

        class MixedSplineWrapper:
            def __init__(self, spline_func: Callable, line_func: Callable, mix_ratio):
                self._spline_func = spline_func
                self._line_func = line_func
                self._mix_ratio = mix_ratio

            def __call__(self, t_fine):
                return self._mix_ratio * self._spline_func(t_fine) + (
                    1.0 - self._mix_ratio
                ) * self._line_func(t_fine)

            def derivative(self, _n):
                # For mixed splines, derivatives are not implemented
                raise NotImplementedError('Derivatives not implemented for mixed splines')

        spline_x = MixedSplineWrapper(spline_x, line_x, mix)
        spline_y = MixedSplineWrapper(spline_y, line_y, mix)

    assert callable(spline_x), f'spline_x is not callable: {type(spline_x)}'
    assert callable(spline_y), f'spline_y is not callable: {type(spline_y)}'
    ax.plot(
        spline_x(t_fine_x),  # codeql[py/call-to-non-callable]
        spline_y(t_fine_y),  # codeql[py/call-to-non-callable]
        label=label,
        color=color,
    )

    ax.scatter(
        spline_x(current_t),  # codeql[py/call-to-non-callable]
        spline_y(current_t),  # codeql[py/call-to-non-callable]
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
