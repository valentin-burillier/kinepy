import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import cm
import matplotlib.colors as mcol

from kinepy.units import *
from kinepy.interface.decorators import physics_input_function, physics_output, physics_input_function_variable

# ---------------------------------------------------- Animate -------------------------------------------------------------

global C
# C = cm.get_cmap('coolwarm') # plasma, coolwarm, bwr, rainbow, OrRd
C = mcol.LinearSegmentedColormap.from_list("", ["cyan", "r"])


def animate(list_paths, list_vectors=None, anim_time=4, repeat=True, scale=1, vector_scale=0.1, magnitude_vector='proportionnal'):
    """
    magnitude_vector : 'proportionnal', 'log', 'unitary'
    """
    if isinstance(list_paths[0], np.ndarray):
        list_paths = [list_paths]
        
    M = []
    x_min, x_max, y_min, y_max = [], [], [], []
    for path in list_paths:
        A = np.array(path)
        nan = np.any(np.isnan(A), axis=(1, 0))
        A[:, :, nan] = 0
        M.append(A.transpose((2, 1, 0)))
        x_min.append(A[:, 0].min())
        x_max.append(A[:, 0].max())
        y_min.append(A[:, 1].min())
        y_max.append(A[:, 1].max())

    x_min, x_max, y_min, y_max = min(x_min), max(x_max), min(y_min), max(y_max)
    dx = (x_max - x_min)*0.05
    dy = (y_max - y_min)*0.05

    fig = plt.figure(figsize=((x_max - x_min)/(y_max - y_min)*scale*7, scale*7))
    ax = plt.axes(xlim=(x_min - dx, x_max + dx), ylim=(y_min - dy, y_max + dy))
    lines = [ax.plot([], [], lw=2, color='tab:blue')[0] for _ in range(len(M))]
    
    if list_vectors is not None:
        if isinstance(list_vectors[0], np.ndarray):
            list_vectors = [list_vectors]
        PAs, vecs = [], []
        for PA_, vec_ in list_vectors:
            PA, vec = PA_.copy(), vec_.copy()
            nan = np.any(np.isnan(PA), axis=0)
            PA[:, nan] = 0
            PAs.append(PA)
            nan = np.any(np.isnan(vec), axis=0)
            vec[:, nan] = 0
            vecs.append(vec)
        PAs = np.transpose(PAs, (2 ,1, 0))
        vecs = np.transpose(vecs, (2 ,1, 0))
        magnitudes = np.sqrt(np.sum(vecs**2, axis=1)) # linalg
        colors = magnitudes/np.max(magnitudes)
        color_array = np.array([C(color) for color in colors])
        
        if magnitude_vector == 'log':
            a = np.log(magnitudes + 1)/magnitudes
            vecs *= a[:, np.newaxis, :]
        elif magnitude_vector == 'unitary':
            vecs /= magnitudes[:, np.newaxis, :]
        
        n = PAs.shape[2]
        lines.append(ax.quiver([10**10]*n,[10**10]*n, [0]*n, [0]*n, scale=1/vector_scale, scale_units='xy', cmap='plasma'))

        def _animate_(i):
            for lnum, line in enumerate(lines[:-1]):
                line.set_data(M[lnum][i, 0], M[lnum][i, 1])
            lines[-1].set_UVC(vecs[i, 0], vecs[i, 1])
            lines[-1].set_offsets(PAs[i].T)
            lines[-1].set_facecolor(color_array[i])
            return lines
    else:
        def _animate_(i):
            for lnum, line in enumerate(lines):
                line.set_data(M[lnum][i, 0], M[lnum][i, 1])
            return lines
    
    anim = animation.FuncAnimation(fig, _animate_, frames=(n:=len(M[0])), interval=anim_time*1000/n, blit=True, repeat=repeat)    
    
    return anim
# ---------------------------------------------------- Inputs ----------------------------------------------------------


@physics_input_function_variable(VARIABLE_UNIT, VARIABLE_UNIT, TIME, '', VARIABLE_DERIVATIVE)
def direct_input(a, b, t, n=101, v_max=None):
    if v_max is not None and abs((b - a) / t) > v_max:
        raise ValueError("Speed is too high")
    return np.linspace(a, b, n)


@physics_input_function_variable(VARIABLE_UNIT, VARIABLE_UNIT, TIME, '', VARIABLE_DERIVATIVE, VARIABLE_SECOND_DERIVATIVE)
def trapezoidal_input(a, b, t, n=101, v_max=None, a_max=None):
    time_line = np.linspace(0, t, n)
    v = 2 * (b - a) / t

    # triangle case
    if v_max is None or abs(v) <= v_max:
        acc = 4 * (b - a) / (t * t)
        if a_max is not None and abs(acc) > a_max:
            raise ValueError("Too much Acceleration")
        t0, t1 = time_line[:n//2], time_line[n//2:]
        return np.r_[a + .5 * acc * t0 * t0, b - .5 * acc * (t1 - t) ** 2]

    # trapezoid case
    if 2 * v_max < abs(v):
        raise ValueError("Impossible input")
    v = np.sign(v) * v_max
    acc = v_max / (t_acc := t - (b - a) / v)
    if a_max is not None and abs(acc) > a_max:
        raise ValueError("Too much acceleration")
    cutting_index = int(n * t_acc / t)
    if cutting_index:
        t0, t1, t2 = time_line[:cutting_index], time_line[cutting_index:-cutting_index], time_line[-cutting_index:]
    else:
        t0, t1, t2 = np.array([], float), time_line, np.array([], float)
    x0 = a + .5 * acc * t_acc * t_acc
    return np.r_[a + .5 * acc * t0 * t0, x0 + v * (t1 - t_acc), b - .5 * acc * (t2 - t) ** 2]


@physics_input_function_variable(VARIABLE_UNIT, VARIABLE_UNIT, TIME, '', VARIABLE_DERIVATIVE, VARIABLE_SECOND_DERIVATIVE)
def sinusoidal_input(a, b, t, n=101, v_max=None, a_max=None):
    time_line = np.linspace(0, t, n)
    v = 2 * (b - a) / t

    # "triangle" case
    if v_max is None or abs(v) <= v_max:
        acc = np.pi * v / t
        if a_max is not None and abs(acc) > a_max:
            raise ValueError("Too much Acceleration")
        omega = 2 * np.pi / t
        return a + .5 * v * (time_line - np.sin(time_line * omega) / omega)

    # "trapezoid" case
    if 2 * v_max < abs(v):
        raise ValueError("Impossible input")
    v = np.sign(v) * v_max
    acc = .5 * np.pi * v_max / (t_acc := t - (b - a) / v)
    if a_max is not None and abs(acc) > a_max:
        raise ValueError("Too much acceleration")

    cutting_index = int(n * t_acc / t)
    if cutting_index:
        t0, t1, t2 = time_line[:cutting_index], time_line[cutting_index:-cutting_index], time_line[-cutting_index:]
    else:
        t0, t1, t2 = np.array([], float), time_line, np.array([], float)
    omega, x0 = np.pi / t_acc, a + .5 * v * t_acc
    return np.r_[a + .5 * v * (t0 - np.sin(omega * t0) / omega), x0 + v * (t1 - t_acc), b + .5 * v * (t2 - t - np.sin(omega * (t2 - t)) / omega)]


# ---------------------------------------------------- Geometry --------------------------------------------------------


def distance(p1, p2):
    return np.linalg.norm(p2 - p1, axis=0)


def norm(v):
    return np.linalg.norm(v, axis=0)

# ---------------------------------------------------- Derivative ------------------------------------------------------


def derivative(obj, t, *, phy=LENGTH):
    if phy not in DERIVATIVES:
        raise ValueError(f"This unit {phy} has no derivative ready to use")
    if len(obj.shape) == 1:
        return np.diff(obj * SYSTEM[phy], append=np.nan) * len(obj) / SYSTEM[TIME] / SYSTEM[DERIVATIVES[phy]] / t
    return np.diff(obj * SYSTEM[phy], axis=1, append=np.nan) * obj.shape[1] / SYSTEM[TIME] / SYSTEM[DERIVATIVES[phy]] / t


def second_derivative(obj, t, *, phy=LENGTH):
    if phy not in DERIVATIVES:
        raise ValueError(f"This unit {phy} has no derivative ready to use")
    if DERIVATIVES[phy] not in DERIVATIVES:
        raise ValueError(f"This unit {phy} has no second derivative ready to use")
    second = DERIVATIVES[DERIVATIVES[phy]]
    if len(obj.shape) == 1:
        return np.diff(obj * SYSTEM[phy], append=np.nan) / SYSTEM[second] * (len(obj) / SYSTEM[TIME] / t) ** 2
    return np.diff(obj * SYSTEM[phy], 2, axis=1, prepend=np.nan, append=np.nan) / SYSTEM[second] * (obj.shape[1] / SYSTEM[TIME] / t ** 2)


# ---------------------------------------------------- Mass ------------------------------------------------------------

@physics_output(MASS)
@physics_input_function(DENSITY, LENGTH, LENGTH)
def cylinder_mass(rho, d, h):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return rho*h*np.pi/4*d**2


def round_rod_mass(rho, d, l):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return cylinder_mass(rho, d, l)


@physics_output(MASS)
@physics_input_function(DENSITY, LENGTH, LENGTH, LENGTH)
def hollow_cylinder_mass(rho, d, h, t):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return rho*h*np.pi*t*(d - t)


def round_pipe_mass(rho, d, l, t):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return hollow_cylinder_mass(rho, d, l, t)


@physics_output(MASS)
@physics_input_function(DENSITY, LENGTH, LENGTH, LENGTH)
def parallelepiped_mass(rho, l, w, h):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return rho*l*w*h


@physics_output(MASS)
@physics_input_function(DENSITY, LENGTH, LENGTH, LENGTH, LENGTH)
def rectangular_tube_mass(rho, l, w, h, t):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return 2*rho*l*t*(h + w - 2*t)


@physics_output(MASS)
@physics_input_function(DENSITY, LENGTH)
def ball_mass(rho, d):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return rho/6*np.pi*d**3


# ---------------------------------------------------- Inertia ---------------------------------------------------------

@physics_output(INERTIA)
@physics_input_function(MASS, LENGTH)
def cylinder_inertia(m, d):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return m*d**2/8


@physics_output(INERTIA)
@physics_input_function(MASS, LENGTH, LENGTH)
def hollow_cylinder_inertia(m, d, t):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return m/4*(d**2 - 2*t*d + 2*t**2)


@physics_output(INERTIA)
@physics_input_function(MASS, LENGTH, LENGTH)
def round_rod_inertia(m, d, l):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return m/4*(d**2/4 + l**2/3)


@physics_output(INERTIA)
@physics_input_function(MASS, LENGTH, LENGTH, LENGTH)
def round_pipe_inertia(m, d, l, t):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return m/4*((d**2 - 2*t*d + 2*t**2)/2 + l**2/3) # (d**2 + (d - 2*t)**2)/4


@physics_output(INERTIA)
@physics_input_function(MASS, LENGTH, LENGTH)
def parallelepiped_inertia(m, l, w):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return m/12*(l**2 + w**2)


@physics_output(INERTIA)
@physics_input_function(MASS, LENGTH, LENGTH, LENGTH, LENGTH)
def rectangular_tube_inertia(m, l, w, h, t):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return m/12*(w*h*2*(w - t)/(w + h - 2*t) + (w - 2*t)**2 + l**2)


@physics_output(INERTIA)
@physics_input_function(MASS, LENGTH)
def ball_inertia(m, d):
    """
    See the correspondence of the arguments here:
        
    https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masseinertie
    """
    return m/10*d**2
