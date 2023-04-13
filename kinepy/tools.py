import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import cm
import matplotlib.colors as mcol

from kinepy.units import *
from kinepy.interface.decorators import physics_input_function, physics_output

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

# ---------------------------------------------------- Inputs -------------------------------------------------------------

def __direct_input(a, b, t, n, v_max):
    v = (b - a)/t
    if v_max is not None and abs(v) > v_max: # triangle
        print('speed too high')
        return
    return np.linspace(a, b, n)

def __trapezoidal_input(a, b, t, n, v_max, a_max):
    v = 2*(b - a)/t
    if v_max is None or abs(v) <= v_max: # triangle
        acc = 4*(b - a)/t**2
        if a_max is not None and abs(acc) > a_max:
            print('too much acceleration')
            return
        T = np.linspace(0, t, n)
        l_acc = a + acc/2*T[:n//2]**2
        l_dec = 2*a - b + acc*T[n//2:]*(t - T[n//2:]/2)
        return np.r_[l_acc, l_dec]
    
    if v_max*t < abs(b - a):
        print('input not possible')
        return
    # trapèze
    v = np.sign(v)*v_max
    t_inf = t - (b - a)/v
    acc = v/t_inf
    if a_max is not None and abs(acc) > a_max:
        print('too much acceleration')
        return
    T = np.linspace(0, t, n)
    i = int(t_inf/t*n)
    l_acc = a + acc/2*T[:i]**2
    l_plateau = v*(T[i:-i] - t_inf) + a + acc/2*t_inf**2
    l_dec = acc*T[-i:]*(t - T[-i:]/2) + v*(t - 2*t_inf) + a + acc*(t_inf**2 - t**2/2)       
    return np.r_[l_acc, l_plateau, l_dec]

def __sinusoidal_input(a, b, t, n, v_max, a_max):
    v = 2*(b - a)/t
    if v_max is None or abs(v) <= v_max: # triangle
        acc = np.pi*2*(b - a)/t**2
        if a_max is not None and acc <= a_max:
            print('too much acceleration')
            return
        T = np.linspace(0, t, n)
        return a + v/2*(T - t/(2*np.pi)*np.sin(2*np.pi/t*T))
    
    if v_max*t < abs(b - a):
        print('input not possible')
        return
    # trapèze
    v = np.sign(v)*v_max
    t_inf = t - (b - a)/v
    acc = v/t_inf*np.pi/2
    if a_max is not None and abs(acc) > a_max:
        print('too much acceleration')
        return
    T = np.linspace(0, t, n)
    i = int(t_inf/t*n)
    l_acc = a + v/2*(T[:i] - t_inf/np.pi*np.sin(np.pi/t_inf*T[:i]))
    l_plateau = v*(T[i:-i] - t_inf) + a + v/2*t_inf
    l_dec = v/2*(T[-i:] - t + t_inf + t_inf/np.pi*np.sin(np.pi/t_inf*(T[-i:] - t + t_inf))) + v*(t - 2*t_inf) + a + v/2*t_inf
    return np.r_[l_acc, l_plateau, l_dec]


@physics_output(ANGLE)
@physics_input_function(ANGLE, ANGLE, TIME, '', ANGULAR_VELOCITY)
def direct_angular_input(a, b, t, n=101, v_max=None):
    return __direct_input(a, b, t, n, v_max)

@physics_output(ANGLE)
@physics_input_function(ANGLE, ANGLE, TIME, '', ANGULAR_VELOCITY, ANGULAR_ACCELERATION)
def trapezoidal_angular_input(a, b, t, n=101, v_max=None, a_max=None):
    return __trapezoidal_input(a, b, t, n, v_max, a_max)

@physics_output(ANGLE)
@physics_input_function(ANGLE, ANGLE, TIME, '', ANGULAR_VELOCITY, ANGULAR_ACCELERATION)
def sinusoidal_angular_input(a, b, t, n=101, v_max=None, a_max=None):
    return __sinusoidal_input(a, b, t, n, v_max, a_max)


@physics_output(LENGTH)
@physics_input_function(LENGTH, LENGTH, TIME, '', SPEED)
def direct_linear_input(a, b, t, n=101, v_max=None):
    return __direct_input(a, b, t, n, v_max)

@physics_output(LENGTH)
@physics_input_function(LENGTH, LENGTH, TIME, '', SPEED, ACCELERATION)
def trapezoidal_angular_input(a, b, t, n=101, v_max=None, a_max=None):
    return __trapezoidal_input(a, b, t, n, v_max, a_max)

@physics_output(LENGTH)
@physics_input_function(LENGTH, LENGTH, TIME, '', SPEED, ACCELERATION)
def sinusoidal_angular_input(a, b, t, n=101, v_max=None, a_max=None):
    return __sinusoidal_input(a, b, t, n, v_max, a_max)

# ---------------------------------------------------- Geometry -------------------------------------------------------------

def distance(p1, p2):
    return np.linalg.norm(p2 - p1, axis=0)

def norm(v):
    return np.linalg.norm(v, axis=0)

# ---------------------------------------------------- Derivate -------------------------------------------------------------

@physics_output(SPEED)
@physics_input_function(LENGTH, TIME)
def get_velocity_vector(p, t):
    return np.diff(p, axis=1, append=np.nan)*p.shape[1]/t


@physics_output(ACCELERATION)
@physics_input_function(LENGTH, TIME)
def get_acceleration_vector(p, t):
    return np.diff(p, 2, axis=1, append=np.nan, prepend=np.nan)/(t/p.shape[1])**2


@physics_output(SPEED)
@physics_input_function(LENGTH, TIME)
def get_linear_speed(x, t):
    return np.diff(x, append=np.nan)*len(x)/t


@physics_output(ACCELERATION)
@physics_input_function(LENGTH, TIME)
def get_linear_acceleration(x, t):
    return np.diff(x, 2, append=np.nan, prepend=np.nan)/(t/len(x))**2


@physics_output(ANGULAR_VELOCITY)
@physics_input_function(ANGLE, TIME)
def get_angular_velocity(a, t):
    return np.diff(a, append=np.nan)*len(a)/t


@physics_output(ANGULAR_ACCELERATION)
@physics_input_function(ANGLE, TIME)
def get_angular_acceleration(a, t):
    return np.diff(a, 2, append=np.nan, prepend=np.nan)/(t/len(a))**2


# ---------------------------------------------------- Mass -------------------------------------------------------------




# ---------------------------------------------------- Inertia -------------------------------------------------------------

@physics_output(INERTIA)
@physics_input_function(LENGTH, LENGTH, LENGTH, MASS)
def parallelepiped_inertia(L, l, e, m):
    return m/12*(L**2 + l**2 + e**2)


@physics_output(INERTIA)
@physics_input_function(LENGTH, LENGTH, MASS)
def bar_inertia(L, r, m):
    return m/4*(r**2 + L**2/3)


@physics_output(INERTIA)
@physics_input_function(LENGTH, MASS)
def cylinder_inertia(r, m):
    return m*r**2/2


@physics_output(INERTIA)
@physics_input_function(LENGTH, MASS)
def ball_inertia(r, m):
    return 2/5*m*r**2


