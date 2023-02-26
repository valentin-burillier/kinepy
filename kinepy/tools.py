import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import cm
import matplotlib.colors as mcol

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

def direct_input(a, b, t, n=101, v_max=None):
    v = (b - a)/t
    if v_max is not None and abs(v) > v_max: # triangle
        print('speed too high')
        return
    return np.linspace(a, b, n)


def trapezoidal_input(a, b, t, n=101, v_max=None, a_max=None):
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
    print(acc)
    if a_max is not None and abs(acc) > a_max:
        print('too much acceleration')
        return
    T = np.linspace(0, t, n)
    i = int(t_inf/t*n)
    l_acc = a + acc/2*T[:i]**2
    l_plateau = v*(T[i:-i] - t_inf) + a + acc/2*t_inf**2
    l_dec = acc*T[-i:]*(t - T[-i:]/2) + v*(t - 2*t_inf) + a + acc*(t_inf**2 - t**2/2)       
    return np.r_[l_acc, l_plateau, l_dec]

def sinusoidal_input(a, b, t, n=101, v_max=None, a_max=None):
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

j_parallelepiped = lambda L, l, e, m : m/12*(L**2 + l**2 + e**2)

j_bar = lambda L, r, m : m/4*(r**2 + L**2/3)

j_cylinder = lambda r, e, m : m*r**2/2

j_ball = lambda r, m : 2/5*m*r**2

def make_continuous(L):
    l0 = 0
    for i, l in enumerate(L):
        if not np.isnan(L[i]):
            k = (l - l0 + np.pi)//(2*np.pi)
            L[i] -= k*2*np.pi
            l0 = L[i]
    return L

def distance(p1, p2):
    return np.linalg.norm(p2 - p1, axis=0)

def norm(v):
    return np.linalg.norm(v, axis=0)

def to_cartesian(r, a):
    return (r*np.cos(a), r*np.sin(a))

def get_speed(p, t):
    if len(p.shape) == 1:    
        return np.diff(p)/t*(len(p) - 1)
    return np.diff(p, axis=1)/t*(p.shape[1] - 1)

def get_speed_accurate(p, t):
    if len(p.shape) == 1:    
        return (p[2:] - p[:-2])/(2*t/(len(p) - 1))
    return (p[:, 2:] - p[:, :-2])/(2*t/(p.shape[1] - 1))

def get_acceleration(p, t):
    if len(p.shape) == 1:    
        return np.diff(p, 2)/(t/(len(p) - 1))**2
    return np.diff(p, 2, axis=1)/(t/(p.shape[1] - 1))**2
