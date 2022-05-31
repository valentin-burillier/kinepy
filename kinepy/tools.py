import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import cm
import matplotlib.colors as mcol


global C
# C = cm.get_cmap('coolwarm') # plasma, coolwarm, bwr, rainbow, OrRd
C = mcol.LinearSegmentedColormap.from_list("", ["cyan", "r"])


def animate(list_paths, list_vectors=None, anim_time=4, repeat=True, scale=1, vector_scale=0.1, magnitude_vector='proportionnal'):
    # magnitude vector = {proportionnal, log, unitary}
    M = np.array(list_paths) # prendre en compte chemins de diff taille
    if len(M.shape) == 3:
        M = M.reshape((1,) + M.shape)
        
    M = np.transpose(M, (0, 3, 2, 1))
    nan = np.any(np.isnan(M), axis=(0, 2, 3))
    M[:, nan] = 0

    x_min = M[:, :, 0].min()
    x_max = M[:, :, 0].max()
    y_min = M[: ,:, 1].min()
    y_max = M[:, :, 1].max()
    dx = (x_max - x_min)*0.05
    dy = (y_max - y_min)*0.05

    fig = plt.figure(figsize=((x_max - x_min)/(y_max - y_min)*scale*7, scale*7))
    ax = plt.axes(xlim=(x_min - dx, x_max + dx), ylim=(y_min - dy, y_max + dy))
    lines = [ax.plot([], [], lw=2, color='tab:blue')[0] for _ in range(len(M))]
    
    if list_vectors is not None:
        PAs, vecs = [], []
        for PA, vec in list_vectors:
            PAs.append(PA)
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
        lines.append(ax.quiver([1000]*n,[1000]*n, [0]*n, [0]*n, scale=1/vector_scale, scale_units='xy', cmap='plasma'))
        def _animate_(i):
            for lnum, line in enumerate(lines[:-1]):
                line.set_data(M[lnum, i, 0], M[lnum, i, 1])
            lines[-1].set_UVC(vecs[i, 0], vecs[i, 1])
            lines[-1].set_offsets(PAs[i].T)
            lines[-1].set_facecolor(color_array[i])
            return lines
    else:
        def _animate_(i):
            for lnum, line in enumerate(lines):
                line.set_data(M[lnum, i, 0], M[lnum, i, 1])
            return lines
    
    n = M.shape[1]
    anim = animation.FuncAnimation(fig, _animate_, frames=n, interval=anim_time*1000/n, blit=True, repeat=repeat)    
    
    return anim

def make_continuous(L):
    l0 = 0
    for i, l in enumerate(L):
        if not np.isnan(L[i]):
            k = (l - l0 + np.pi)//(2*np.pi)
            L[i] -= k*2*np.pi
            l0 = L[i]
    return L

distance = lambda point1, point2 : np.sqrt(np.sum((point2 - point1)**2, axis=0))

def get_speed(L, t):
    if len(L.shape) == 1:    
        return (L[1:] - L[:-1])/(t/(len(L) - 1))
    return (L[:, 1:] - L[:, :-1])/(t/(L.shape[1] - 1))

def get_speed_precis(L, t):
    if len(L.shape) == 1:    
        return (L[2:] - L[:-2])/(2*t/(len(L) - 1))
    return (L[:, 2:] - L[:, :-2])/(2*t/(L.shape[1] - 1))

def get_acceleration(L, t):
    if len(L.shape) == 1:    
        return (L[2:] - 2*L[1:-1] + L[:-2])/(t/(len(L) - 1))**2
    return (L[:, 2:] - 2*L[:, 1:-1] + L[:, :-2])/(t/(L.shape[1] - 1))**2

# prendre en compte chemins de diff taille
# prolongement de vitesse et d'acc pour résoudre le pb de shape, fct np.diff
