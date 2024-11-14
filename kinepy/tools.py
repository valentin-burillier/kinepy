import numpy as np
import matplotlib.pyplot as plt

from kinepy.units import *
from kinepy.interface.decorators import physics_input_function, physics_output, physics_input_function_variable

# ---------------------------------------------------- Inputs ----------------------------------------------------------

def __sgn(ordre):
    suite = np.array([1])
    for i in range(ordre - 1):
        suite = np.r_[suite, -suite]
    return suite

def __suite_t(ordre, t):
    suite = np.array([0])
    for i in range(ordre - 1):
        suite += 1
        if t[ordre - 2 - i] == 0:
            suite = np.r_[suite, suite]
        else:
            suite = np.r_[suite, 0, suite]
    return suite

__calcul = lambda ordre, p, x : np.sum(p*x**np.arange(0, ordre + 1))

def __deriv(ordre, p):
    p[:ordre] = p[1:]*np.arange(1, ordre + 1)
    p[ordre] = 0
    return p


def __creation_tab(ordre, a, const, suite, t):
    tab = np.zeros((len(suite), ordre + 1))
    tab[suite==ordre-1, ordre] = const[-1]/np.math.factorial(ordre)*__sgn(ordre)
    tab[0, 0] = a
    for i in range(len(suite) - 1):
        dt = t[suite[i]]
        p = tab[i].copy()
        tab[i+1, 0] = __calcul(ordre, p, dt)
        for d in range(1, ordre):
            p = __deriv(ordre, p)
            tab[i+1, d] = __calcul(ordre, p, dt)/np.math.factorial(d)
    return tab
    
def __trouver_const(a, b, const, ordre, iteration):
    c = [b - a] + const
    k = [c[i]/c[i + 1] for i in range(ordre)]
    
    hist = []
    for _ in range(iteration):
        for i in range(ordre - 1):
            sum_k = sum(k[i+1:])
            if i in hist or k[i] < sum_k:
                if i not in hist:
                    hist.append(i)
                if i + 2 > ordre:
                    c[i+1] = np.sqrt(c[i]*c[i+2])
                else:
                    b = sum_k - k[i+1]
                    det = b**2 + 4*c[i]/c[i+2]
                    c[i+1] = (-b + np.sqrt(det))/2*c[i+2]
                k[i] = c[i]/c[i+1]
                k[i+1] = c[i+1]/c[i+2]
                
    return c, k

def polynomial_input(a, b, const, n=101, iteration=100):
    ordre = len(const)

    c, k = __trouver_const(a, b, const, ordre, iteration)

    t = []
    sum_k = sum(k)
    for i in range(ordre):
        sum_k -= k[i]
        t_i = k[i] - sum_k
        t.append(t_i if t_i > 10e-15 else 0)
    
    suite = __suite_t(ordre, t)
    tab = __creation_tab(ordre, a, const, suite, t)
    X = np.linspace(0, np.sum(t*2**np.arange(0, ordre)), n)
    Y = []
    i = 0
    x_inf = 0
    x_sup = t[suite[0]]
    for x in X:
        while x > x_sup and i + 1 < len(suite):
            i += 1
            x_inf = x_sup
            x_sup += t[suite[i]]
        Y.append(__calcul(ordre, tab[i], x - x_inf))
    return X, np.array(Y)


@physics_input_function_variable(VARIABLE_UNIT, VARIABLE_UNIT, VARIABLE_DERIVATIVE, '', '')
def direct_input(a, b, v_max, n=101, iteration=100):
    polynomial_input(a, b, [v_max], n=n, iteration=iteration)

@physics_input_function_variable(VARIABLE_UNIT, VARIABLE_UNIT, VARIABLE_DERIVATIVE, VARIABLE_SECOND_DERIVATIVE, '', '')
def trapezoidal_input(a, b, v_max, a_max, n=101, iteration=100):
    polynomial_input(a, b, [v_max, a_max], n=n, iteration=iteration)


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
        return np.diff(obj * SYSTEM[phy], append=np.nan) * (len(obj) - 1) / SYSTEM[TIME] / SYSTEM[DERIVATIVES[phy]] / t
    return np.diff(obj * SYSTEM[phy], axis=1, append=np.nan) * (obj.shape[1] - 1) / SYSTEM[TIME] / SYSTEM[DERIVATIVES[phy]] / t


def second_derivative(obj, t, *, phy=LENGTH):
    if phy not in DERIVATIVES:
        raise ValueError(f"This unit {phy} has no derivative ready to use")
    if DERIVATIVES[phy] not in DERIVATIVES:
        raise ValueError(f"This unit {phy} has no second derivative ready to use")
    second = DERIVATIVES[DERIVATIVES[phy]]
    if len(obj.shape) == 1:
        return np.diff(obj * SYSTEM[phy], append=np.nan) / SYSTEM[second] * ((len(obj) - 1) / SYSTEM[TIME] / t) ** 2
    return np.diff(obj * SYSTEM[phy], 2, axis=1, prepend=np.nan, append=np.nan) / SYSTEM[second] * ((obj.shape[1]-1) / SYSTEM[TIME] / t ** 2)


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
