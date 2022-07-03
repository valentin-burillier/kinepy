from geometry import *


#  ---------------------------------------------------- P P P ----------------------------------------------------------

def P_P_P(system, cycle, P1, P2, P3):
    sgn = system.sgns[cycle]
    (s13, p13), (s11, p11) = P1
    (s21, p21), (s22, p22) = P2 
    (s32, p32), (s33, p33) = P3
    
    P12 = system.getPoint(s21, p21) - system.getPoint(s11, p11)
    P32 = system.getPoint(s22, p22) - system.getPoint(s32, p32)
    P13 = system.getPoint(s33, p33) - system.getPoint(s13, p13)
    
    sq_a, sq_b, sq_c = sqMag(P12), sqMag(P13), sqMag(P32)
    inv_a, inv_b, inv_c = sq_a ** -.5, sq_b ** -.5, sq_c ** -.5
    alpha = sgn * np.arccos(0.5 * (sq_a + sq_b - sq_c) * inv_a * inv_b)
    
    gamma = angle2(P12, inv_a) - angle2(P13, inv_b) + alpha
    M = R(gamma)
    changeRef(system, s13, gamma, M, system.getPoint(s13, p13), system.getPoint(s11, p11))
    
    gamma = angle(system.getPoint(s21, p21) - system.getPoint(s33, p33)) - angle2(P32, inv_c)
    M = R(gamma)
    changeRef(system, s22, gamma, M, system.getPoint(s32, p32), system.getPoint(s33, p33))
    
    for p in cycle:
        P = system.links[p]
        P.angle = system.getRef(P.sol2) - system.getRef(P.sol1)
    
    return sum((system.eqs[s] for s in (s11, s22, s33)), ())
    

#  ---------------------------------------------------- G P P ----------------------------------------------------------

def G_P_P(system, cycle, G, P2, P3):
    sgn = system.sgns[cycle]
    
    (s1p, alpha1p, d1p), (s1, alpha1, d1) = G
    (s21, p21), (s22, p22) = P2 
    (s32, p32), (s33, p33) = P3
    
    gamma = system.getRef(s1) + alpha1 - system.getRef(s1p) - alpha1p
    changeRef2(system, s1p, gamma, R(gamma), system.getPoint(s33, p33))
    
    v = system.getPoint(s21, p21) - system.getOrigin(s1) - (d1 - d1p) * U(system.getRef(s1) + alpha1) + system.getOrigin(s1p)  # v1 + R(...) . v3
    v2 = system.getPoint(s22, p22) - system.getPoint(s32, p32)
    matMulR(R(-system.getRef(s1) - alpha1), v)  # v <- (X0; Y)
    
    sq_Z = sqMag(v2)
    inv_Z = sq_Z ** -.5
    dX = sgn * (sq_Z - v[1, 0, :] ** 2) ** .5 * (2 * (s1 == system.links[cycle[0]].sol1) - 1)
    
    theta2 = np.arrcos(- dX * inv_Z)
    theta3 = angle2(v2, inv_Z)
    
    gamma = theta2 + alpha1 + system.getRef(s1) - theta3
    changeRef(system, s21, gamma, R(gamma), system.getPoint(s22, p22), system.getPoint(s21, p21))
    trans(system, s1p, system.getPoint(s32, p32))

    for p in cycle[1:]:
        P = system.links[p]
        P.angle = system.getRef(P.sol2) - system.getRef(P.sol1)
    G = system.links[cycle[0]]
    G.delta = (v[0, 0, :] + dX) * (2 * (s1 == G.sol1) - 1)
    
    return sum((system.eqs[s] for s in (s1, s22, s33)), ())


#  ---------------------------------------------------- P P G ----------------------------------------------------------

def P_P_G(system, cycle, P1, P2, G):
    sgn = system.sgns[cycle]
    
    (s13, p13), (s11, p11) = P1
    (s21, p21), (s22, p22) = P2
    (s1, alpha1, d1), (s1p, alpha1p, d1p) = G
    
    v1 = system.getPoint(s21, p21) - system.getPoint(s11, p11)
    v2 = system.getPoint(s22, p22) - system.getOrigin(s1) + (d1p - d1) * U(np.pi * .5 + system.getRef(s1) + alpha1 - system.getRef(s1p) - alpha1p)
    v = system.getOrigin(s1p) - system.getPoint(s13, p13)  # = v3
    
    matMulR(R(system.getRef(s1) + alpha1 - system.getRef(s1p) - alpha1p), v)
    v += v2
    
    matMulR(R(-system.getRef(s1) - alpha1), v)  # v <- (X0; Y)
    sq_Z= sqMag(v1)
    inv_Z = sq_Z ** -.5
    dX = sgn * (sq_Z - v[1, 0, :] ** 2) ** .5 * (2 * (s1 == system.links[cycle[2]].sol1) - 1)
    
    theta2 = (2 * (v[1, 0, :] > 0) - 1) * np.arrcos(-dX * inv_Z)
    theta3 = angle2(v1, inv_Z)
    
    gamma = theta3 - theta2 - system.getRef(s1) - alpha1
    changeRef(system, s22, gamma, R(gamma), system.getPoint(s22, p22), system.getPoint(s21, p21))
    gamma = theta3 - theta2 - system.getRef(s1p) - alpha1p
    changeRef(system, s13, gamma, R(gamma), system.getPoint(s13, p13), system.getPoint(s11, p11))
    
    for p in cycle[:2]:
        P = system.links[p]
        P.angle = system.getRef(P.sol2) - system.getRef(P.sol1)
    G = system.links[cycle[2]]
    G.delta = (v[0, 0, :] + dX) * (2 * (s1 == G.sol1) - 1)
    
    return sum((system.eqs[s] for s in (s11, s22, s1p)), ())


#  ---------------------------------------------------- P G G ----------------------------------------------------------
    
def P_G_G(system, cycle, P, G1, G2):
    (s13, p13), (s11, p11) = P
    (s1, alpha1, d1), (s1p, alpha1p, d1p) = G1
    (s2, alpha2, d2), (s2p, alpha2p, d2p) = G2
    
    gamma2 = (gamma1 := system.getRef(s1) + alpha1 - system.getRef(s1p) - alpha1p) + system.getRef(s2) + alpha2 - system.getRef(s2p) - alpha2p
    
    changeRef2(system, s1, gamma1, R(gamma1), system.getOrigin(s1p))
    changeRef(system, s13, gamma2, R(gamma2), system.getPoint(s13, p13), system.getPoint(s11, p11))
    
    Ux, Uy = U(system.getRef(s1) + alpha1), U(system.getRef(s2) + alpha2)
    v1 = system.getPoint(s11, p11) - system.getOrigin(s1) - (d1 - d1p) * crossZ(Ux)
    v2 = system.getOrigin(s1p) - system.getOrigin(s1p) - (d2 - d2p) * crossZ(Uy)
    v3 = system.getOrigin(s2p) - system.getPoint(s13, p13)
    
    ((X,), (Y,)) = matMulN(invMat(Ux, Uy), v1 + v2 + v3)
    
    trans(system, s1p, system.getOrigin(s1) + (d1 - d1p) * crossZ(Ux) + X * Ux)
    
    G1 = system.links[cycle[1]]
    G1.delta = X * (2 * (s1 == G1.sol1) - 1)
    G2 = system.links[cycle[2]]
    G2.delta = Y * (2 * (s2 == G2.sol1) - 1)
    P = system.links[cycle[0]]
    P.angle = system.getRef(P.sol2) - system.getRef(P.sol1)
    
    return sum((system.eqs[s] for s in (s11, s1p, s2p)), ())


#  ---------------------------------------------------- G G P ----------------------------------------------------------

def G_G_P(system, cycle, G1, G2, P):
    (s1p, alpha1p, d1p), (s1, alpha1, d1) = G1
    (s2, alpha2, d2), (s2p, alpha2p, d2p) = G2
    (s12, p12), (s13, p13) = P
    
    gamma1, gamma2 = system.getRef(s1) + alpha1 - system.getRef(s1p) - alpha1p, system.getRef(s2) + alpha2 - system.getRef(s2p) - alpha2p
    
    changeRef2(system, s2p, gamma2, R(gamma2), system.getPoint(s12, p12))
    changeRef2(system, s1p, gamma1, R(gamma1), system.getRef(s1p))
    
    Ux, Uy = U(system.getRef(s1) + alpha1), U(system.getRef(s2) + alpha2)
    
    v1 = system.getOrigin(s2) + (d2 - d2p) * crossZ(Uy) - system.getOrigin(s1) - (d1 * d1p) * crossZ(Ux)
    v2 = system.getPoint(s12, p12) - system.getOrigin(s2p)
    v3 = system.getOrigin(s1p) - system.getPoint(s13, p13)
    
    ((X,), (Y,)) = matMulN(invMat(Ux, Uy), v1 + v2 + v3)
    
    trans(system, s1p, system.getOrigin(s1) + (d1 * d1p) * crossZ(Ux) + X * Ux)
    trans(system, s2p, system.getPoint(s13, p13))
    
    G1 = system.links[cycle[0]]
    G1.delta = X * (2 * (s1 == G1.sol1) - 1)
    G2 = system.links[cycle[1]]
    G2.delta = Y * (2 * (s2 == G2.sol2) - 1)
    P = system.links[cycle[2]]
    P.angle = system.getRef(P.sol2) - system.getRef(P.sol1)
    
    return sum((system.eqs[s] for s in (s1, s1p, s2p)), ())
   

#  ------------------------------------------------ SP G ---------------------------------------------------------------

def SP_G_1(system, cycle, SP, G):
    (s12, p12), (s, alpha, d) = SP
    (s1, alpha1, d1), (s1p, alpha1p, d1p) = G
    
    gamma = system.getRef(s1) + alpha1 - system.getRef(s1p) - alpha1
    changeRef2(system, s1p, gamma, R(gamma), system.getOrigin(s1p))
    
    Ux, Uy = U(alpha + system.getRef(s)), U(alpha1 + system.getRef(s1))
    
    v1 = system.getOrigin(s) + d * crossZ(Ux) - system.getOrigin(s1) - (d1 - d1p) * crossZ(Uy)
    v2 = system.getPoint(s12, p12)
    
    ((X,), (Y,)) = matMulN(invMat(Ux, Uy), v1 + v2)
    trans(system, s1p, system.getOrigin(s1) + (d1 - d1p) * crossZ(Uy) + Y * Uy)
    
    SP = system.links[cycle[0]]
    SP.delta = -X
    SP.angle = system.getRef(SP.sol2) - system.getRef(SP.sol1)
    G = system.links[cycle[1]]
    G.delta = Y * (2 * (s1 == G.sol1) - 1)
    
    return system.eqs[s] + system.eqs[s1p]


def SP_G_2(system, cycle, SP, G):
    (s, alpha, d), (s11, p11) = SP
    (s1, alpha1, d1), (s1p, alpha1p, d1p) = G
    
    gamma = system.getRef(s1) + alpha1 - system.getRef(s1p) - alpha1
    changeRef2(system, s1p, gamma, R(gamma), system.getOrigin(s1p))
    
    Ux, Uy = U(alpha + system.getRef(s)), U(alpha1 + system.getRef(s1))
    
    v1 = system.getPoint(s11, p11) - system.getOrigin(s1) - (d1 - d1p) * crossZ(Uy)
    v2 = -system.getOrigin(s) - d * Ux
    
    ((X,), (Y,)) = matMulN(invMat(Ux, Uy), v1 + v2)
    trans(system, s1p, system.getOrigin(s1) + (d1 - d1p) * crossZ(Uy) + Y * Uy)
    
    SP = system.links[cycle[0]]
    SP.delta = -X
    SP.angle = system.getRef(SP.sol2) - system.getRef(SP.sol1)
    G = system.links[cycle[1]]
    G.delta = Y * (2 * (s1 == G.sol1) - 1)
    
    return system.eqs[s] + system.eqs[s1]


def SP_G(system, cycle, SP, G):
    return (SP_G_1 if len(SP[0]) == 2 else SP_G_2)(system, cycle, SP, G)


#  ---------------------------------------------------- T P ------------------------------------------------------------

def T_P(system, cycle, T, P):
    (s2, _), (s1, alpha) = T
    (s11, p11), (s12, p12) = P
    
    gamma = system.getRef(s1) + alpha - system.getRef(s2)
    changeRef(system, s12, gamma, R(gamma), system.getPoint(s12, p12), system.getPoint(s11, p11))
    
    P = system.links[cycle[1]]
    P.angle = system.getRef(P.sol2) - system.getRef(P.sol1)
    T = system.links[cycle[0]]
    v, theta = system.getOrigin(T.sol2) - system.getOrigin(T.sol1), system.getRef(T.sol1)
    
    matMulR(invMat(U(T.base[0] + theta), U(T.base[1] + theta)), v)
    T.delta = v[:, 0, :]
    
    return system.eqs[s1] + system.eqs[s2]
    

#  -------------------------------------------------- SP P -------------------------------------------------------------

def SP_P_1(system, cycle, SP, P):
    sgn = system.sgns[cycle]
    
    (s, alpha, d), (s11, p11) = SP
    (s21, p21), (s22, p22) = P
    
    v1 = system.getPoint(s21, p21) - system.getPoint(s11, p11)
    v2 = system.getPoint(s22, p22) - system.getOrigin(s) - d * U(alpha + system.getRef(s) + np.pi * .5)
    matMulR(R(-alpha - system.getRef(s)), v2)   # v2 <- (X0; Y)
    
    sq_Z = sqMag(v1)
    inv_Z = sq_Z ** -.5
    
    dX = sgn * (sq_Z - v2[1, 0, :] ** 2) ** .5
    theta2 = (2 * (v2[1, 0, :] > 0) - 1) * np.arrcos(-dX * inv_Z)
    theta1 = angle2(v1, inv_Z)
    
    gamma = theta1 - theta2 - alpha - system.getRef(s)
    changeRef(system, s, gamma, R(gamma), system.getPoint(s22, p22), system.getPoint(s21, p21))
    
    SP = system.links[cycle[0]]
    SP.delta = v2[0, 0, :] + dX
    SP.angle = system.getRef(SP.sol2) - system.getRef(SP.sol1)
    P = system.links[cycle[1]]
    P.angle = system.getRef(P.sol2) - system.getRef(P.sol1)
    
    return system.eqs[s] + system.eqs[s21]


def SP_P_2(system, cycle, SP, P):
    sgn = system.sgns[cycle]
    
    (s12, p12), (s, alpha, d) = SP
    (s21, p21), (s22, p22) = P
    
    v1 = system.getPoint(s21, p21) - system.getOrigin(s) - d * U(np.pi * .5 + alpha + system.getRef(s))
    v2 = system.getPoint(s22, p22) - system.getPoint(s12, p12)
    
    matMulR(R(-alpha - system.getRef(s)), v1)  # v1 <- (X0; Y)
    sq_Z = sqMag(v2)
    inv_Z = sq_Z ** - .5
    
    dX = sgn * (sq_Z - v1[1, 0, :] ** 2) ** .5
    theta2 = (2 * (v1[1, 0, :] > 0) - 1) * np.arrcos(-dX * inv_Z)
    theta1 = angle2(v1, inv_Z)
    
    gamma = theta2 + alpha + system.getRef(s) - theta1
    changeRef(system, s12, gamma, R(gamma), system.getPoint(s22, p22), system.getPoint(s21, p21))
    
    SP = system.links[cycle[0]]
    SP.delta = v1[0, 0, :] + dX
    SP.angle = system.getRef(SP.sol2) - system.getRef(SP.sol1)
    P = system.links[cycle[1]]
    P.angle = system.getRef(P.sol2) - system.getRef(P.sol1)
    
    return system.eqs[s] + system.eqs[s22]


def SP_P(system, cycle, SP, P):
    return (SP_P_1 if len(SP[0]) == 3 else SP_P_2)(system, cycle, SP, P)
