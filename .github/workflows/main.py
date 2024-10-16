import math
import time
import sympy

# funciones de membresía control difuso
# entradas
# funciones eD -----------------------------------------------

def integrator(Xp):
    T = 0.193
    # Definir el intervalo de integración y la condición inicial
    tspan = [0, T]  # intervalo de tiempo (0 a 2.54 segundos)
    X0 = 0  # valor inicial de X en t = 0

    # Definir la función de integración
    t_final = tspan[-1]  # obtener el valor final de tiempo (2.54 en este caso)
    X_final = X0 + Xp * t_final
    return X_final

# funciones eDelta -----------------------------------------------
def control_errdN(error_d): # saturacion izquierda
    a_Ned = -1
    b_Ned = 0
    if error_d <= a_Ned:
        return 1
    elif error_d > a_Ned and error_d <= b_Ned:
        return (error_d - b_Ned) / (a_Ned - b_Ned)
    else:
        return 0

def control_errdZ(error_d):
    a_Zed = -1.5
    b_Zed = 0
    d_Zed = 1.5
    if error_d >= a_Zed and error_d <= b_Zed:
        return (error_d - a_Zed) / (b_Zed - a_Zed)
    elif error_d >= b_Zed and error_d <= d_Zed:
        return (error_d - d_Zed) / (b_Zed - d_Zed)
    else:
        return 0

def control_errdP(error_d): # saturacion derecha
    a_Ped = 0
    b_Ped = 1
    if error_d <= a_Ped:
        return 0
    elif error_d >= a_Ped and error_d <= b_Ped:
        return (error_d - a_Ped) / (b_Ped - a_Ped)
    elif error_d >= b_Ped:
        return 1

# funciones eX -----------------------------------------------
def control_errxN(error_x):  # funcion saturación izquierda de N de eX en Fctrl
    a_Nex = -0.2
    b_Nex = 0
    if error_x <= a_Nex:
        return 1
    elif error_x > a_Nex and error_x <= b_Nex:
        return (error_x - b_Nex) / (a_Nex - b_Nex)
    else:
        return 0

def control_errxZ(error_x):  # funcion triangular de Z de eX en Fctrl
    a_Zex = -0.15134
    b_Zex = 0
    d_Zex = 0.15134
    if error_x >= a_Zex and error_x <= b_Zex:
        return (error_x - a_Zex) / (b_Zex - a_Zex)
    elif error_x >= b_Zex and error_x <= d_Zex:
        return (error_x - d_Zex) / (b_Zex - d_Zex)
    else:
        return 0

def control_errxP(error_x):  # funcion saturacion derecha de P de eX en Fctrl
    a_Pex = 0
    b_Pex = 0.2
    if error_x <= a_Pex:
        return 0
    elif error_x >= a_Pex and error_x <= b_Pex:
        return (error_x - a_Pex) / (b_Pex - a_Pex)
    elif error_x >= b_Pex:
        return 1

# funciones eY -----------------------------------------------
def control_erryN(error_y):  # funcion saturación izquierda de N de eY en Fctrl
    a_Ney = -0.2
    b_Ney = 0
    if error_y <= a_Ney:
        return 1
    elif error_y > a_Ney and error_y <= b_Ney:
        return (error_y - b_Ney) / (a_Ney - b_Ney)
    else:
        return 0

def control_erryZ(error_y):  # funcion triangular de Z de eY en Fctrl
    a_Zey = -0.15
    b_Zey = 0
    d_Zey = 0.15
    if error_y >= a_Zey and error_y <= b_Zey:
        return (error_y - a_Zey) / (b_Zey - a_Zey)
    elif error_y >= b_Zey and error_y <= d_Zey:
        return (error_y - d_Zey) / (b_Zey - d_Zey)
    else:
        return 0

def control_erryP(error_y):  # funcion saturacion derecha de P de eY en Fctrl
    a_Pey = 0
    b_Pey = 0.2
    if error_y <= a_Pey:
        return 0
    elif error_y >= a_Pey and error_y <= b_Pey:
        return (error_y - a_Pey) / (b_Pey - a_Pey)
    elif error_y >= b_Pey:
        return 1

# funciones V -----------------------------------------------
def control_errvN(error_v):  # funcion saturación izquierda de N de V en Fctrl
    a_Nv = -0.02
    b_Nv = 0
    if error_v <= a_Nv:
        return 1
    elif error_v > a_Nv and error_v <= b_Nv:
        return (error_v - b_Nv) / (a_Nv - b_Nv)
    else:
        return 0

def control_errvZ(error_v):  # funcion triangular de Z de v en Fctrl
    a_Zv = -0.015
    b_Zv = 0
    d_Zv = 0.015
    if error_v >= a_Zv and error_v <= b_Zv:
        return (error_v - a_Zv) / (b_Zv - a_Zv)
    elif error_v >= b_Zv and error_v <= d_Zv:
        return (error_v - d_Zv) / (b_Zv - d_Zv)
    else:
        return 0

def control_errvP(error_v):  # funcion saturacion derecha de P de v en Fctrl
    a_Pv = 0
    b_Pv = 0.02
    if error_v <= a_Pv:
        return 0
    elif error_v >= a_Pv and error_v <= b_Pv:
        return (error_v - a_Pv) / (b_Pv - a_Pv)
    elif error_v >= b_Pv:
        return 1

# funciones W -----------------------------------------------
def control_errwN(error_w):  # funcion saturación izquierda de N de W en Fctrl
    a_Nw = -0.35
    b_Nw = 0
    if error_w <= a_Nw:
        return 1
    elif error_w > a_Nw and error_w <= b_Nw:
        return (error_w - b_Nw) / (a_Nw - b_Nw)
    else:
        return 0

def control_errwZ(error_w):  # funcion triangular de Z de W en Fctrl
    a_Zw = -0.3
    b_Zw = 0
    d_Zw = 0.3
    if error_w >= a_Zw and error_w <= b_Zw:
        return (error_w - a_Zw) / (b_Zw - a_Zw)
    elif error_w >= b_Zw and error_w <= d_Zw:
        return (error_w - d_Zw) / (b_Zw - d_Zw)
    else:
        return 0

def control_errwP(error_w):  # funcion saturacion derecha de P de W en Fctrl
    a_Pw = 0
    b_Pw = 0.35
    if error_w <= a_Pw:
        return 0
    elif error_w >= a_Pw and error_w <= b_Pw:
        return (error_w - a_Pw) / (b_Pw - a_Pw)
    elif error_w >= b_Pw:
        return 1

# reglas de control difuso basadas en las funciones de membresía -----------------------------------------------
def reglas_control_w(error_d, error_x, error_y):
    # calcular valores de reglas para W
    pw1 = control_errdP(error_d)  # regla 1
    zw2 = control_errdZ(error_d)  # regla 2
    nw3 = control_errdN(error_d)  # regla 3
    zw4 = min(control_errxZ(error_x), control_erryZ(error_y))  # regla 4
    zw5 = min(control_errxN(error_x), control_erryZ(error_y))  # regla 5
    # defuzzificacion =========================================
    # w - velocidad angular
    b_pw1 = 0.35
    b_zw2 = 0.0001
    b_nw3 = -0.35
    b_zw4 = 0.0001
    b_zw5 = 0.0001
    denom_w = pw1 + zw2 + nw3 + zw4 + zw5
    if denom_w < 0.00000001:
        return 0
    numer_w = (pw1 * b_pw1 + zw2 * b_zw2 + nw3 * b_nw3 + zw4 * b_zw4 + zw5 * b_zw5)
    if numer_w < 0.00000001:
        return 0
    W_f = numer_w / denom_w
    return W_f

def reglas_control_v(error_d, error_x, error_y):
    # calcular valores de reglas para V
    pv2 = control_errdZ(error_d)  # regla 2
    zv4 = min(control_errxZ(error_x), control_erryZ(error_y))  # regla 4
    nv5 = min(control_errxN(error_x), control_erryZ(error_y))  # regla 5
    # defuzzificacion
    # v - velocidad lineal
    b_pv2 = 0.02
    b_zv4 = 0.0001
    b_nv5 = -0.02
    denom_v = pv2 + zv4 + nv5
    if denom_v < 0.00000001:
        return 0
    numer_v = (pv2 * b_pv2 + zv4 * b_zv4 + nv5 * b_nv5)
    if numer_v < 0.00000001:
        return 0
    V_f = numer_v / denom_v
    return V_f

# calculo de delta -----------------------------------------------
def calculo_delta(x, y):
    edelta = math.atan2(y, x)
    return edelta

# modelo de velocidad de ruedas -----------------------------------------------
def mod_velr(V, W):
    fi1 = -V-W
    fi2 = V-W
    return fi1, fi2

# modelo de postura -----------------------------------------------
def mod_post(Vcalc, Wcalc, theta):
    xp = Vcalc*math.cos(theta)
    yp = Vcalc*math.sin(theta)
    thp = Wcalc
    return xp, yp, thp
    
def integrar_todo(xp, yp, thp, fi1, fi2):
    xcalc = integrator(xp)
    ycalc = integrator(yp)
    thcalc = integrator(thp)
    fi1calc = integrator(fi1)
    fi2calc = integrator(fi2)
    return xcalc, ycalc, thcalc, fi1calc, fi2calc

# main entradas y con errores inventados -----------------------------------------------

def main():
    xreal = 0
    yreal = 0
    threal = 0.7921


    x = float(input("Dame el valor de X: "))
    y = float(input("Dame el valor de Y: "))
    print("\n")
    
    eX = x - xreal
    eY = y - yreal
    delta = calculo_delta(eX, eY)
    print("El valor de Delta es: {:.6f}".format(delta))
    print("\n")
    edelta = delta - threal
    print("El error de x es: {:.6f}".format(eX))
    print("El error de y es: {:.6f}".format(eY))
    print("El error de theta es: {:.6f}".format(edelta))
    print("\n")
    Vgain = 3
    Wgain = 7
    
    Vdif = reglas_control_v(edelta, eX, eY)
    Wdif = reglas_control_w(edelta, eX, eY)
    
    Vcalc = Vdif * Vgain
    Wcalc = Wdif * Wgain
    
    print("Velocidad lineal V: {:.6f}".format(Vcalc))
    print("Velocidad angular W: {:.6f}".format(Wcalc))
    print("\n")
    radio = 0.0162
    long = 0.15
    fact1 = 1/radio
    fact2 = long/radio

    inp1_modvelr = fact1*Vcalc
    inp2_modvelr = fact2*Wcalc

    [fi1, fi2] = mod_velr(inp1_modvelr, inp2_modvelr)
    [xp, yp, thp] = mod_post(Vcalc, Wcalc, threal)

    #print("Velocidad rueda fi1: {:.6f}".format(fi1))
    #print("Velocidad rueda fi2: {:.6f}".format(fi2))
    print("Velocidad en X: {:.6f}".format(xp))
    print("Velocidad en Y: {:.6f}".format(yp))
    print("Velocidad ang, theta: {:.6f}".format(thp))
    print("\n")
    [xcalc, ycalc, thcalc, fi1calc, fi2calc] = integrar_todo(xp, yp, thp, fi1, fi2)
    
    print("Posición rueda fi1: {:.6f}".format(fi1calc))
    print("Posición rueda fi2: {:.6f}".format(fi2calc))
    print("Posición en X: {:.6f}".format(xcalc))
    print("Posición en Y: {:.6f}".format(ycalc))
    print("Posición ang, theta: {:.6f}".format(thcalc))
    print("\n")
if __name__ == "__main__":
    main()
