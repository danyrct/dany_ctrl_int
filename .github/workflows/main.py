import math
import time
# Función atan2 para calcular el error angular (Delta)
def calcular_delta(y, x):
    return math.atan2(y, x)

# funciones de membresía control difuso
# entradas
#funciones eD -----------------------------------------------
def control_errdN(error_d):
    a_Ned = -1
    b_Ned = 0
    if error_d <= a_Ned:
        return 1
    elif error_d > a_Ned and error_d <= b_Ned:
        return (b_Ned - error_d) / (b_Ned - a_Ned)
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
def control_errdP(error_d):
    a_Ped = 0
    b_Ped = 1
    if error_d <= a_Ped:
        return 0
    elif error_d >= a_Ped and error_d <= b_Ped:
        return (error_d - a_Ped) / (b_Ped - a_Ped)
    elif error_d >= b_Ped:
        return 1
# funciones eX -----------------------------------------------
def control_errxN(error_x):  # funcion saturación de N de eX en Fctrl
    a_Nex = -0.2
    b_Nex = 0
    if error_x <= a_Nex:
        return 1
    elif error_x > a_Nex and error_x <= b_Nex:
        return (b_Nex - error_x) / (b_Nex - a_Nex)
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
def control_errxP(error_x):  # funcion saturacion de P de eX en Fctrl
    a_Pex = 0
    b_Pex = 0.2
    if error_x <= a_Pex:
        return 0
    elif error_x >= a_Pex and error_x <= b_Pex:
        return (error_x - a_Pex) / (b_Pex - a_Pex)
    elif error_x >= b_Pex:
        return 1
# funciones eY -----------------------------------------------
def control_erryN(error_y):  # funcion saturación de N de eY en Fctrl
    a_Ney = -0.2
    b_Ney = 0
    if error_y <= a_Ney:
        return 1
    elif error_y > a_Ney and error_y <= b_Ney:
        return (b_Ney - error_y) / (b_Ney - a_Ney)
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
def control_erryP(error_y):  # funcion saturacion de P de eY en Fctrl
    a_Pey = 0
    b_Pey = 0.2
    if error_y <= a_Pey:
        return 0
    elif error_y >= a_Pey and error_y <= b_Pey:
        return (error_y - a_Pey) / (b_Pey - a_Pey)
    elif error_y >= b_Pey:
        return 1
        # salidas
# funciones V -----------------------------------------------
def control_errvN(error_v):  # funcion saturación de N de V en Fctrl
    a_Nv = -0.02
    b_Nv = 0
    if error_v <= a_Nv:
        return 1
    elif error_v > a_Nv and error_v <= b_Nv:
        return (b_Nv - error_v) / (b_Nv - a_Nv)
    else:
        return 0
def control_errvZ(error_v):  # funcion triangular de Z de v en Fctrl
    a_Zv = -0.015
    b_Zv = 0
    d_Zv = 0.015
    if error_y >= a_Zey and error_y <= b_Zey:
        return (error_y - a_Zey) / (b_Zey - a_Zey)
    elif error_y >= b_Zey and error_y <= d_Zey:
        return (error_y - d_Zey) / (b_Zey - d_Zey)
    else:
        return 0
def control_errvP(error_v):  # funcion saturacion de P de v en Fctrl
    a_Pv = 0
    b_Pv = 0.02
    if error_v <= a_Pv:
        return 0
    elif error_v >= a_Pv and error_v <= b_Pv:
        return (error_v - a_Pv) / (b_Pv - a_Pv)
    elif error_v >= b_Pv:
        return 1
# funciones W -----------------------------------------------
def control_errwN(error_w):  # funcion saturación de N de W en Fctrl
    a_Nw = -0.35
    b_Nw = 0
    if error_w <= a_Nw:
        return 1
    elif error_w > a_Nw and error_w <= b_Nw:
        return (b_Nw - error_w) / (b_Nw - a_Nw)
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
def control_errwP(error_w):  # funcion saturacion de P de W en Fctrl
    a_Pw = 0
    b_Pw = 0.35
    if error_w <= a_Pw:
        return 0
    elif error_w >= a_Pw and error_w <= b_Pw:
        return (error_w - a_Pw) / (b_Pw - a_Pw)
    elif error_w >= b_Pw:
        return 1

# Reglas de control difuso basadas en las funciones de membresía
def aplicar_reglas_fuzzy(error_x, error_y, delta):
    # Calcular valores de funciones de membresía
    mu_exN = control_errxN(error_x)
    mu_exZ = control_errxZ(error_x)
    mu_exP = control_errxP(error_x)

    mu_eyN = control_erryN(error_y)
    mu_eyZ = control_erryZ(error_y)
    mu_eyP = control_erryP(error_y)

    mu_dN = control_errdN(delta)
    mu_dZ = control_errdZ(delta)
    mu_dP = control_errdP(delta)

    # Aplicar regla 1: Si Delta (Error del ángulo) es P, entonces W es P
    if mu_dP > 0:
        W = mu_dP  # Aquí W sigue la función P si Delta es P
    else:
        W = 0

    # Aplicar regla 2: Si Delta (Error del ángulo) es Z, entonces V es P y W es Z
    if mu_dZ > 0:
        V = mu_dZ * 1  # Aquí V es proporcional a la membresía de Z
        W = mu_dZ * 1
    else:
        V = 0

    # Aplicar regla 3: Si Delta (Error del ángulo) es N, entonces W es N
    if mu_dN > 0:
        W = mu_dN  # Aquí W sigue la función N si Delta es N

    # Aplicar regla 4: Si X es Z y Y es Z, entonces V es Z
    if mu_exZ > 0 and mu_eyZ > 0:
        V = min(mu_exZ, mu_eyZ)  # La intersección de ambas pertenencias define V

    # Aplicar regla 5: Si X es N y Y es Z, entonces V es Z
    if mu_exN > 0 and mu_eyZ > 0:
        V = min(mu_exN, mu_eyZ)

    return V, W
# Función para generar datos de prueba
def generar_pruebas():
    # Ejemplos de valores de prueba
    pruebas = [
        {"error_x": 0.1, "error_y": -0.1, "delta": 0.5},  # Caso 1
        {"error_x": -0.1, "error_y": 0.1, "delta": -0.2}, # Caso 2
        {"error_x": 0.05, "error_y": 0.05, "delta": 0.0}, # Caso 3
        {"error_x": -0.2, "error_y": -0.2, "delta": -0.5}, # Caso 4
        {"error_x": 0.2, "error_y": 0.2, "delta": 1.0},   # Caso 5
    ]
    return pruebas

# Probar el controlador difuso con estos valores
pruebas = generar_pruebas()
for prueba in pruebas:
    V, W = aplicar_reglas_fuzzy(prueba["error_x"], prueba["error_y"], prueba["delta"])
    print(f"Para error_x: {prueba['error_x']}, error_y: {prueba['error_y']}, delta: {prueba['delta']} => V: {V}, W: {W}")
# Reglas de control difuso ********************************************************
