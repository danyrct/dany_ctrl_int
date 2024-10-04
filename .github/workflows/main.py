import math
import time


# Función atan2 para calcular el error angular (Delta)
def calcular_delta(y, x):
    return math.atan2(y, x)

# funciones de membresía control difuso
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
# Controlador difuso basado en las reglas
def controlador_difuso(delta, x, y):
    delta_fuzz = pertenencia_angular(delta)
    x_fuzz = pertenencia_x(x)
    y_fuzz = pertenencia_y(y)

    # Inicialización de las salidas
    v = 0  # Velocidad
    w = 0  # Ángulo

    # Implementación de las reglas difusas (basadas en las reglas que proporcionaste)
    # reglas reglas min max etc
    return v, w


# Función para calcular los errores
def calcular_errores(x_ref, y_ref, x_actual, y_actual):
    error_x = x_ref - x_actual
    error_y = y_ref - y_actual
    delta = calcular_delta(error_y, error_x)
    return error_x, error_y, delta


# Bucle principal para controlar el coche
def main():
    # Valores de referencia (el punto objetivo donde debe ir el coche)
    x_ref = 0.3
    y_ref = 0.3

    while True:
        # Valores actuales simulados (podemos reemplazarlos con lecturas de sensores)
        x_actual = 0.2
        y_actual = 0.25
        # Calcular errores (error en X, error en Y, y el ángulo Delta)
        error_x, error_y, delta = calcular_errores(x_ref, y_ref, x_actual,
                                                   y_actual)
        # Mostrar los errores (equivalente a los "display" de Simulink)
        print(
            f"Error X1: {error_x:.2f}, Error Y1: {error_y:.2f}, Error Delta: {delta:.2f}"
        )
        # Aplicar el controlador difuso
        v, w = controlador_difuso(delta, error_x, error_y)
        # Imprimir las salidas del controlador (velocidad y ángulo)
        print(f"Velocidad (V): {v:.2f}, Angular (W): {w:.2f}")
        # Aquí puedes añadir el control de motores según las salidas V y W para mover el coche

        # Esperar un micro segundo antes de la siguiente iteración
        time.sleep(0.1)


# Ejecutar el programa
if __name__ == "__main__":
    main()
