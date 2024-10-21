import math
import time

from machine import Pin, PWM
# Configuración del pin PWM
pwm_pin = Pin(0)  # Cambia el número de pin según tu configuración
pwm = PWM(pwm_pin)
# Configuración de la frecuencia del PWM
pwm.freq(1000)  # Ajusta la frecuencia según tus necesidades

# Tabla de calibración (ejemplo)
# Formato: [(velocidad angular en rad/s, valor PWM), ...]
calibration_table = [
    (0.5, 10),
    (1.80, 15),
    (4, 20),
    (5, 25),
    (7, 30),
    (8, 35),
    (9, 40),
    (12, 45),
    (13, 50),
    (15, 55),
    (16, 60),
    (19, 65),
    (20, 70),
    (22, 75),
    (23, 80),
    (25, 85),
    (26, 90),
    (28, 95),
    (30, 100)
]

# Función de integración (sin imprimir "Tiempo transcurrido" y actualizando valores)
def integrator(value, previous_value, dt):
    integrated_value = previous_value + value * dt
    return integrated_value

# funciones de membresía control difuso
# entradas
# funciones eDelta -----------------------------------------------
def control_errdN(error_d): # saturación izquierda
    a_Ned = -1
    b_Ned = 0
    if error_d <= a_Ned:
        return 1
    elif a_Ned < error_d <= b_Ned:
        return (error_d - b_Ned) / (a_Ned - b_Ned)
    else:
        return 0

def control_errdZ(error_d):
    a_Zed = -1.5
    b_Zed = 0
    d_Zed = 1.5
    if a_Zed <= error_d <= b_Zed:
        return (error_d - a_Zed) / (b_Zed - a_Zed)
    elif b_Zed <= error_d <= d_Zed:
        return (error_d - d_Zed) / (b_Zed - d_Zed)
    else:
        return 0

def control_errdP(error_d): # saturación derecha
    a_Ped = 0
    b_Ped = 1
    if error_d <= a_Ped:
        return 0
    elif a_Ped < error_d <= b_Ped:
        return (error_d - a_Ped) / (b_Ped - a_Ped)
    else:
        return 1

# funciones eX -----------------------------------------------
def control_errxN(error_x):  # saturación izquierda
    a_Nex = -0.2
    b_Nex = 0
    if error_x <= a_Nex:
        return 1
    elif a_Nex < error_x <= b_Nex:
        return (error_x - b_Nex) / (a_Nex - b_Nex)
    else:
        return 0

def control_errxZ(error_x):  # triangular
    a_Zex = -0.15134
    b_Zex = 0
    d_Zex = 0.15134
    if a_Zex <= error_x <= b_Zex:
        return (error_x - a_Zex) / (b_Zex - a_Zex)
    elif b_Zex <= error_x <= d_Zex:
        return (error_x - d_Zex) / (b_Zex - d_Zex)
    else:
        return 0

def control_errxP(error_x):  # saturación derecha
    a_Pex = 0
    b_Pex = 0.2
    if error_x <= a_Pex:
        return 0
    elif a_Pex < error_x <= b_Pex:
        return (error_x - a_Pex) / (b_Pex - a_Pex)
    else:
        return 1

# funciones eY -----------------------------------------------
def control_erryN(error_y):  # saturación izquierda
    a_Ney = -0.2
    b_Ney = 0
    if error_y <= a_Ney:
        return 1
    elif a_Ney < error_y <= b_Ney:
        return (error_y - b_Ney) / (a_Ney - b_Ney)
    else:
        return 0

def control_erryZ(error_y):  # triangular
    a_Zey = -0.15
    b_Zey = 0
    d_Zey = 0.15
    if a_Zey <= error_y <= b_Zey:
        return (error_y - a_Zey) / (b_Zey - a_Zey)
    elif b_Zey <= error_y <= d_Zey:
        return (error_y - d_Zey) / (b_Zey - d_Zey)
    else:
        return 0

def control_erryP(error_y):  # saturación derecha
    a_Pey = 0
    b_Pey = 0.2
    if error_y <= a_Pey:
        return 0
    elif a_Pey < error_y <= b_Pey:
        return (error_y - a_Pey) / (b_Pey - a_Pey)
    else:
        return 1

# reglas de control difuso basadas en las funciones de membresía
def reglas_control_w(error_d, error_x, error_y):
    # calcular valores de reglas para W
    pw1 = control_errdP(error_d)  # regla 1
    zw2 = control_errdZ(error_d)  # regla 2
    nw3 = control_errdN(error_d)  # regla 3
    zw4 = min(control_errxZ(error_x), control_erryZ(error_y))  # regla 4
    zw5 = min(control_errxN(error_x), control_erryZ(error_y))  # regla 5
    # defuzzificación
    b_pw1 = 0.35
    b_zw2 = 0.0001
    b_nw3 = -0.35
    b_zw4 = 0.0001
    b_zw5 = 0.0001
    denom_w = pw1 + zw2 + nw3 + zw4 + zw5
    if denom_w < 1e-8:
        return 0
    numer_w = (pw1 * b_pw1 + zw2 * b_zw2 + nw3 * b_nw3 + zw4 * b_zw4 + zw5 * b_zw5)
    W_f = numer_w / denom_w
    return W_f

def reglas_control_v(error_d, error_x, error_y):
    # calcular valores de reglas para V
    pv2 = control_errdZ(error_d)  # regla 2
    zv4 = min(control_errxZ(error_x), control_erryZ(error_y))  # regla 4
    nv5 = min(control_errxN(error_x), control_erryZ(error_y))  # regla 5
    # defuzzificación
    b_pv2 = 0.02
    b_zv4 = 0.0001
    b_nv5 = -0.02
    denom_v = pv2 + zv4 + nv5
    if denom_v < 1e-8:
        return 0
    numer_v = (pv2 * b_pv2 + zv4 * b_zv4 + nv5 * b_nv5)
    V_f = numer_v / denom_v
    return V_f

# cálculo de delta
def calculo_delta(x, y):
    edelta = math.atan2(y, x)
    return edelta

# modelo de velocidad de ruedas
def mod_velr(V, W):
    fi1 = -V - W
    fi2 = V - W
    return fi1, fi2

# modelo de postura
def mod_post(Vcalc, Wcalc, theta):
    xp = Vcalc * math.cos(theta)
    yp = Vcalc * math.sin(theta)
    thp = Wcalc
    return xp, yp, thp

def integrar_todo(xp, yp, thp, fi1, fi2, x_prev, y_prev, th_prev, fi1_prev, fi2_prev, dt):
    xcalc = integrator(xp, x_prev, dt)
    ycalc = integrator(yp, y_prev, dt)
    thcalc = integrator(thp, th_prev, dt)
    fi1calc = integrator(fi1, fi1_prev, dt)
    fi2calc = integrator(fi2, fi2_prev, dt)
    return xcalc, ycalc, thcalc, fi1calc, fi2calc

def interpolate_pwm(Phi):
    # Si Phi está fuera de la tabla y es menor a 0.5, PWM es 0
    if Phi < 0.5:
        return 0
    # Si Phi está fuera de la tabla y es mayor a 30, PWM es 100
    elif Phi > 30:
        return 100
    # Encuentra los dos puntos más cercanos en la tabla de calibración
    for i in range(len(calibration_table) - 1):
        if calibration_table[i][0] <= Phi <= calibration_table[i + 1][0]:
            # Interpolación lineal
            Phi1, pwm1 = calibration_table[i]
            Phi2, pwm2 = calibration_table[i + 1]
            pwm_value = pwm1 + (Phi - Phi1) * (pwm2 - pwm1) / (Phi2 - Phi1)
            return int(pwm_value)
    return 0  # Si Phi está fuera del rango de la tabla

def set_motor_speed(Phi):
    pwm_value = interpolate_pwm(Phi)
    pwm.duty_u16(pwm_value * 655)  # Ajusta el valor PWM a 16 bits
    print(f"Velocidad angular (Phi): {Phi} rad/s, Valor PWM: {pwm_value}")

# main
def main():
    xreal = 0.0
    yreal = 0.0
    threal = 0.0

    dt = 0.1  # intervalo de tiempo (puedes ajustarlo)
    t_total = 5.0  # tiempo total de simulación
    t = 0.0

    x_prev = xreal
    y_prev = yreal
    th_prev = threal
    fi1_prev = 0.0
    fi2_prev = 0.0

    x_destino = float(input("Dame el valor de X destino: "))
    y_destino = float(input("Dame el valor de Y destino: "))
    print("\n")

    while t < t_total:
        eX = x_destino - x_prev
        eY = y_destino - y_prev
        delta = calculo_delta(eX, eY)
        edelta = delta - th_prev

        print("Tiempo: {:.2f} s".format(t))
        print("El valor de Delta es: {:.6f}".format(delta))
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
        fact1 = 1 / radio
        fact2 = long / radio

        inp1_modvelr = fact1 * Vcalc
        inp2_modvelr = fact2 * Wcalc

        fi1, fi2 = mod_velr(inp1_modvelr, inp2_modvelr)
        xp, yp, thp = mod_post(Vcalc, Wcalc, th_prev)

        print("Velocidad en X: {:.6f}".format(xp))
        print("Velocidad en Y: {:.6f}".format(yp))
        print("Velocidad angular theta: {:.6f}".format(thp))
        print("\n")

        xcalc, ycalc, thcalc, fi1calc, fi2calc = integrar_todo(
            xp, yp, thp, fi1, fi2, x_prev, y_prev, th_prev, fi1_prev, fi2_prev, dt
        )

        print("Posición rueda fi1: {:.6f}".format(fi1calc))
        print("Posición rueda fi2: {:.6f}".format(fi2calc))
        print("Posición en X: {:.6f}".format(xcalc))
        print("Posición en Y: {:.6f}".format(ycalc))
        print("Posición angular theta: {:.6f}".format(thcalc))
        print("-----------------------------------------------")
        print("\n")

        # Actualizar variables para la siguiente iteración
        x_prev = xcalc
        y_prev = ycalc
        th_prev = thcalc
        fi1_prev = fi1calc
        fi2_prev = fi2calc

        t += dt
        time.sleep(dt)

    # Ejemplo de uso
    print("Velocidad fi1: {:.6f}".format(fi1_prev))
    if fi1_prev > 0.0:
        Phi = fi1_prev  # Velocidad angular deseada en radianes/segundo
        set_motor_speed(Phi)
    
    print("Simulación finalizada.")

if __name__ == "__main__":
    main()
