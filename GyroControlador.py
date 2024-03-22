#!/usr/bin/env pybricks-micropython

from ucollections import namedtuple
import urandom

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Color, ImageFile, SoundFile
from pybricks.tools import wait, StopWatch

#Se inicializa el EV3
ev3 = EV3Brick()

# #Se inicializan los notores conectados a las llantas
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

# Se inicializa el motor conectado a los brazos
arm_motor = Motor(Port.B)

# Se inicialiaza el giroscopio
gyro_sensor = GyroSensor(Port.S2)

# Se inicializa el sensor ultrasónico
ultrasonic_sensor = UltrasonicSensor(Port.S4)

# Se inicializan los timers
fall_timer = StopWatch()
single_loop_timer = StopWatch()
control_loop_timer = StopWatch()
action_timer = StopWatch()


# Constantes empleadas en el control del programa.
GYRO_CALIBRATION_LOOP_COUNT = 200
GYRO_OFFSET_FACTOR = 0.0005
TARGET_LOOP_PERIOD = 15  # ms
ARM_MOTOR_SPEED = 600  # deg/s

# BANDERA QUE PERMITE ALTERNAR ENTRE MODOS
bandera = 0

# Acciones que cambian la dirección del robot
Action = namedtuple('Action ', ['drive_speed', 'steering'])

# Acciones predefinidas
STOP = Action(drive_speed=0, steering=0)
FORWARD_FAST = Action(drive_speed=95, steering=0)
FORWARD_SLOW = Action(drive_speed=40, steering=0)
BACKWARD_FAST = Action(drive_speed=-75, steering=0)
BACKWARD_SLOW = Action(drive_speed=-10, steering=0)
TURN_RIGHT = Action(drive_speed=0, steering=70)
TURN_LEFT = Action(drive_speed=0, steering=-70)


# Funciones para el movimiento del robot.
def move_forward():
    return FORWARD_SLOW

def move_backward():
    return BACKWARD_SLOW

def turn_left():
    return TURN_LEFT

def turn_right():
    return TURN_RIGHT


# Función para monitorear los brazos del robot
def update_action():
    arm_motor.reset_angle(0)
    action_timer.reset()

    # Inicio del movimiento
    yield move_forward()
    while action_timer.time() < 3000:
        yield STOP

    action = STOP
    yield action

    # Configuración "Por Defecto" del robot, en esta irá siempre hacia delante, y en caso de detectar un obstáculo 
    # con el sensor ultrasónico girará a la derecha o izqueirda de forma aleatoria.

    if bandera == 0:
        while True:

            # Si la medida del sensor es menor a 70 (obstáculo cercano), se irá para atrás y girara en una dirección
            if ultrasonic_sensor.distance() < 120:
                
                yield move_backward()
                ev3.speaker.beep(500, -1)
                arm_motor.run_angle(ARM_MOTOR_SPEED, 30, wait=False)
                while not arm_motor.control.done():
                    yield
                arm_motor.run_angle(ARM_MOTOR_SPEED, -60, wait=False)
                while not arm_motor.control.done():
                    yield
                arm_motor.run_angle(ARM_MOTOR_SPEED, 30, wait=False)
                while not arm_motor.control.done():
                    yield
                turn = urandom.choice([turn_left, turn_right])
                yield turn()
                action_timer.reset()
                while action_timer.time() < 4000:
                    yield

            elif ultrasonic_sensor.distance() > 1200:
                yield move_forward()
                #
                action_timer.reset()
                
                while action_timer.time() < 100:
                    yield
                ev3.speaker.beep(0, -1)

                yield action

            #Reseteo del timer
            action_timer.reset()
            while action_timer.time() < 100:
                yield

    # Modo de Programación de rutinas
    # Es posible modificar las acciones del robot, así como el tiempo que las ejecutará, para ello solo hay que 
    # eliminar y/o agrear acciones.

    if bandera==1:
        while True:

            yield move_forward()
            action_timer.reset()
            while action_timer.time() < 10000:
                yield
            yield action
            
            yield turn_right()
            action_timer.reset()
            while action_timer.time() < 4000:
                yield

            yield move_backward()
            action_timer.reset()
            while action_timer.time() < 15000:
                yield
            yield action

            yield turn_left()
            action_timer.reset()
            while action_timer.time() < 4000:
                yield
            yield action
            
          
# Función para detener el motor y el speaker
def stop_action():
    ev3.speaker.beep(0, -1)
    arm_motor.run_target(ARM_MOTOR_SPEED, 0)


while True:
    # Los ojos dormidos y la luz apagada nos permiten saber que el robot está esperando que se 
    # detenga cualquier movimiento antes de que el programa pueda continuar.
    ev3.screen.load_image(ImageFile.SLEEPING)
    ev3.light.off()

    # Se resetean los valores de los motores y el timer fall
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    fall_timer.reset()

    motor_position_sum = 0
    wheel_angle = 0
    motor_position_change = [0, 0, 0, 0]
    drive_speed, steering = 0, 0
    control_loop_count = 0
    robot_body_angle = -0.25

    # Dado que update_action() es un generador (usa "rendimiento" en lugar de "retorno"),
    # en realidad no ejecuta update_action() en este momento, sino que lo prepara para su uso más adelante.
    
    action_task = update_action()

    # Calibre el desplazamiento del giroscopio. Esto asegura que el robot esté perfectamente quieto 
    # asegurándose de que la velocidad medida no fluctúe más de 2 grados/s. La deriva del giroscopio 
    # puede hacer que la velocidad sea distinta de cero incluso cuando el robot no se está moviendo, 
    # por lo que guardamos ese valor para usarlo más adelante.

    while True:
        gyro_minimum_rate, gyro_maximum_rate = 440, -440
        gyro_sum = 0
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
            gyro_sensor_value = gyro_sensor.speed()
            gyro_sum += gyro_sensor_value
            if gyro_sensor_value > gyro_maximum_rate:
                gyro_maximum_rate = gyro_sensor_value
            if gyro_sensor_value < gyro_minimum_rate:
                gyro_minimum_rate = gyro_sensor_value
            wait(5)
        if gyro_maximum_rate - gyro_minimum_rate < 2:
            break
    gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

    # Los ojos despiertos y la luz verde nos permiten saber que el robot está listo para funcionar!
    ev3.speaker.play_file(SoundFile.SPEED_UP)
    ev3.screen.load_image(ImageFile.AWAKE)
    ev3.light.on(Color.GREEN)

    # Lazo de control principal para equilibrar el robot.
    while True:

        # Este temporizador mide cuánto dura un solo bucle. Esto se utilizará para 
        # ayudar a mantener constante el tiempo del bucle, incluso cuando se produzcan diferentes acciones.
        single_loop_timer.reset()

        # Esto calcula el período promedio del bucle de control. Esto se utiliza en el cálculo de la 
        # retroalimentación de control en lugar del tiempo de bucle único para filtrar fluctuaciones aleatorias.
        if control_loop_count == 0:

            # La primera vez que realizamos el ciclo, debemos asignar un valor para evitar dividir por cero más adelante.
            average_control_loop_period = TARGET_LOOP_PERIOD / 1000
            control_loop_timer.reset()
        else:
            average_control_loop_period = (control_loop_timer.time() / 1000 /
                                           control_loop_count)
        control_loop_count += 1

        # calcular el ángulo y la velocidad del cuerpo del robot
        gyro_sensor_value = gyro_sensor.speed()
        gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
        gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
        robot_body_rate = gyro_sensor_value - gyro_offset
        robot_body_angle += robot_body_rate * average_control_loop_period

        # calcular el ángulo y la velocidad de la rueda
        left_motor_angle = left_motor.angle()
        right_motor_angle = right_motor.angle()
        previous_motor_sum = motor_position_sum
        motor_position_sum = left_motor_angle + right_motor_angle
        change = motor_position_sum - previous_motor_sum
        motor_position_change.insert(0, change)
        del motor_position_change[-1]
        wheel_angle += change - drive_speed * average_control_loop_period
        wheel_rate = sum(motor_position_change) / 4 / average_control_loop_period

        # Este es el cálculo principal de la retroalimentación de control.
        output_power = (-0.01 * drive_speed) + (0.8 * robot_body_rate +
                                                15 * robot_body_angle +
                                                0.08 * wheel_rate +
                                                0.12 * wheel_angle)
        if output_power > 100:
            output_power = 100
        if output_power < -100:
            output_power = -100

        # Conducir los motores.
        left_motor.dc(output_power - 0.1 * steering)
        right_motor.dc(output_power + 0.1 * steering)

        # Compruebe si el robot se cayó. Si la velocidad de salida es de +/-100% 
        # durante más de un segundo, sabemos que ya no estamos equilibrando correctamente.
        if abs(output_power) < 100:
            fall_timer.reset()
        elif fall_timer.time() > 1000:
            break

        # Esto ejecuta update_action() hasta la siguiente declaración de "rendimiento".
        action = next(action_task)
        if action is not None:
            drive_speed, steering = action

        # Asegúrese de que el tiempo del bucle sea al menos TARGET_LOOP_PERIOD. El cálculo de potencia 
        # de salida anterior depende de tener una cierta cantidad de tiempo en cada bucle.
        wait(TARGET_LOOP_PERIOD - single_loop_timer.time())

    # Código para caída del robot

    # Stop all of the motors.
    stop_action()
    left_motor.stop()
    right_motor.stop()

    # Los ojos rotos y la luz roja nos hacen saber que el robot perdió el equilibrio.
    ev3.light.on(Color.RED)
    ev3.screen.load_image(ImageFile.KNOCKED_OUT)
    ev3.speaker.play_file(SoundFile.SPEED_DOWN)

    # Espere unos segundos antes de intentar equilibrar nuevamente.
    wait(3000)