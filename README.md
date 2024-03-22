# Mini-Proyecto-Giroboy

## Introducción
En esta práctica se armará y programará un robot usando el kit misdsotrms de lego. Este kit permite la construcción de distintas estructuras usando las piezas lego, señores y actuadores incluidos en el kit. El kit incluye un microcontrolador que puede ser reprogramado usando lenguajes de programación por bloques y también se pueden crear programas más complejos usando micro Python  y las librerías proporcionadas por lego. 
Para esta práctica se construirá y programara el robot Gyroby, la estructura de este robot ya esta definida dentro de las estructuras oficiales de lego, el manual de construcción se puede encontrar en el siguiente enlace: https://education.lego.com/v3/assets/blt293eea581807678a/blt9b683d3a8c4c4078/5f8801eba0ee6b216678e013/ev3-model-core-set-gyro-boy.pdf 

El robot LEGO Mindstorms Education EV3 es un robot compuesto por múltiples piezas de LEGO, sensores y motores que permiten construir una amplia gama de robots, dentro de los cuales destacan un brazo robótico, una banda clasificadora de piezas por color, un perro y un robot  que se mantiene en dos ruedas, además de las múltiples piezas de armado, el kit del robot está compuesto por las siguientes piezas:

 - Ladrillo EV3: el cual es el procesador central del robot.
 - Sensor de colores.
 - Sensore de tacto.
 - Sensor ultrasónico.
 - Giroscopio.
 - Motores DCC.

El EV3 de LEGO se puede programar de varias maneras para controlar sus motores y sensores. Aquí hay algunos de los modos de programación más comunes para el EV3:

LEGO Mindstorms EV3 Software: Este es el entorno de programación oficial proporcionado por LEGO. Utiliza una interfaz gráfica de arrastrar y soltar que es fácil de entender para principiantes. Los bloques de programación representan diferentes acciones, como mover motores, leer sensores y tomar decisiones lógicas. Es una opción popular para usuarios principiantes y escolares.

LEGO Mindstorms EV3 Programming App: LEGO también ofrece una aplicación móvil para programar el EV3 que es similar al software de escritorio. Permite a los usuarios crear programas utilizando una interfaz táctil en dispositivos móviles como tabletas y teléfonos inteligentes.

Programación basada en bloques (Blockly): Algunas plataformas de programación en línea, como Scratch y Blockly, ofrecen soporte para el EV3. Estas plataformas permiten a los usuarios programar utilizando bloques de código visual, lo que hace que la programación sea accesible para niños y principiantes.

LEGO Mindstorms EV3 MicroPython: Para usuarios más avanzados, LEGO ofrece soporte para programación en Python utilizando MicroPython. Esto permite a los usuarios escribir código en lenguaje Python para controlar el EV3, lo que ofrece más flexibilidad y control sobre el comportamiento del robot.

Programación en C o C++: Para usuarios avanzados que deseen un mayor control sobre el hardware y el rendimiento del EV3, es posible programar en lenguajes de programación de bajo nivel como C o C++. Esto generalmente implica utilizar bibliotecas de terceros y un entorno de desarrollo cruzado para compilar y cargar el código en el EV3.

La programacion del robot se realizara usando micro pythoncon la finalidad de lograr una mayor libertad en los comandos e instrucciones usadas, así como también aplicar y aprender los principales conceptos y funcionalidades de Python en la programación de robots.

La configuración "GiroBoy", es un modelo enfocado en la locomoción y capacidad de giro, puesto que emplea dos motores para el movimiento del robot, el cual se estabiliza gracias al giroscopio, ya que este le permite orientarse y calcular la velocidad angular. En los brazos se colocan los sensores de color y ultrasónico. Por lo que la tarea principal de esta practica es desarrollar un programa funcional usando mico Python que pueda mantener al robot en equilibro y que pueda moverse en distintas direcciones. 


## Objetivos

- Consturir y armar el robot GiroBoy de Lego.
- Lograr la estabilidad del robot mediante la programación de un controlador 
- Programar en el robot movimientos de avanzar, retroceder, girar a la izqueirda y derecha.
- Implementar la opción de programar distintas rutinas.


## Metodología

La metodología para dessarrollar y lograr los objetivos de la practica se ve compuesta por los siguientes pasos:

  1.- Armar el robot en la configuración "GiroBoy"
  
  2.- Instalar las heeramientas de software para la programación del robot
        a) Instalar Visual Studio code 
        b) Instalar la extensión de python en Visual Studio Code para su programación.
        c) Instalar la extencion de Lego Mindstorms en visual estudio code

 3.- Bootear al robot en modo programación para Python 
        a) Descarga y flashea la imagen de MicroPython EV3 en una tarjeta micro SD. El archivo e instrucciones de instalacion completas se puede encontrar en el siguiente enlace:      https://education.lego.com/en-us/product-resources/mindstorms-ev3/teacher-resources/python-for-ev3/ 
        b) Descargar una herramienta para flashear tarjetas sd, la aplicación recomendad y usada para esta práctica es Etcher, disponible en: https://etcher.balena.io/
        c) Una vez que la SD a sido flasheada el siguiente paso es: Inserta tu tarjeta micro SD en la ranura para tarjetas SD del ladrillo EV3 y enciéndelo.
        d) Conecta el ladrillo EV3 a tu computadora y esta listo para comenzar la programacion. 
  
  4.- Conectar vís USB el robot a la computadora y cargar el código.
  
  5.- Verificar el correcto funcionamiento del robot.
  
  6.- Modificar el código para cumplir con los objetivos establecidos sin el uso del sensor de colores.

Para el desarrollo del código la metodología empleada se dividió en 3 pasos. 
        Comprobación del funcionamiento de los sensores y actuadores
        Implementación del un controlador de estabilidad 
        Implementación de movimientos y rutinas 


## Código implementado
La programacion del robot usando micro python puede ser realizada de diversas maneras, esto gracias a que existen numerosas librerias para controlar motres, sesnores, etc. Tambien lego provee con un sufciente numero de librerias que permiten comandar los motores y la lectura de sesnores de diversas maneras. 
Para este proyecto decidimos usar las librerías proporcionadas y creadas por lego debido a que están verificadas y garantizan compatibilidad y un buen funcionamiento.  Las librerías usadas son: 
        #!/usr/bin/env pybricks-micropython

        from ucollections import namedtuple
        import urandom
        
        from pybricks.hubs import EV3Brick
        from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
        from pybricks.parameters import Port, Color, ImageFile, SoundFile
        from pybricks.tools import wait, StopWatch

El siguiente paso es inicilizar el microcontroldar principal 
        #Se inicializa el EV3
        ev3 = EV3Brick()

Ahora se deben crear los objetos correspondientes a cada sensor y actuador e indicar en que puerto del ev3 están conectados.

        #Se inicializan los notores conectados a las llantas
        left_motor = Motor(Port.D)
        right_motor = Motor(Port.A)
        
        #Se inicializa el motor conectado a los brazos
        arm_motor = Motor(Port.B)
        
        #Se inicialiaza el giroscopio
        gyro_sensor = GyroSensor(Port.S2)
        
        #Se inicializa el sensor ultrasónico
        ultrasonic_sensor = UltrasonicSensor(Port.S4)

 Ahora se deben crear los objetos correspondientes a cada sensor y actuador e indicar en que puerto del ev3 están conectados, así como también inicializar las variables globales que se van a usar.

        #Se inicializan los timers
        fall_timer = StopWatch()
        single_loop_timer = StopWatch()
        control_loop_timer = StopWatch()
        action_timer = StopWatch()
        
        #Constantes empleadas en el control del programa.
        GYRO_CALIBRATION_LOOP_COUNT = 200
        GYRO_OFFSET_FACTOR = 0.0005
        TARGET_LOOP_PERIOD = 15  # ms
        ARM_MOTOR_SPEED = 600  # deg/s
        

El programa tiene una variable llamada bandera que define como se va a comportar el robot, cuando la bandera es 0, el robot ejecutara acciones para ir hacia adelante indefinidamente hasta que detecte un objeto con el sensor ultrasónico, luego retroceder un poco, girar hacia la izquierda o derecha para evitar el obstáculo y repetir el ciclo. Otro modo de programación es cuando la bandera vale 1, en este modo se pueden programar la secuencia de acciones por un determinado tiempo. 

        #BANDERA QUE PERMITE ALTERNAR ENTRE MODOS
        bandera = 0
        
        #Acciones que cambian la dirección del robot
        Action = namedtuple('Action ', ['drive_speed', 'steering'])
        
        #Acciones predefinidas
        STOP = Action(drive_speed=0, steering=0)
        FORWARD_SLOW = Action(drive_speed=60, steering=0)
        BACKWARD_SLOW = Action(drive_speed=-10, steering=0)
        TURN_RIGHT = Action(drive_speed=0, steering=70)
        TURN_LEFT = Action(drive_speed=0, steering=-70)
        
        
        #Funciones para el movimiento del robot.
        def move_forward():
            return FORWARD_SLOW
        
        def move_backward():
            return BACKWARD_SLOW
        
        def turn_left():
            return TURN_LEFT
        
        def turn_right():
            return TURN_RIGHT
        
        #Función para monitorear los brazos del robot
        def update_action():
            arm_motor.reset_angle(0)
            action_timer.reset()
        
            # Inicio del movimiento
            yield move_forward()
            while action_timer.time() < 3000:
                yield STOP
        
            action = STOP
            yield action
        
            #Configuración "Por Defecto" del robot, en esta irá siempre hacia delante, y en caso de detectar un obstáculo 
            #con el sensor ultrasónico girará a la derecha o izqueirda de forma aleatoria.

            if bandera == 0:
                while True:

                    #Si la medida del sensor es menor a 70 (obstáculo cercano), se irá para atrás y girara en una dirección
                    if ultrasonic_sensor.distance() < 70:
                        
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
        
                    elif ultrasonic_sensor.distance() > 40:
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
        
Cuando el valor de la bandera es 1, se ejecutara esta parte del código donde  se programaran las rutinas, para este ejemplo la rutina programada es moverse hacia adelante por 10 segundos, girar a la derecha por 4 segundos, moverse hacia atrás por 15 segundos y como última acción girar a la izquierda, en este modo la rutina es repetida indefinidamente a menos que le robot pierda el equilibrio 

            #Modo de Programación de rutinas
            #Es posible modificar las acciones del robot, así como el tiempo que las ejecutará, para ello solo hay que 
            #eliminar y/o agrear acciones.
        
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
                    
                
        #Función para detener el motor y el speaker
        def stop_action():
            ev3.speaker.beep(0, -1)
            arm_motor.run_target(ARM_MOTOR_SPEED, 0)
        
        
        while True:
            #Los ojos dormidos y la luz apagada nos permiten saber que el robot está esperando que se 
            #detenga cualquier movimiento antes de que el programa pueda continuar.
            ev3.screen.load_image(ImageFile.SLEEPING)
            ev3.light.off()
        
            #Se resetean los valores de los motores y el timer fall
            left_motor.reset_angle(0)
            right_motor.reset_angle(0)
            fall_timer.reset()
        
            motor_position_sum = 0
            wheel_angle = 0
            motor_position_change = [0, 0, 0, 0]
            drive_speed, steering = 0, 0
            control_loop_count = 0
            robot_body_angle = -0.25
        
            #Dado que update_action() es un generador (usa "rendimiento" en lugar de "retorno"),
            #en realidad no ejecuta update_action() en este momento, sino que lo prepara para su uso más adelante.
            
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
        
            #Los ojos despiertos y la luz verde nos permiten saber que el robot está listo para funcionar!
            ev3.speaker.play_file(SoundFile.SPEED_UP)
            ev3.screen.load_image(ImageFile.AWAKE)
            ev3.light.on(Color.GREEN)
        
            #Lazo de control principal para equilibrar el robot.
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
        
El robot puede conservar el equilibrio debido a que los motores son comandados mediante potencia con el comando motor.dc. este comando manda la potencia al motor necesaria para moverse y no mantener el equilibrio. El controlador determina la potencia adecuada para que el robot pueda mantener el equilibrio mediante los parámetros de velocidad, la velocidad angular del robot, el angulo del robot con respecto a la vertical y el Angulo de las ruedas.  A cada uno de estos valores se les asigna un peso constante de la potencia de salida. 
En caso de que el calculo de potencia sobre pase 100, significa que el valor requerido de potencia es mayor a lo que el motor puede entregar por lo que se limitara este valor a 100 o -100 dependiendo la dirección. 

                # Este es el cálculo principal de la retroalimentación de control.
                output_power = (-0.01 * drive_speed) + (0.8 * robot_body_rate +
                                                        15 * robot_body_angle +
                                                        0.08 * wheel_rate +
                                                        0.12 * wheel_angle)
                if output_power > 100:
                    output_power = 100
                if output_power < -100:
                    output_power = -100

Una vez que el valor de potencia requerido para mantener el equilibrio ha sido calculado por el controlador, es mandado a los motores mediante el comando motor.dc, Como ultimo parámetro a considerar se toman los giros izquierda o derecha y estos son multiplicados por una constante, así el robot es capaz de dar la vuelta sin perder el equilibrio, 

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
        
            #Código para caída del robot
        
            #Stop all of the motors.
            stop_action()
            left_motor.stop()
            right_motor.stop()
        
            #Los ojos rotos y la luz roja nos hacen saber que el robot perdió el equilibrio.
            ev3.light.on(Color.RED)
            ev3.screen.load_image(ImageFile.KNOCKED_OUT)
            ev3.speaker.play_file(SoundFile.SPEED_DOWN)
        
            #Espere unos segundos antes de intentar equilibrar nuevamente.
            wait(3000)

## Resultados

Tras la realización del código se comprueba el correcto funcionamiento del mismo cargandolo al robot. 
La parte de la programación fue la más complicada del proyecto, sin embargo, el robot cumple con los requisitos de movilidad planteados. El reto más difícil de la programación del robot fue el controlador para el equilibrio. El robot es una estructura compleja para mantener equilibrio debido a que la mayor parte del peso se encuentra distribuido en la parte superior, es decir el centro de masa esta muy arriba y el robot tiende a caerse fácilmente, los apoyos del robot únicamente son las llantas, por lo que prácticamente es un péndulo invertido.  El controlador implementado funciona lo suficientemente bien como para mantener el equilibrio del robot en una superficie plana, con pequeños desniveles o imperfecciones en la superficie. El controlador del robot es capaz de mantener el equilibrio por largos periodos de tiempo si no es perturbado por una fuerza externa, aun así, en la presencia de una fuerza externa menor es capaz de corregir su posición y seguir en equilibrio. En cuanto a la movilidad del robot, es difícil hacer que el robot avance continuamente, por lo que se decidió hacerlo avanzar por pequeños periodos de tiempo, y recuperar el equilibrio, el movimiento hacia adelante es lento, pero garantiza una estabilidad.  

La siguiente imagen muestra 3 estados del robot, el de la izquierda el robot "dormido" y esperando a iniciar, en medio el robot "despierto" y ejecutando el proceso, y a la deracha el robot tras sufrir una caída.

![image](https://github.com/DiegoJGutierrezReyes/Mini-Proyecto-Giroboy/assets/132300202/062244ae-94ee-436d-b02e-b9b71d063b15)

Se probo el robot en los dos modos, en ambos el resultado fue bueno y cumplió con los requisitos planteados
### Evidencias 
Se implementó una rutina predetermianda para el robot cuando la variable *bandera=0*, en esta rutina el robot avanzará hacia adelante de forma constante y al detectar un obstáculo con el sensor ultrasónico girará a la derecha o izquierda de forma aleatoria. Este modo se observa en el siguiente video: https://youtu.be/TARbWWwveg8


VIDEO DEL MODO PREDETERMINADO: https://youtu.be/mpFF__DfdWc

Por su parte, cuando *bandera=1* se entra en el modo de programción, donde se ejecutará en bucle la secuencia creada, a esta secuencia se le puede cambiar el orden de las acciones, asi como el tiempo que debe de ejecutarlas. El video de una secuencia programada se muestra a continuación:

VIDEO DE LA RUTINA PROGRAMADA

## Conclusiones

