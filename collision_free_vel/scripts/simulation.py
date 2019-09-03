from auxiliar_functions import *
from classes import *
from algorithms import *
from random import shuffle

# - Experimento donde todos lo UAVs estan en una fila y tratan de alcanzar un punto a una
# distancia distance (eje y)
# - Los UAVs de radio *radio se encuentran en el eje x alineados y su destino tambien esta
# alineado en el eje x
# - Los UAVs y sus destinos estan aleatoriamente distribuidos a lo largo de *total_line_span
# - El primer UAV se debe encontrar a en la posicion *uavs_start_axis del eje x

def generate_UAVS_row_experiment(uavs_start_point, distance_y_axis, n, total_line_span, radio = 1):

    x, y = uavs_start_point
    uav_distance = total_line_span / n
    x_axis = [x + i * uav_distance for i in range(n)]
    x_axis_targets = x_axis[:]
    while x_axis == x_axis_targets:
        shuffle(x_axis_targets)

    uavs = [UAV((x_axis[i], y, 0), 1, radio, (1, 1, 1), (x_axis_targets[i], y + distance_y_axis, 0)) for i in range(n)]
    for uav in uavs:
        uav.direction = uav.get_optimal_direction()

    return uavs


def simulate(UAVs, k, max_span_angle, time_interval, ac_method, epsilon=0.0):

    detect_method = lambda UAV1, d1, UAV2, d2 : is_collision_on_interval(UAV1, d1, UAV2, d2, time_interval)
    cost_function = lambda x, y: abs(vector2angles(x)[0] -  vector2angles(y)[0])
    vect_dist = vectors_distance_by_components
    collision_number = 0
    solutions_number = []
    deviation = {uav:0 for uav in UAVs}
    no_solution = False

    dict_points = {}
    for uav in UAVs:
        dict_points[uav.goal_point] = ([uav.position[0]], [uav.position[1]])

    distances = [] # la distancia desde el final del recorrido hasta el objetivo prefjado

    try:
        while UAVs:

            for uav in UAVs:
                uav.direction = uav.get_optimal_direction()

            if detect_collisions_on_time_interval(UAVs, time_interval):
                collision_number += 1
                directions = [uav.generate_directions2D(pi/4, k) for uav in UAVs]

                result, no = ac_method(UAVs, directions, cost_function, detect_method)
                solutions_number.append(no)

                if not result:
                    break

                for uav, d in result:
                    uav.direction = d

            arrive = []
            for i, uav in enumerate(UAVs):

                if epsilon:
                    pos = uav.position_after_t(time_interval - epsilon)
                else:
                    pos = uav.position_after_t(time_interval)

                distance2_goal_line = pos[1] - uav.goal_point[1]
                if pos == uav.goal_point:
                    arrive.append(i)
                    uav.position = pos
                    deviation[uav] += time_interval + abs(pos[0] - uav.goal_point[0])

                elif distance2_goal_line >= 0:
                    arrive.append(i)
                    time_intersect = (uav.goal_point[1] - uav.position[1])/cos(abs(vector2angles(uav.direction)[0] - pi/2))
                    uav.position = uav.position_after_t(time_intersect)
                    deviation[uav] += time_intersect + abs(uav.position[0] - uav.goal_point[0])

                else:
                    uav.position = pos
                    deviation[uav] += time_interval


            for uav in UAVs:
                dict_points[uav.goal_point][0].append(uav.position[0])
                dict_points[uav.goal_point][1].append(uav.position[1])

            # Eliminar de la lista los UAVs que ya estan en su goal position
            for i in range(len(arrive)):
                uav = UAVs.pop(arrive[-(i+1)])
                distances.append(vect_dist(uav.position, uav.goal_point))

    except:
        no_solution = True

        for uav in UAVs:
            deviation[uav] = vectors_distance_by_components(uav.goal_point, uav.starting_position)

    finally:
        pass

        # from matplotlib import pyplot as plt
        # for xs, ys in dict_points.values():
        #     plt.plot(xs, ys, "-b.")
        # plt.show()


    if no_solution:
        return False, collision_number


    deviations = [abs(dev/vectors_distance_by_components(uav.goal_point, uav.starting_position)) for uav, dev in deviation.items()]
    d = max(distances) if distances else -1
    return solutions_number, d, collision_number, deviations




if __name__ == "__main__":

    # PARAMETROS A ENTRAR
    # Numero de UAVs
    # Numero de direcciones a tomar
    # Angulo de giro maximo para los UAVs
    # Definir la ventana de tiempo
    # Radio de los UAVS
    # Velocidad inicial de los UAVs (igual para todos)

    # PARAMETROS DE SALIDA
    # Tiempos de cada solucion de collisiones optimizada (brute force)
    # Numero de soluciones en cada instancia del problema collision avoidance
    # Distancia maxima entre objetivo y final
    # cantidad de colisiones detectada

    # GRAFICAS
    # Radio contra cantidad de soluciones (promedio, maximo y minimo de la cantidad de soluciones que hay por cada radio)
    # Tiempo de ejecucion por numero de UAVs (promedio, maximo y minimo del tiempo de solucion de los conflicto)
    # Cantidad de conflictos contra proporcion entre antidad de conflictos y numero de pasos

    # # ************************************************************
    #
    # SIMULATION
    no_simulaciones = 10
    no_vehicle = 5
    radio = 0.3
    k = 10
    time_interval = 20
    max_deviation = pi/4
#     no_simulaciones = int(input("Entrar numero de simulaciones: "))
#     no_vehicle = int(input("Entrar numero de vehiculos: "))
#     radio = float(input("Entrar radio de vehiculos: "))
#     k = int(input("Entrar cantidad de direcciones por vehiculo: ")) - 1
#     time_interval = int(input("Entrar intervalo de tiempo: "))
#     max_deviation = radians(int(input("Desviacion maxima permitida (grados): ")))

    initial_point = (1,0)
    board_height = 100
    board_width = 100

    no_s, dm, nc, dev = [], 0, [], []

    for i in range(no_simulaciones):
        UAVs = generate_UAVS_row_experiment(initial_point, board_height, no_vehicle, board_width, radio)
        result = simulate(UAVs, k, max_deviation, time_interval, bf_minimize_max_deviation)

        if not result:
            continue

        no_soluciones, dist_max, no_colisiones, deviations = result
        no_s += no_soluciones
        dm = max(dm, dist_max)
        nc.append(no_colisiones)


    # # ******************************************************************
    # # Metodos para graficar
    #
    # vehicle_area_VS_n_solution()
    #
    # vehicle_area_VS_n_colisiones()
    #
    # k_VS_n_soluciones()
    #
    # n_vehiculos_VS_tiempo()
    #
    # intervalo_VS_n_colisiones()
