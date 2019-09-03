from math import sqrt, atan2, pi, sin, cos, acos


def vectors_distance_by_components(vector1, vector2):
    return sqrt(sum([(x-y)**2 for x,y in zip(vector1, vector2)]))

vect_dist = vectors_distance_by_components

# Dado un vector devuelve el vector normalizado con norma euclideana
def get_normalized_vector(vector):
    vx, vy, vz = vector
    n = vector_norm(vector)
    return (vx/n, vy/n, vz/n)


# Retorna la norma de un vector
def vector_norm(vector):
    return sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)


# Distancia de punto a recta en el espacio
def rect_point_space_distance(rect_point, rect_vector, point):

    p11, p12, p13 = point
    p21, p22, p23 = rect_point

    q1, q2, q3 = p11 - p21, p12 - p22, p13 - p23
    v1, v2, v3 = rect_vector

    # producto vectorial
    p1, p2, p3 = (q2 * v3 - q3 * v2, -(q1 * v3 - q3 * v1), q1 * v2 - q2 * v1)

    s = (v1 ** 2 + v2 ** 2 + v3 ** 2)
    return sqrt((p1 ** 2 + p2 ** 2 + p3 ** 2) * s) / s if s else None


def vector2angles(vector):
    Vx, Vy, Vz = vector
    # phi, theta, r
    return atan2(Vy, Vx) % (2 * pi), acos(Vz) % (2 * pi), (Vx**2 + Vy**2 + Vz**2)**0.5


def angles2vector(phi, theta = pi/2, r = 1):
    return r*sin(theta)*cos(phi), r*sin(theta)*sin(phi), r*cos(theta)


# Verificar si existe colision entre un dron y una lista de drones y direcciones
def provoke_collisions(UAV, direction, UAV_list, detect_collision_method):
    for uav, d in UAV_list:
        if detect_collision_method(uav, d, UAV, direction):
            return True
    return False


# Detecta si los UAVs colisionan siguiendo sus direcciones en un intervalo de tiempo
def is_collision_on_interval(UAV1, d1, UAV2, d2, time_interval):
    UAV1, d1, UAV2, d2 = (UAV1, d1, UAV2, d2) if UAV1.velocity < UAV2.velocity else (UAV2, d2, UAV1, d1)
    distance, vector = colliding_directions(UAV1, d1, UAV2, d2)

    if distance == None or distance > (UAV1.radio + UAV2.radio): return False

    time_collision = sqrt(distance**2 + vect_dist(UAV1.position, UAV2.position)**2)/vector_norm(vector)
    if time_collision > time_interval:
        return False

    return True


# Detecta si hay colisiones en un intervalo de tiempo sin importar la region
def detect_collisions_on_time_interval(UAVs, time_interval):

    for i in range(len(UAVs)):
        for j in range(i+1, len(UAVs)):
            if is_collision_on_interval(UAVs[i], UAVs[i].direction, UAVs[j], UAVs[j].direction, time_interval):
                return True

    return False


# Dados los drones UAV1, UAV2 siguiendo dos direcciones saber si colisionan y la direccion trasladada para UAV2
def colliding_directions(UAV1, d1, UAV2, d2):
    # dejar en UAV1 el de menor velocidad que se fijara

    vector = (d2[0]*UAV2.velocity - d1[0]*UAV1.velocity, d2[1]*UAV2.velocity - d1[1]*UAV1.velocity, d2[2]*UAV2.velocity - d1[2]*UAV1.velocity)
    return rect_point_space_distance(UAV2.position, vector, UAV1.position), vector

