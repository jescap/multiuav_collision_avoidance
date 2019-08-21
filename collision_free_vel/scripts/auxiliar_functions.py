from math import sqrt, atan2, pi, sin, cos, acos


def vectors_distance_by_components(vector1, vector2):
    return sqrt(sum([(x-y)**2 for x,y in zip(vector1, vector2)]))

vect_dist = vectors_distance_by_components


# Return vector normalized by euclidean norm
def get_normalized_vector(vector):
    vx, vy, vz = vector
    n = vector_norm(vector)
    return (vx/n, vy/n, vz/n)


# Return vector norm
def vector_norm(vector):
    return sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)


# Distance from point to a line in space
def rect_point_space_distance(rect_point, rect_vector, point):

    p11, p12, p13 = point
    p21, p22, p23 = rect_point

    q1, q2, q3 = p11 - p21, p12 - p22, p13 - p23
    v1, v2, v3 = rect_vector

    # vectorial product
    p1, p2, p3 = (q2 * v3 - q3 * v2, -(q1 * v3 - q3 * v1), q1 * v2 - q2 * v1)

    s = (v1 ** 2 + v2 ** 2 + v3 ** 2)
    return sqrt((p1 ** 2 + p2 ** 2 + p3 ** 2) * s) / s if s else None


def vector2angles(vector):
    Vx, Vy, Vz = vector
    return atan2(Vy, Vx) % (2 * pi), acos(Vz) % (2 * pi), (Vx**2 + Vy**2 + Vz**2)**0.5


def angles2vector(phi, theta = pi/2, r = 1):
    return r*sin(theta)*cos(phi), r*sin(theta)*sin(phi), r*cos(theta)


# Verify if there is collision between one UAV and a list of UAVs and directions
def provoke_collisions(UAV, direction, UAV_list, detect_collision_method):
    for uav, d in UAV_list:
        if detect_collision_method(uav, d, UAV, direction):
            return True
    return False


# Detect if UAVs collide following their directions within a time interval
def is_collision_on_interval(UAV1, d1, UAV2, d2, time_interval):
    UAV1, d1, UAV2, d2 = (UAV1, d1, UAV2, d2) if UAV1.velocity < UAV2.velocity else (UAV2, d2, UAV1, d1)
    distance, vector = colliding_directions(UAV1, d1, UAV2, d2)

    if distance == None: return False

    time_collision = sqrt(distance**2 + vect_dist(UAV1.position, UAV2.position)**2)/vector_norm(vector)
    time = min(time_collision, time_interval)

    x, y = UAV2.position[0] + time*vector[0], UAV2.position[1] + time*vector[1]

    if vect_dist((x,y,0), UAV1.position) <= (UAV1.radio + UAV2.radio) and time > 0:
        return True
    return False


# Detect if there are collisions within a time interval no matter the region
def detect_collisions_on_time_interval(UAVs, time_interval):

    for i in range(len(UAVs)):
        for j in range(i+1, len(UAVs)):
            if is_collision_on_interval(UAVs[i], UAVs[i].direction, UAVs[j], UAVs[j].direction, time_interval):
                return True

    return False


# Given two UAVs with two directions, check whether they collide and the direction translated for UAV2
def colliding_directions(UAV1, d1, UAV2, d2):
    # leave in UAV1 the one with slowest velocity that was fixed

    vector = (d2[0]*UAV2.velocity - d1[0]*UAV1.velocity, d2[1]*UAV2.velocity - d1[1]*UAV1.velocity, d2[2]*UAV2.velocity - d1[2]*UAV1.velocity)
    return rect_point_space_distance(UAV2.position, vector, UAV1.position), vector


