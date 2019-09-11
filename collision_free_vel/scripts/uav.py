from auxiliar_functions import get_normalized_vector, vector2angles, angles2vector, vectors_distance_by_components
from math import sqrt, pi, sin, cos, atan2, radians, degrees
from random import uniform
import numpy


class UAV:

    def __init__(self, position, velocity, radio, direction, goal_point):
        self.starting_position = position
        self.position = position
        self.velocity = velocity
        self.radio = radio
        self.direction = get_normalized_vector(direction)
        self.goal_point = goal_point

    def position_after_t(self, time, epsilon = 10**-6):

        px, py, pz = self.position
        vx, vy, vz = self.direction
        return (px + self.velocity*time*vx, py + self.velocity*time*vy, pz + self.velocity*time*vz)

    # los rangos son los valores que limitan el desvio, o sea, el UAV solo podra tomar desde d-range/2 hasta d+range/2
    # k es el paso que en cada rango salta
    def generate_directions_by_vector_ranges(self, Xrange, Yrange, Zrange, k):

        Xstep, Ystep, Zstep = Xrange / k, Yrange / k, Zrange / k
        x, y, z = self.direction
        directions = []
        for i in range(k):
            for j in range(k):
                for k in range(k):
                    d = (x - Xrange/2 + i * Xstep, y - Yrange/2 + j * Ystep, z - Zrange/2 + k * Zstep)
                    if d != (0,0,0):
                        directions.append(get_normalized_vector(d))

        directions.append(self.direction)
        return directions

    # los rangos se dan en radianes para los planos phi -> XY, theta -> eje Z
    def generate_directions_by_span_angles(self, span_phi_range, span_theta_range, k):

        phi, theta, _ = vector2angles(self.direction)
        n = int(sqrt(k))

        directions = []
        for i in range(n):
            for j in range(n):
                phi_i = (phi - span_phi_range + 2 * i * span_phi_range / n + span_phi_range / n) % (2 * pi)
                theta_i = (theta - span_theta_range + 2 * i * span_theta_range / n + span_theta_range / n) % (2 * pi)
                directions.append(angles2vector(phi_i, theta_i))

        return directions


    def get_optimal_direction(self):
        X1, Y1, Z1 = self.goal_point
        X2, Y2, Z2 = self.position
        return get_normalized_vector((X1-X2, Y1-Y2, Z1-Z2))

    # Genera las direcciones en angulos
    def generate_directions2D_randomly(self, span_theta_range, k):
        theta, _, _ = vector2angles(self.direction)

        directions = [self.get_optimal_direction()]
        for i in range(k-1):
            angle = uniform(theta-span_theta_range/2, theta+span_theta_range/2)%(2*pi)
            directions.append(angles2vector(angle))

        directions.sort(key=lambda x: atan2(x[1], x[0]) % (2 * pi))
        return directions

    # Genera las direcciones en angulos
    def generate_directions2D(self, span_theta_range, k):

        theta, _, _ = vector2angles(self.direction)

        directions = []
        for i in range(k):
            theta_i = (theta - span_theta_range + 2 * i * span_theta_range / k + span_theta_range / k) % (2 * pi)
            directions.append(angles2vector(theta_i))
        
        directions.append(self.get_optimal_direction())
        directions.sort(key=lambda x: atan2(x[1], x[0]) % (2 * pi))
        return directions


    def generate_directions_by_step(self, k, min_step=3, max_turn_angle=pi/4):

        # step = min(min_step, degrees(max_turn_angle)/k)
        step = min_step
        angle = degrees(vector2angles(self.direction)[0]) % 360
        directions = [self.direction]
        for i in range(k//2):
            directions += [angles2vector(radians((angle-(i+1)*step) % 360)), angles2vector(radians((angle+(i+1)*step) % 360))]

        directions.sort(key=lambda x: atan2(x[1], x[0]) % (2 * pi))
        return directions


    def generate_free_obstacles_directions(self, k, obstacles):
        angles = [angles2vector(radians(a)) for a in range(0, 360, 3) if not collide_with_any_obstacle(self, angles2vector(radians(a)), obstacles)]
        angles.sort(key=lambda x: vectors_distance_by_components(x, self.direction))

        return angles[:k]



class Region:
    # La region es un cubo en el espacio
    def __init__(self, init_point, width, long, height):
        self.init_point = init_point
        self.width = width # ancho
        self.long = long # largo
        self.height = height # altura

    def contains(self, point):
        px, py, pz = self.init_point
        x, y, z = point
        return (px <= x <= px + self.width) and (py <= y <= py + self.width) and (pz <= z <= pz + self.width)


class Obstacle:

    def __init__(self, position, radio):
        self.position = position
        self.radio = radio




if __name__ == "__main__":

    from math import atan2, degrees

    uav = UAV((1,1,0), 1, 1, (1,0,0), (20,0,0))
    o1 = Obstacle((4, 2, 0), 1)
    o2 = Obstacle((6,0, 0), 1)
    o3 = Obstacle((-5, 1, 0), 2)
    print([degrees(vector2angles(v)[0]) for v in uav.generate_free_obstacles_directions(40, [o1])])


    # angles = [vector2angles(v) for v in uav.generate_directions_by_span_angles(pi/2, pi/2, 25)]
    # angles = [(degrees(a1), degrees(a2)) for a1, a2, _ in angles]
    # print(angles)

    # print(uav.get_optimal_direction())

