from auxiliar_functions import get_normalized_vector, vector2angles, angles2vector
from math import pi, atan2
from random import uniform


class UAV:

    def __init__(self, position, velocity, radio, direction, goal_point):
        self.position = position
        self.velocity = velocity
        self.radio = radio
        self.direction = get_normalized_vector(direction)
        self.goal_point = goal_point

    def position_after_t(self, time, epsilon = 10**-6):

        px, py, pz = self.position
        vx, vy, vz = self.direction
        return (px + self.velocity*time*vx, py + self.velocity*time*vy, pz + self.velocity*time*vz)

    def get_optimal_direction(self):
        X1, Y1, Z1 = self.goal_point
        X2, Y2, Z2 = self.position
        return get_normalized_vector((X1-X2, Y1-Y2, Z1-Z2))

    # Generate directions in angles
    def generate_directions2D_randomly(self, span_theta_range, k):
        theta, _, _ = vector2angles(self.direction)

        directions = [self.get_optimal_direction()]
        for i in range(k):
            angle = uniform(theta-span_theta_range/2, theta+span_theta_range/2)%(2*pi)
            directions.append(angles2vector(angle, pi/2))

        directions.sort(key=lambda x: atan2(x[1], x[0]) % (2 * pi))
        return directions
