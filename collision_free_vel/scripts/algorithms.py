from random import shuffle
from auxiliar_functions import provoke_collisions

# Calcula todas las soluciones factibles y escoge la optima segun el criterio de minimizar la maxima desviacion
def bf_minimize_max_deviation(UAVs, directions_list, cost_function, detect_collision_method):
    
    def criteria(uavs):
        return max([cost_function(uav.direction, d) for uav, d in uavs])

    result_list = optimize_brute_force(UAVs, directions_list, 0, [], [], detect_collision_method)
    result_list.sort(key= lambda x: criteria(x))

    result, value = [result_list[0]], criteria(result_list[0])
    for i in range(1, len(result_list)):
        if criteria(result_list[i]) != value:
            break
        result.append(result_list[i])

    def criteria2(uavs):
        return sum([cost_function(uav.direction, d) for uav, d in uavs])

    return select_optimum(result, criteria2), len(result_list)


def optimize_brute_force(UAVs, directions_list, index, result, result_list, detect_collision_method):

    if index == len(UAVs):
        result_list.append(result)
        return result_list

    for k in directions_list[index]:
        if not provoke_collisions(UAVs[index], k, result, detect_collision_method):
            result_list = optimize_brute_force(UAVs, directions_list, index+1, result + [(UAVs[index], k)], result_list, detect_collision_method)

    return result_list



def select_optimum(result_list, cost):
    if not result_list: return False

    result_list.sort(key=lambda x: cost(x))
    result, value = [result_list[0]], cost(result_list[0])
    for i in range(1, len(result_list)):
        if cost(result_list[i]) != value:
            break
        result.append(result_list[i])
    shuffle(result)
    return result[0]
