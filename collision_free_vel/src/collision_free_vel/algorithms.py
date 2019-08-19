from auxiliar_functions import provoke_collisions

# Compute all feasible solutions and choose the optimum minimizing the maximum deviation
def bf_minimize_max_deviation(UAVs, directions_list, cost_function, detect_collision_method):

    # Criterion to minimize the maximum deviation
    def criteria(uavs):
        return max([cost_function(uav.direction, d) for uav, d in uavs])

    # list with all feasible solutions
    result_list = optimize_brute_force(UAVs, directions_list, 0, [], [], detect_collision_method)
    result_list.sort(key= lambda x: criteria(x))

    # keep solutions with the same maximum deviation
    result = []
    for i in range(1,len(result_list)):
        if criteria(result_list[i]) != criteria(result_list[i-1]):
            break
        result.append(result_list[i])

    # Criterion to choose according to the minimum sum of deviations
    def criteria2(uavs):
        return sum([cost_function(uav.direction, d) for uav, d in uavs])

    # return from solutions with equal maximum deviation, the one with minimum sum of deviations 
    return select_optimum(result, criteria2), len(result)


def optimize_brute_force(UAVs, directions_list, index, result, result_list, detect_collision_method):

    if index == len(UAVs):
        result_list.append(result)
        return result_list

    for k in directions_list[index]:
        if not provoke_collisions(UAVs[index], k, result, detect_collision_method):
            result_list = optimize_brute_force(UAVs, directions_list, index+1, result + [(UAVs[index], k)], result_list, detect_collision_method)

    return result_list


def select_optimum(result_list, cost):
    if not result_list:
        return False
    costs = list(map(cost, result_list[:]))
    index = costs.index(min(costs))
    return result_list[index]