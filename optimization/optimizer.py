from car_model import Car
import numpy
from scipy.optimize import minimize
import argparse


global car
car = Car()

parser = argparse.ArgumentParser(description="Add parameters to model")
parser.add_argument("-d", "--distance", type=int, default=5490,
                    help="Distance to travel (m)")
parser.add_argument("-t", "--time", type=int, default=450,
                    help="Maximum allowable time (s)")
parser.add_argument("-v", "--min_velocity", type=int, default=5,
                    help="Minimum allowable velocity (m/s)")
parser.add_argument("-s", "--step", type=int, default=30,
                    help="Distance between elevation profile measurements")
parser.add_argument("-sv", "--stop_velocity", type=int, default=5,
                    help="Maximum velocity for stops/turns")
parser.add_argument("-ea", "--elev_absolute", action="store_true",
                    help="Set elevation interpretation to absolute from 0 "
                         "(interprets as delta from previous when absent)")
args = parser.parse_args()


def load_course_map(course_name="COTA"):
    """
    :param course_name: name of the course we are loading in
    :return elev_profile: map of the elevations on a course
    :return stops: list of all indices where stops occur
    :return total_dist: total distance of the track in m
    """
    if course_name == "COTA":
        with open('COTAelevation_var.txt', 'r') as file:
        # with open('COTAnonegative_var.txt', 'r') as file:
            line = file.readline()
            data = []
            while line:
                line = file.readline()
                data.append(line)
        clean_data = []
        for row in data:
            row = row.replace('\n', '').split(',')
            if row != '':
                clean_data.append(row)
        # Create the elevation profile
        elev_profile = []
        stops = []
        total_dist = 0
        for i in range(len(clean_data) - 1):
            if args.elev_absolute and i > 0:
                pitch = numpy.arctan((float(clean_data[i][1]) -
                                     float(clean_data[i-1][1]))
                                     / float(clean_data[i][2]))
            else:
                pitch = numpy.arctan(float(clean_data[i][1])
                                    / float(clean_data[i][2]))
            elev_profile.append((pitch, int(clean_data[i][2])))
            total_dist += int(clean_data[i][2])
            if len(clean_data[i]) > 3:
                stops.append(int(clean_data[i][0]))
                # stops.append(total_dist)
    if course_name == "ASC":
        pass
    return (elev_profile, stops, total_dist)

def generate_initial_section(avg_v, e_profile, min_velocity):
    """
    :param avg_v: Average velocity in m/s
    :param e_profile: List of pitches the car must travel
    :param min_velocity: Minimum allowable velocity
    :return: initial_profile: a velocity profile of the given section
    """
    initial_profile = [avg_v]
    for point in range(len(e_profile) - 1):  # We don't care about the endpoint
        pitch = e_profile[point][0]
        dist_step = e_profile[point][1]
        old_v = initial_profile[point - 1]
        timestep = dist_step / old_v
        max_v = car.max_velocity(old_v, theta=pitch, timestep=timestep)
        if max_v >= avg_v:
            initial_profile.append(float(avg_v))
        elif max_v >= min_velocity:
            initial_profile.append(float(max_v))
        else:
            initial_profile.append(float(min_velocity))
    # Here we generate the naive solution
    # traveling at the average velocity required
    return initial_profile

def generate_initial_profile(total_time, total_distance, e_profile,
                            min_velocity, stop_profile):
    """
    :param total_time: total time allowable in the race
    :param total_distance: total distance of the track
    :param e_profile: profile of elevations for the entire track
    :param min_velocity: minimum allowable velocity
    :param stop_profile: profile of stops in the race
    :return: initial_profile: array of sections that describe velocity between stops
    """
    total_distance = float(total_distance)
    avg_velocity = total_distance / total_time
    initial_profile = []
    prev_stop = 0
    for stop in stop_profile:
        e_section = e_profile[prev_stop:stop]
        prev_stop = stop
        section_distance = 0
        for entry in e_section:
            section_distance += entry[1]
        initial_profile.append(
            (generate_initial_section(avg_velocity, e_section, min_velocity), e_section))
    e_section = e_profile[prev_stop:]
    section_distance = 0
    for entry in e_section:
        section_distance += entry[1]
    initial_profile.append(
        (generate_initial_section(avg_velocity, e_section, min_velocity), e_section))
    return initial_profile

def apply_stops(v_profile, e_profile, stop_v=1, calc_interval=10):
    """
    :param v_profile: optimized velocity profile
    :param e_profile: elevation measurements
    :param stop_v: velocity to be travelling to be considered a stop
    :param calc_interval: interval at which we should be calculating at in m
    :return: updated v_profile to allow for stops
    """
    def apply_rolldown(v_section, e_section, stop_v=1, calc_interval=10):
        need_brakes = True
        v_section[-1] = stop_v
        for i in range(len(v_section) - 2, 0, -1):
            next_v = v_section[i + 1]
            dist = e_section[i + 1][1]
            theta = e_section[i + 1][0]
            curr_v = car.prev_roll_speed(dist, theta,
                                         target_v=next_v,
                                         calc_interval=calc_interval)
            if curr_v == -1:
                # return -1 means acceleration downhill, needs brakes
                need_brakes = True
                break
            if curr_v > v_section[i]:
                # rolldown dist <= v_section dist -> no brakes needed
                need_brakes = False
                break
            v_section[i] = curr_v
        return need_brakes, v_section

    new_v_profile = []
    for i in range(len(v_profile)):
        new_v_section = apply_rolldown(v_profile[i], e_profile[i], stop_v,
                                       calc_interval)
        new_v_profile.append(new_v_section)
    return new_v_profile


if __name__ == "__main__":

    map_data = load_course_map()
    elev_profile = map_data[0]
    stop_profile = map_data[1]
    distance = float(map_data[2])
    # Load in the distance and necessary time for a lap
    dist_step = args.step
    time = args.time  # max allowable time in s
    min_velocity = args.min_velocity
    max_stop_velocity = args.stop_velocity
    max_velocity = 25
    init_profile = generate_initial_profile(time, distance, elev_profile,
                                            min_velocity, stop_profile.copy(),
                                            max_stop_velocity)

    final_v_profile = []
    final_e_profile = []


    for section in init_profile:
        v_section = [max_stop_velocity] + section[0]
        e_section = [section[1][0]] + section[1]


        def objective(v_profile):
            energy = car.energy_used(v_section, e_section)
            return energy / 10000

        def time_constraint(v_profile):
            time_used = [e_section[i][1] / v_section[i]
                         for i in range(1, len(e_section))]
            section_distance = sum(e_section[i][1]
                                   for i in range(1, len(e_section)))
            section_time = float(time) * float(section_distance) / distance
            return section_time-sum(time_used)

        def speed_constraint(v_profile):
            error = 0
            for i in range(1, len(v_profile) - 1):
                timestep = e_section[i-1][1] / v_section[i-1]
                #TODO: add vwind to parameters of car.max_velocity()
                max_velocity = car.max_velocity(v_section[i-1],
                                                theta=e_section[i][0],
                                                timestep=timestep)
                if v_section[i] > max_velocity:
                    error += max_velocity - v_section[i]

            return error

        # initial guess
        v0 = numpy.asarray(v_section)
        print('Initial SSE Objective: ' + str(objective(v0)))

        # bounds
        elements = len(v0)
        bounds = [(max_stop_velocity, max_stop_velocity)] + \
                 ([(min_velocity, max_velocity)] * (elements - 1))

        # constraints
        condition1 = {'type': 'ineq', 'fun': time_constraint}
        condition2 = {'type': 'ineq', 'fun': speed_constraint}
        conditions = ([condition1, condition2])
        solution = minimize(objective, v0, method='trust-constr',
                            bounds=bounds, constraints=conditions, options={"disp": True,  "maxiter": 1000})

        v = solution.x
        message = solution.message
        status = solution.status
        print(message)
        print(status)
        print(v)
        final_v_profile.append(v[1:])
        final_e_profile.append(e_section[1:])

    final_v_profile = apply_stops(final_v_profile, final_e_profile,
                                  max_stop_velocity, calc_interval=1)
