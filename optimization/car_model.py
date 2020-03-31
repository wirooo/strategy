from math import sin, cos, sqrt
from numpy import arctan


class Car():

    g = 9.81  # Acceleration due to gravity in m/s^2
    rho = 1.225  # Density of air at room temperature

    def __init__(self, m=720, Crr=0.0015, CdA=0.15, max_force=100):
        self.m = m  # mass of car in kg
        self.Crr = Crr  # Rolling Resistance coefficient of the car
        self.CdA = CdA  # Drag coefficient of the car
        self.max_force = max_force  # Max force of motors in N

        # Ff = self.Crr * self.m * self.g * cos(theta)
        # Fg = self.m * self.g * sin(theta)
        # Fd = 0.5 * self.rho * self.CdA * vcurr ** 2
        # anet = -(Ff + Fd + Fg) / self.m

    def prev_roll_speed(self, dist, theta, target_v=0, calc_interval=1):
        """
        :param dist: distance to travel, in m
        :param theta: pitch of distance to be travelled, in radians
        :param target_v
        :param calc_interval: interval of distance calculations in m
        :return: initial velocity to reach target_v after rolling dist
        """
        total_dist = 0
        Ff = self.Crr * self.m * self.g * cos(abs(theta))
        Fg = self.m * self.g * sin(abs(theta))
        vprev = target_v
        while total_dist < dist:
            if theta < 0:
                # downhill: drag and friction are -'ve, gravity is +'ve
                if Fg > Ff + (0.5 * self.rho * self.CdA * vprev ** 2):
                    # accelerating downhill
                    return -1

            vprev = sqrt((vprev**2 + (2*calc_interval/self.m*(Ff+Fg)))/
                         (1-(calc_interval*self.rho*self.CdA/self.m)))
            total_dist += calc_interval
            if total_dist > dist:
                diff = total_dist - dist
                vprev = sqrt((vprev ** 2 + (2 * diff / self.m * (Ff + Fg))) /
                             (1 - (diff * self.rho * self.CdA / self.m)))

        return vprev

    def stop_dist(self, vprev, theta, vwind=0, stop_v=0, calc_interval=1):
        """
        :param vprev: initial velocity of the car in m/s
        :param theta: pitch of distance to be travelled, in radians
        :param vwind: velocity of the wind relative to the car in m/s
        :param stop_v: velocity of car to be considered a stop in m/s
        :param calc_interval: interval of distance calculations in m
        :return: distance the car would take to stop by rolling in m
        """
        vcurr = vprev + vwind
        total_dist = 0
        count = 0
        Ff = self.Crr * self.m * self.g * cos(theta)
        Fg = self.m * self.g * sin(theta)
        while vcurr > stop_v:
            count += 1
            Fd = 0.5 * self.rho * self.CdA * vcurr**2
            anet = -(Ff + Fd + Fg) / self.m
            # calculate velocity after travelling calc_interval distance
            # using v2^2 = v1^2 + 2ad
            if vcurr**2 < abs(2 * anet * calc_interval):
                # if calculation results in negative of sqrt
                # calculate for d, given v2 = stop_v
                total_dist += abs((vcurr**2)/(2 * anet))
                break
            else:
                # calculate for v2, given d = calc_interval
                total_dist += calc_interval
                vcurr = sqrt((vcurr**2) + (2 * anet * calc_interval))
        return total_dist



    def force_req(self, v, vwind=0, v_old=None, theta=0, timestep=30):
        """
        :param v: velocity of the car in m/s
        :param vwind: velocity of the wind relative to the car (+ve with car)
        :param v_old: speed of the car at the initial point
        :param theta: angle that must be climbed by the car in radians
        :param timestep: time in s between measurements
        :return force: force in N required to power the car
        """
        # If we don't set v_old, we assume v has not changed
        if v_old is None:
            v_old = v

        Ffric = self.m * self.g * cos(theta) * self.Crr
        Fdrag = 0.5 * self.rho * self.CdA * (v + vwind) ** 2
        Fg = self.m * self.g * sin(theta)
        Fa = self.m * (v - v_old) / timestep
        Fmotor = Fa + Ffric + Fdrag + Fg
        return Fmotor

    def max_velocity(self, v_old, vwind=0, theta=0, timestep=30):
        """
        :param v_old: speed of the car at the initial point
        :param vwind: velocity of the wind relative to the car (+ve with car)
        :param theta: angle that must be climbed by the car in radians
        :param timestep: time in s between measurement
        :return velocity: max velocity that the car can travel in m/s
        """
        # We need to solve the quadratic for an isolated v
        a = 0.5 * self.rho * self.CdA
        b = (self.m / timestep) + vwind * self.rho * self.CdA
        Ffric = self.m * self.g * cos(theta) * self.Crr
        Fg = self.m * self.g * sin(theta)
        c = Ffric + Fg - self.max_force - self.m * v_old / timestep
        v = (-b + sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        return v

    def energy_used(self, v_profile, e_profile, wind=0):
        """
        :param v_profile: a series of velocities in m/s separated by a distance
        :param e_profile: a series of elevations in m separated by a distance
        :return energy used: energy used in J for the path and velocity profile
        """
        # TODO: Add error handling for len(v_profile) != len(e_profile)
        energy = 0
        num_points = len(v_profile)
        # Note we will end 1 before because we don't care about the distance
        # that happens after the last point because it is the "finish line"
        for point in range(num_points - 1):
            v_new = v_profile[point + 1]
            v_old = v_profile[point]
            e_new = e_profile[point + 1][0]
            e_old = e_profile[point][0]
            e_gain = e_new - e_old
            dist = e_profile[point][1]
            theta = arctan(e_gain / dist)  # Calculate the angle of elev
            v_avg = (v_new + v_old) / 2
            timestep = dist / v_avg
            energy_used = self.force_req(v_new, wind, v_old,
                                         theta, timestep) * dist
            energy += energy_used
        return energy
