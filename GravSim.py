import numpy as np
from math import pi
import time
import sys
import matplotlib.pyplot as plt



A = 10
dt = .01

## Push test. New desktop 9/25

def custom_norm(array):
    return sum([_**2 for _ in array])**.5


def newt_grav(r1, r2, m2, G=0.01):
    r12 = r2 - r1
    return r12 * (G * m2 / (custom_norm(r12)**3))


def post_plot(planets, trace=False):
    plt.style.use('dark_background')
    fig, ax = plt.subplots()
    plt.grid(c='w', zorder=-1, alpha=.3)
    ax.set_aspect('equal')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])

    for ii in range(len(planets[0].r_hist)):
        collection = [ax.scatter(p.r_hist[ii][0],
                                      p.r_hist[ii][1],
                                      color=p.color,
                                      s=p.radius,
                                      zorder=10) for p in planets]

        ax.set_xlabel("Time: " + str(ii))

        plt.draw()
        plt.pause(.00001)
        [c.remove() for c in collection]

        if trace:
            [ax.scatter(p.r_hist[ii][0],
                             p.r_hist[ii][1],
                             color=p.color,
                             s=p.radius*.1,
                             alpha=.5,
                             zorder=5) for p in planets]



class Planet:

    def __init__(self, mass, r0, v0):
        self.mass = mass
        self.radius = (A*(self.mass/pi)**.5)**2
        self.r_hist = [r0]
        self.v_hist = [v0]

        self.color = np.random.rand(3,)

    def __repr__(self):
        return "\nPlanet\nmass: {}\nr: {}\nv: {}\n".format(self.mass, self.r_hist[-1], self.v_hist[-1])

    def move(self):
        self.r_hist.append(self.r_hist[-1] + (self.v_hist[-1]*dt))

    def update_v(self, other_planets):
        r_dot_dot = sum([newt_grav(self.r_hist[-1], p.r_hist[-1], p.mass) for p in other_planets])
        self.v_hist.append(self.v_hist[-1] + (r_dot_dot*dt))


class SolarSystem:

    def __init__(self, nbodies):
        self._n = nbodies
        self.planets = [Planet(np.random.triangular(left=.1, right=10, mode=.1),
                               np.random.uniform(low=-1, high=1, size=2),
                               np.random.uniform(low=-.1, high=.1, size=2)) for _ in range(nbodies)]

    def init_plot(self):
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots()
        plt.grid(c='w', zorder=-1, alpha=.3)
        self.ax.set_aspect('equal')
        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])

    def propogate(self, ii, plot):

        for idx, p in enumerate(self.planets):
            p.move()
            p.update_v([body for et_idx, body in enumerate(self.planets) if et_idx != idx])

        cm = sum([p.r_hist[-1] * p.mass for p in self.planets]) / sum([p.mass for p in self.planets])

        if plot:
            collection = [self.ax.scatter(p.r_hist[-1][0],
                                          p.r_hist[-1][1],
                                          color=p.color,
                                          s=p.radius,
                                          zorder=10) for p in self.planets]


            cm_col = self.ax.scatter(cm[0], cm[1], c='r', s=5, zorder=100)
            # [self.ax.scatter(p.r_hist[-1][0],
            #                  p.r_hist[-1][1],
            #                  color=p.color,
            #                  s=p.radius*.1,
            #                  alpha=.5,
            #                  zorder=5) for p in self.planets]

            self.ax.set_xlabel("Time: " + str(ii))

            plt.draw()
            plt.pause(.001)
            [c.remove() for c in collection]
            # cm_col.remove()

    def run(self, n, plot=True):
        start_time = time.time()
        if plot:
            self.init_plot()
        [self.propogate(ii, plot) for ii in range(n)]
        self.run_time = time.time() - start_time
        print("Simulation Time: ", self.run_time)


nbodies = int(sys.argv[1])
if len(sys.argv)>2:
    sim_time = int(sys.argv[2])
else:
    sim_time = 100
system = SolarSystem(nbodies)
system.run(sim_time, plot=False)
post_plot(system.planets)




