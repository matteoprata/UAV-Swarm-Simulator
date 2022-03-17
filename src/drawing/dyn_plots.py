from matplotlib import pyplot as plt
import numpy as np 
import pandas as pd

""" An Utility class to plot the rewards in the Q-Table 
    of RL Routing Algorithm 
"""
class QTablePlotter():

    def __init__(self, states : list, actions : list, drone_id : int):
        """ init the plot with states and actions name """
        self.nstate = len(states)
        self.drone_id = drone_id
        self.actions = len(actions)
        self.states = states
        self.actions = actions

    def update(self, Q, time_step=0.0000001):
        """ update the Q-Table plot """
        out = []
        for state in self.states:
            out.append(Q[state])
        out = np.asarray(out)

        plt.clf()
        plt.imshow(out, cmap=plt.cm.gist_earth_r)

        # We want to show all ticks...
        plt.xticks(np.arange(len(self.actions)), self.actions)
        plt.yticks(np.arange(len(self.states)),[str(el[0].value) + "-" + str(el[1].value) + "-" + str(el[2].value)
                                                    for el in self.states])

        plt.title("[den, mob, err] : action ->" + str(self.drone_id))
        plt.pause(time_step)



""" An Utility class to plot the Moving Average of performance
    --- this helps to shows convergences of RL Routing Algorithm 
"""
class MovingAveragePlot():

    def __init__(self, metric_name : str, window=20):
        """ init the plot with states and actions name """
        self.metric_name = metric_name
        self.window = window
        self.values = []

    def add_value(self, value):
        self.values.append(value)

    def plot(self, time_step=1, values=None):
        """ update the Q-Table plot """
        if values is None:
            values = self.values

        if len(values) < 20:
            return
        df = pd.Series(values)
        scatter_print = values[self.window:]
        to_plot_data = df.rolling(self.window).mean()[self.window:]

        plt.clf()
        plt.plot(range(len(scatter_print)), to_plot_data, color="red", label="MA-" + str(self.window))
        plt.scatter(range(len(scatter_print)), scatter_print, color="blue", s=1, label="values")

        plt.legend()
        plt.title(self.metric_name)
        plt.pause(time_step)
