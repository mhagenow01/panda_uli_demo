"""Functions for easily plotting
various types of data

 Last Updated: 09/08/2021
"""

__author__ = "Mike Hagenow"

import numpy as np
import plotly
import plotly.graph_objs as go
import copy


def threedplot(x,y,z):
    # plot x,y,z using plotly

    # # Configure Plotly to be rendered inline for a jupyter notebook.
    # plotly.offline.init_notebook_mode()

    # Original Data
    trace = go.Scatter3d(
        x=x,  # <-- Put your data instead
        y=y,  # <-- Put your data instead
        z=z,  # <-- Put your data instead
        mode='markers',
        marker={
            'size': 1.0,
            'opacity': 0.8,
            'color': 'blue'
        }
    )

    # Starting Point
    trace2 = go.Scatter3d(
        x=[x[0]],  # <-- Put your data instead
        y=[y[0]],  # <-- Put your data instead
        z=[z[0]],  # <-- Put your data instead
        mode='markers',
        marker={
            'size': 5.0,
            'opacity': 0.8,
            'color': 'blue'
        }
    )

    # Bounds configuring with invisible dots
    min_val = np.min(np.array([np.min(x),np.min(y),np.min(z)]))
    max_val = np.max(np.array([np.max(x),np.max(y),np.max(z)]))

    trace3 = go.Scatter3d(
        x=[min_val, max_val],  # <-- Put your data instead
        y=[min_val, max_val],  # <-- Put your data instead
        z=[min_val, max_val],  # <-- Put your data instead
        mode='markers',
        marker={
            'size': 0.0,
            'opacity': 0.0,
            'color': 'blue'
        }
    )

    # Configure the layout.
    layout = go.Layout(
        margin={'l': 0, 'r': 0, 'b': 0, 't': 0}
    )

    data = [trace,trace2,trace3]

    plot_figure = go.Figure(data=data, layout=layout)

    # Render the plot.
    plotly.offline.iplot(plot_figure)


def highlightedthreedplot(x,y,z,xs_gray,ys_gray,zs_gray):
    # plot x,y,z using plotly
    # one set in red, others in gray

    # # Configure Plotly to be rendered inline for a jupyter notebook.
    # plotly.offline.init_notebook_mode()

    # Original Data
    trace = go.Scatter3d(
        x=x,  # <-- Put your data instead
        y=y,  # <-- Put your data instead
        z=z,  # <-- Put your data instead
        mode='markers',
        marker={
            'size': 1.0,
            'opacity': 0.8,
            'color': 'red'
        }
    )

    # Starting Point
    trace2 = go.Scatter3d(
        x=[x[0]],  # <-- Put your data instead
        y=[y[0]],  # <-- Put your data instead
        z=[z[0]],  # <-- Put your data instead
        mode='markers',
        marker={
            'size': 5.0,
            'opacity': 0.8,
            'color': 'red'
        }
    )

    # Bounds configuring with invisible dots
    min_val = np.min(np.array([np.min(x),np.min(y),np.min(z)]))
    max_val = np.max(np.array([np.max(x),np.max(y),np.max(z)]))

    trace3 = go.Scatter3d(
        x=[min_val, max_val],  # <-- Put your data instead
        y=[min_val, max_val],  # <-- Put your data instead
        z=[min_val, max_val],  # <-- Put your data instead
        mode='markers',
        marker={
            'size': 0.0,
            'opacity': 0.0,
            'color': 'blue'
        }
    )

    data = [trace, trace2, trace3]

    for ii in range(0,len(xs_gray)):
        trace_temp = go.Scatter3d(
            x=xs_gray[ii],  # <-- Put your data instead
            y=ys_gray[ii],  # <-- Put your data instead
            z=zs_gray[ii],  # <-- Put your data instead
            mode='markers',
            marker={
                'size': 1.0,
                'opacity': 0.8,
                'color': 'gray'
            }
        )

        data.append(copy.deepcopy(trace_temp))

    # Configure the layout.
    layout = go.Layout(
        margin={'l': 0, 'r': 0, 'b': 0, 't': 0}
    )

    plot_figure = go.Figure(data=data, layout=layout)

    # Render the plot.
    plotly.offline.plot(plot_figure)