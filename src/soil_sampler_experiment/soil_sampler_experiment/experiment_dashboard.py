import logging
import threading

import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
from plotly.subplots import make_subplots

import rclpy
from rclpy.node import Node

from soil_sampler_interfaces.msg import ExperimentSample


class ExperimentVisualization(Node):

    def __init__(self) -> None:
        super().__init__("soil_sampler_experiment_visualization")

        self.samples = []
        self.lock = threading.Lock()

        self.subscription = self.create_subscription(
            ExperimentSample,
            "sample",
            self.sample_callback,
            10,
        )

    def sample_callback(self, message: ExperimentSample) -> None:
        with self.lock:
            self.samples.append(message)

    def get_samples(self):
        with self.lock:
            return list(self.samples)


def calculate_axis_ranges(samples):
    if not samples:
        return [-0.05, 0.05], [-0.05, 0.05]

    x_values = [sample.x for sample in samples]
    y_values = [sample.y for sample in samples]

    x_min = min(x_values)
    x_max = max(x_values)
    y_min = min(y_values)
    y_max = max(y_values)

    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2

    x_extent = x_max - x_min
    y_extent = y_max - y_min

    minimum_extent = 0.05

    x_extent = max(x_extent, minimum_extent)
    y_extent = max(y_extent, minimum_extent)

    padding = 0.25

    x_extent *= 1.0 + padding
    y_extent *= 1.0 + padding

    return (
        [
            x_center - x_extent / 2,
            x_center + x_extent / 2,
        ],
        [
            y_center - y_extent / 2,
            y_center + y_extent / 2,
        ],
    )


def create_figure(samples):
    figure = make_subplots(
        rows=2,
        cols=2,
        subplot_titles=(
            "VWC",
            "EC",
            "Soil temperature",
            "Humidity",
        ),
        horizontal_spacing=0.18,
        vertical_spacing=0.16,
    )

    x_range, y_range = calculate_axis_ranges(samples)

    positions = [(sample.x, sample.y) for sample in samples]

    data = [
        (
            [
                sample.sample.teros12.volumetric_water_content_mean
                for sample in samples
            ],
            "VWC",
            1,
            1,
        ),
        (
            [
                sample.sample.teros12.electrical_conductivity_mean
                for sample in samples
            ],
            "EC",
            1,
            2,
        ),
        (
            [
                sample.sample.teros12.temperature_mean
                for sample in samples
            ],
            "Soil temperature",
            2,
            1,
        ),
        (
            [
                sample.sample.sen0658.humidity_mean
                for sample in samples
            ],
            "Humidity",
            2,
            2,
        ),
    ]

    colorbar_positions = {
        (1, 1): dict(x=0.42, y=0.77),
        (1, 2): dict(x=1.01, y=0.77),
        (2, 1): dict(x=0.42, y=0.23),
        (2, 2): dict(x=1.01, y=0.23),
    }

    for values, title, row, col in data:

        x = [position[0] for position in positions]
        y = [position[1] for position in positions]

        colorbar = colorbar_positions[(row, col)]

        figure.add_trace(
            go.Scatter(
                x=x,
                y=y,
                mode="markers+text",
                text=[
                    str(index + 1)
                    for index in range(len(samples))
                ],
                textposition="top right",
                marker=dict(
                    size=12,
                    color=values if values else [0],
                    colorscale="Viridis",
                    showscale=len(values) > 1,
                    colorbar=dict(
                        title=title,
                        x=colorbar["x"],
                        y=colorbar["y"],
                        xanchor="left",
                        yanchor="middle",
                        len=0.30,
                        thickness=15,
                    ),
                ),
                hovertemplate=(
                    "Sample %{text}<br>"
                    "X: %{x:.3f} m<br>"
                    "Y: %{y:.3f} m<br>"
                    f"{title}: %{{marker.color:.3f}}"
                    "<extra></extra>"
                ),
                showlegend=False,
            ),
            row=row,
            col=col,
        )

    figure.update_xaxes(
        title_text="X [m]",
        range=x_range,
        showgrid=True,
        zeroline=False,
        constrain="domain",
    )

    figure.update_yaxes(
        title_text="Y [m]",
        range=y_range,
        showgrid=True,
        zeroline=False,
        constrain="domain",
    )

    figure.update_layout(
        height=800,
        margin=dict(
            l=70,
            r=120,
            t=80,
            b=60,
        ),
        template="plotly_white",
    )

    figure.update_layout(
        xaxis=dict(
            scaleanchor="y",
            scaleratio=1,
        ),
        xaxis2=dict(
            scaleanchor="y2",
            scaleratio=1,
        ),
        xaxis3=dict(
            scaleanchor="y3",
            scaleratio=1,
        ),
        xaxis4=dict(
            scaleanchor="y4",
            scaleratio=1,
        ),
    )

    return figure


def create_dash_app(node):

    app = dash.Dash(__name__)

    app.layout = html.Div(
        [
            html.H1("Soil Sampling Experiment"),
            dcc.Graph(
                id="experiment-plot",
                style={
                    "height": "85vh",
                    "width": "100%",
                },
                config={
                    "displaylogo": False,
                    "scrollZoom": True,
                },
            ),
            dcc.Interval(
                id="update-interval",
                interval=500,
                n_intervals=0,
            ),
        ],
        style={
            "width": "95%",
            "margin": "auto",
        },
    )

    @app.callback(
        Output("experiment-plot", "figure"),
        Input("update-interval", "n_intervals"),
    )
    def update_plot(_):
        samples = node.get_samples()
        return create_figure(samples)

    return app


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ExperimentVisualization()

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    ros_thread.start()

    app = create_dash_app(node)

    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    try:
        app.run(
            host="0.0.0.0",
            port=8050,
            debug=False,
        )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()