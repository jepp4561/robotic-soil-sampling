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
    )

    if not samples:
        figure.update_layout(
            height=800,
            margin=dict(l=60, r=60, t=80, b=60),
        )
        return figure

    positions = [(sample.x, sample.y) for sample in samples]

    data = [
        (
            [sample.sample.teros12.volumetric_water_content_mean for sample in samples],
            "VWC",
            "VWC",
            1,
            1,
        ),
        (
            [sample.sample.teros12.electrical_conductivity_mean for sample in samples],
            "EC",
            "EC",
            1,
            2,
        ),
        (
            [sample.sample.teros12.temperature_mean for sample in samples],
            "Soil temperature",
            "Soil temperature",
            2,
            1,
        ),
        (
            [sample.sample.sen0658.humidity_mean for sample in samples],
            "Humidity",
            "Humidity",
            2,
            2,
        ),
    ]

    for values, title, colorbar_title, row, col in data:
        x = [position[0] for position in positions]
        y = [position[1] for position in positions]

        figure.add_trace(
            go.Scatter(
                x=x,
                y=y,
                mode="markers+text",
                text=[str(index + 1) for index in range(len(samples))],
                textposition="top right",
                marker=dict(
                    size=12,
                    color=values,
                    colorscale="Viridis",
                    showscale=len(values) > 1,
                    colorbar=dict(
                        title=colorbar_title,
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
        showgrid=True,
        zeroline=False,
        scaleanchor="y",
        scaleratio=1,
    )

    figure.update_yaxes(
        title_text="Y [m]",
        showgrid=True,
        zeroline=False,
    )

    figure.update_layout(
        height=800,
        margin=dict(l=60, r=60, t=80, b=60),
        template="plotly_white",
    )

    return figure


def create_dash_app(node):

    app = dash.Dash(__name__)

    app.layout = html.Div(
        [
            html.H1("Soil Sampling Experiment"),
            dcc.Graph(
                id="experiment-plot",
                style={"height": "85vh"},
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