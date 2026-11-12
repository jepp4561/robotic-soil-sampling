import logging
import math
import threading

import dash
from dash import ctx, dcc, html, no_update
from dash.dependencies import Input, Output, State
import plotly.graph_objects as go
from plotly.subplots import make_subplots

import rclpy
from rclpy.node import Node

from soil_sampler_interfaces.msg import ExperimentSample, ForceProfileSample, SkippedLocation

MAXIMUM_STORED_PROFILES = 20
COLUMNS = 2
PANEL_HEIGHT = 380

MODALITIES = [
    ("vwc", "VWC", lambda sample: sample.teros12.volumetric_water_content_mean),
    ("ec", "EC", lambda sample: sample.teros12.electrical_conductivity_mean),
    ("soil_temperature", "Soil temperature", lambda sample: sample.teros12.temperature_mean),
    ("humidity", "Humidity", lambda sample: sample.sen0658.humidity_mean),
    ("air_temperature", "Air temperature", lambda sample: sample.sen0658.temperature_mean),
    ("pressure", "Pressure", lambda sample: sample.sen0658.pressure_mean),
    ("wind_speed", "Wind speed", lambda sample: sample.sen0658.wind_speed_mean),
    ("noise", "Noise", lambda sample: sample.sen0658.noise_mean),
    ("pm2_5", "PM2.5", lambda sample: sample.sen0658.pm2_5_mean),
    ("pm10", "PM10", lambda sample: sample.sen0658.pm10_mean),
    ("illuminance", "Illuminance", lambda sample: sample.sen0658.illuminance),
    ("rainfall", "Rainfall", lambda sample: sample.sen0658.rainfall),
]

DEFAULT_MODALITIES = ["vwc", "ec", "soil_temperature", "humidity"]


class ExperimentVisualization(Node):

    def __init__(self) -> None:
        super().__init__("soil_sampler_experiment_visualization")

        self.declare_parameter("force_profile_topic", "/soil_sampler/force_profile/sample")
        force_profile_topic = str(self.get_parameter("force_profile_topic").value)

        self.samples = []
        self.skipped = []
        self.samples_version = 0
        self.profiles_version = 0
        self.profiles: dict[str, list[tuple[float, float]]] = {}
        self.lock = threading.Lock()

        self.subscription = self.create_subscription(ExperimentSample, "sample", self.sample_callback, 10)
        self.skipped_subscription = self.create_subscription(SkippedLocation, "skipped_location", self.skipped_callback, 10)
        self.force_profile_subscription = self.create_subscription(ForceProfileSample, force_profile_topic, self.force_profile_callback, 100)

    def sample_callback(self, message: ExperimentSample) -> None:
        with self.lock:
            self.samples.append(message)
            self.samples_version += 1

    def skipped_callback(self, message: SkippedLocation) -> None:
        with self.lock:
            self.skipped.append(message)
            self.samples_version += 1

    def force_profile_callback(self, message: ForceProfileSample) -> None:
        with self.lock:
            self.profiles.setdefault(message.run_id, []).append((float(message.depth), float(message.force)))

            while len(self.profiles) > MAXIMUM_STORED_PROFILES:
                del self.profiles[next(iter(self.profiles))]

            self.profiles_version += 1

    def get_version(self, view: str) -> int:
        with self.lock:
            return self.profiles_version if view == "force-profile" else self.samples_version

    def get_samples(self):
        with self.lock:
            return list(self.samples)

    def get_skipped(self):
        with self.lock:
            return list(self.skipped)

    def get_profiles(self) -> dict[str, list[tuple[float, float]]]:
        with self.lock:
            return {run_id: list(points) for run_id, points in self.profiles.items()}


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

    return ([x_center - x_extent / 2, x_center + x_extent / 2], [y_center - y_extent / 2, y_center + y_extent / 2,])


def create_figure(samples, skipped, selected):
    modalities = [modality for modality in MODALITIES if modality[0] in (selected or [])]

    if not modalities:
        modalities = [modality for modality in MODALITIES if modality[0] in DEFAULT_MODALITIES]

    count = len(modalities)
    columns = 1 if count == 1 else COLUMNS
    rows = math.ceil(count / columns)

    specs = [[{} if row * columns + column < count else None for column in range(columns)] for row in range(rows)]

    figure = make_subplots(
        rows=rows,
        cols=columns,
        specs=specs,
        subplot_titles=[label for _, label, _ in modalities],
        horizontal_spacing=0.2,
        vertical_spacing=0.14 if rows == 1 else min(0.14, 0.9 / (rows - 1)),
    )

    x_range, y_range = calculate_axis_ranges(list(samples) + list(skipped))

    x = [sample.x for sample in samples]
    y = [sample.y for sample in samples]
    labels = [str(index + 1) for index in range(len(samples))]
    depths = [sample.sample.depth for sample in samples]

    for index, (key, label, getter) in enumerate(modalities):
        row = index // columns + 1
        column = index % columns + 1

        values = [getter(sample.sample) for sample in samples]

        subplot = figure.get_subplot(row, column)
        x_domain = subplot.xaxis.domain
        y_domain = subplot.yaxis.domain

        figure.add_trace(
            go.Scatter(
                x=x,
                y=y,
                mode="markers+text",
                text=labels,
                textposition="top right",
                customdata=[[depth] for depth in depths],
                marker=dict(
                    size=12,
                    color=values,
                    colorscale="Viridis",
                    showscale=bool(values),
                    colorbar=dict(
                        title=label,
                        x=x_domain[1] + 0.01,
                        y=(y_domain[0] + y_domain[1]) / 2,
                        xanchor="left",
                        yanchor="middle",
                        len=(y_domain[1] - y_domain[0]) * 0.9,
                        thickness=14,
                    ),
                ),
                hovertemplate=(
                    "Sample %{text}<br>"
                    "X: %{x:.3f} m<br>"
                    "Y: %{y:.3f} m<br>"
                    "Depth: %{customdata[0]:.3f} m<br>"
                    f"{label}: %{{marker.color:.3f}}"
                    "<extra></extra>"
                ),
                showlegend=False,
            ),
            row=row,
            col=column,
        )

        if skipped:
            figure.add_trace(
                go.Scatter(
                    x=[location.x for location in skipped],
                    y=[location.y for location in skipped],
                    mode="markers",
                    marker=dict(symbol="x", size=12, color="black", line=dict(width=2)),
                    customdata=[[location.peak_force, location.max_rate] for location in skipped],
                    hovertemplate=(
                        "Rock<br>"
                        "X: %{x:.3f} m<br>"
                        "Y: %{y:.3f} m<br>"
                        "Peak force: %{customdata[0]:.1f} N<br>"
                        "Max rate: %{customdata[1]:.0f} N/m"
                        "<extra></extra>"
                    ),
                    showlegend=False,
                ),
                row=row,
                col=column,
            )

        x_axis_id = "x" if index == 0 else f"x{index + 1}"

        figure.update_xaxes(title_text="X [m]", range=x_range, showgrid=True, zeroline=False, row=row, col=column)
        figure.update_yaxes(title_text="Y [m]", range=y_range, showgrid=True, zeroline=False, scaleanchor=x_axis_id, scaleratio=1, row=row, col=column)

    summary = f"{len(samples)} samples"

    if skipped:
        summary += f", {len(skipped)} skipped (black x = rock)"

    figure.update_layout(
        title=dict(text=summary, x=0.0, xanchor="left", font=dict(size=14)),
        height=PANEL_HEIGHT * rows + 140,
        margin=dict(l=70, r=120, t=110, b=60),
        template="plotly_white",
        uirevision="soil-samples-" + ",".join(key for key, _, _ in modalities),
    )

    return figure


def create_force_profile_figure(profiles):
    figure = go.Figure()

    latest_run_id = next(reversed(profiles)) if profiles else None

    for run_id, points in profiles.items():
        is_latest = run_id == latest_run_id

        figure.add_trace(
            go.Scatter(
                x=[point[0] for point in points],
                y=[point[1] for point in points],
                mode="lines+markers",
                name=run_id,
                line=dict(width=3 if is_latest else 1.5),
                marker=dict(size=6 if is_latest else 4),
                opacity=1.0 if is_latest else 0.55,
                hovertemplate=(
                    f"{run_id}<br>"
                    "Depth: %{x:.4f} m<br>"
                    "Force: %{y:.2f} N"
                    "<extra></extra>"
                ),
            )
        )

    figure.update_xaxes(
        title_text="Depth [m]",
        showgrid=True,
        zeroline=False,
    )

    figure.update_yaxes(
        title_text="Force [N]",
        showgrid=True,
        zeroline=False,
    )

    figure.update_layout(
        height=700,
        margin=dict(
            l=70,
            r=40,
            t=60,
            b=60,
        ),
        template="plotly_white",
        title="Force profile",
        legend=dict(title="Run"),
        uirevision="force-profile",
    )

    return figure


def create_dash_app(node):

    app = dash.Dash(__name__)

    app.layout = html.Div(
        [
            html.H1("Soil Sampling Experiment"),
            dcc.Tabs(
                id="view-tabs",
                value="soil-samples",
                children=[
                    dcc.Tab(label="Soil samples", value="soil-samples"),
                    dcc.Tab(label="Force profile", value="force-profile"),
                ],
            ),
            dcc.Dropdown(
                id="modality-select",
                options=[{"label": label, "value": key} for key, label, _ in MODALITIES],
                value=DEFAULT_MODALITIES,
                multi=True,
                clearable=False,
                style={"marginTop": "10px"},
            ),
            dcc.Graph(
                id="experiment-plot",
                style={"width": "100%"},
                config={
                    "displaylogo": False,
                    "scrollZoom": True,
                },
            ),
            dcc.Store(id="render-key"),
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
        Output("render-key", "data"),
        Input("update-interval", "n_intervals"),
        Input("view-tabs", "value"),
        Input("modality-select", "value"),
        State("render-key", "data"),
    )
    def update_plot(_, view, modalities, rendered_key):
        key = [view, node.get_version(view)]

        if ctx.triggered_id == "update-interval" and key == rendered_key:
            return no_update, no_update

        if view == "force-profile":
            return create_force_profile_figure(node.get_profiles()), key

        return create_figure(node.get_samples(), node.get_skipped(), modalities), key

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