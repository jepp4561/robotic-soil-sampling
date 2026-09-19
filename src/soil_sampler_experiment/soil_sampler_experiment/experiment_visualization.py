import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from soil_sampler_interfaces.msg import ExperimentSample


class ExperimentVisualization(Node):

    def __init__(self) -> None:
        super().__init__("soil_sampler_experiment_visualization")

        self.samples = []
        self.colorbars = [None, None, None, None]

        self.subscription = self.create_subscription(
            ExperimentSample,
            "sample",
            self.sample_callback,
            10,
        )

        self.timer = self.create_timer(0.5, self.update_plot)

        plt.ion()

        self.figure, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.figure.canvas.manager.set_window_title("Soil Sampling Experiment")

    def sample_callback(self, message: ExperimentSample) -> None:
        self.samples.append(message)

    def update_plot(self) -> None:
        if not self.samples:
            return

        positions = [(sample.x, sample.y) for sample in self.samples]

        vwc = [sample.sample.teros12.volumetric_water_content_mean for sample in self.samples]
        ec = [sample.sample.teros12.electrical_conductivity_mean for sample in self.samples]
        soil_temperature = [sample.sample.teros12.temperature_mean for sample in self.samples]
        humidity = [sample.sample.sen0658.humidity_mean for sample in self.samples]

        self.axes[0, 0].clear()
        self.axes[0, 1].clear()
        self.axes[1, 0].clear()
        self.axes[1, 1].clear()

        self.plot_spatial_data(self.axes[0, 0], positions, vwc, "VWC", 0)
        self.plot_spatial_data(self.axes[0, 1], positions, ec, "EC", 1)
        self.plot_spatial_data(self.axes[1, 0], positions, soil_temperature, "Soil temperature", 2)
        self.plot_spatial_data(self.axes[1, 1], positions, humidity, "Humidity", 3)

        self.figure.tight_layout()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def plot_spatial_data(self, axis, positions, values, title: str, colorbar_index: int) -> None:
        x = [position[0] for position in positions]
        y = [position[1] for position in positions]

        scatter = axis.scatter(x, y, c=values, s=100)

        for index, (sample_x, sample_y) in enumerate(positions):
            axis.annotate(
                str(index + 1),
                (sample_x, sample_y),
                xytext=(5, 5),
                textcoords="offset points",
            )

        axis.set_title(title)
        axis.set_xlabel("X [m]")
        axis.set_ylabel("Y [m]")
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(True)

        if self.colorbars[colorbar_index] is not None:
            self.colorbars[colorbar_index].remove()
            self.colorbars[colorbar_index] = None

        if len(values) > 1:
            self.colorbars[colorbar_index] = self.figure.colorbar(scatter, ax=axis)

    def destroy_node(self) -> bool:
        plt.close(self.figure)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ExperimentVisualization()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
