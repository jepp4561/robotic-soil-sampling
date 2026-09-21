from setuptools import find_packages, setup

package_name = "soil_sampler_experiment"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
        (
            f"share/{package_name}/launch",
            ["launch/experiment_sim.launch.py"],
        ),
        (
            f"share/{package_name}/config",
            ["config/experiment.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jeppe Hansen",
    maintainer_email="jeppha@mmmi.sdu.dk",
    description="Experiment orchestration and visualization for robotic soil sampling.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "experiment_node = soil_sampler_experiment.experiment_node:main",
            "experiment_visualization = soil_sampler_experiment.experiment_visualization:main",
            "experiment_dashboard = soil_sampler_experiment.experiment_dashboard:main",
        ],
    },
)