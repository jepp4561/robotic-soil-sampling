from setuptools import find_packages, setup

package_name = "soil_sampler_control"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jeppe Hansen",
    maintainer_email="jeppha@mmmi.sdu.dk",
    description="Control logic for the robotic soil sampling implement.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "soil_sampler_node = soil_sampler_control.sampler_node:main",
            "mock_robot_server = soil_sampler_control.mock_robot_server:main",
            "hardware_simulator = soil_sampler_control.hardware_simulator:main",
        ],
    },
)