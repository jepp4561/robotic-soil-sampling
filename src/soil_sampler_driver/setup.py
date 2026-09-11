from setuptools import find_packages, setup

package_name = "soil_sampler_driver"

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
            f"share/{package_name}/config",
            ["config/drivers.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jeppe Hansen",
    maintainer_email="jeppha@mmmi.sdu.dk",
    description=(
        "Hardware drivers for the robotic soil sampling implement."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "teros12_node = soil_sampler_driver.teros12_node:main",
        ],
    },
)