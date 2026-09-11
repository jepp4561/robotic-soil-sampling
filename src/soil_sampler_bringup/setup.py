from glob import glob
from setuptools import setup

package_name = "soil_sampler_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ["package.xml"],
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml'),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jeppe Hansen",
    maintainer_email="jeppha@mmmi.sdu.dk",
    description="Launch and configuration files for the robotic soil sampling implement.",
    license="MIT",
)
