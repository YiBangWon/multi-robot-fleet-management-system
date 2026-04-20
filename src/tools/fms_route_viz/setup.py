"""Package metadata for the route-table visualization tool."""

from setuptools import setup


package_name = "fms_route_viz"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Visualize FMS route tables as layered dependency graphs.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "route_viz_node = fms_route_viz.route_viz_node:main",
        ],
    },
)
