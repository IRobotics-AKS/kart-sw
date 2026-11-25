from setuptools import find_packages, setup

package_name = "sim_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.12",
    zip_safe=False,
    maintainer="Kart maintainers",
    maintainer_email="maintainers@example.invalid",
    description="ROS adapter for MuJoCo with simulation-only joint efforts",
    license="TODO",
    entry_points={"console_scripts": [
        "sim_bridge = sim_ros.bridge_node:main",
        "send_efforts = sim_ros.send_efforts:main",
    ]},
)
