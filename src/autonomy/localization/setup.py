from setuptools import find_packages, setup

package_name = "localization"

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
    description="Sensor-based vehicle state estimation (scaffold)",
    license="TODO",
    entry_points={"console_scripts": []},
)
