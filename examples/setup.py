from setuptools import find_packages, setup


package_name = "gennav_ros2_examples"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="avurd",
    maintainer_email="ddruva445@gmail.com",
    description="Test package for gennav_ros2",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "runner = gennav_ros2_examples.runner:main",
        ],
    },
)
