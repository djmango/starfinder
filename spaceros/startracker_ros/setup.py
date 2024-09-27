from setuptools import find_packages, setup

package_name = "startracker_ros"

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
    maintainer="spaceros-user",
    maintainer_email="spaceros-user@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "my_node = startracker_ros.my_node:main",
            "starfinder_publisher = startracker_ros.starfinder_publisher_node:main",
            "starfinder_subscriber = startracker_ros.starfinder_subscriber_node:main",
        ],
    },
)
