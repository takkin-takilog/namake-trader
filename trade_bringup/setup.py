from setuptools import setup
from glob import glob

package_name = "trade_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/params", glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="takkin",
    maintainer_email="takkin.takilog@gmail.com",
    description="The trade_bringup package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
