from setuptools import setup

package_name = "trade_manager"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="takkin",
    maintainer_email="takkin.takilog@gmail.com",
    description="The trade_manager package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "order_scheduler = " + package_name + ".order_scheduler:main",
            "candles_store = " + package_name + ".candles_store:main",
        ],
    },
)
