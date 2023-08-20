from setuptools import setup  # type: ignore[import]

package_name = "api_server_oanda"

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
    description="The api_server_oanda package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pricing_publisher = " + package_name + ".pricing_publisher:main",
            "api_server = " + package_name + ".api_server:main",
        ],
    },
)
