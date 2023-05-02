from setuptools import setup  # type: ignore[import]

package_name = "trade_app"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="takkin",
    maintainer_email="takkin.takilog@gmail.com",
    description="The trade_app package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["ex_sma = trade_app.app_sma:main"],
    },
)
