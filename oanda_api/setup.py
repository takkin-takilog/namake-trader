from setuptools import setup

package_name = "oanda_api"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="takkin",
    maintainer_email="takkin.takilog@gmail.com",
    description="The oanda_api package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pricing_exe = " + package_name + ".pricing:main",
            "pricing_stream_exe = " + package_name + ".pricing_stream:main",
            "order_requester_exe = " + package_name + ".order_requester:main",
            "fetcher_exe = " + package_name + ".fetcher:main",
        ],
    },
)
