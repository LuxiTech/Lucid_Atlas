from glob import glob

from setuptools import setup

package_name = "luxi_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/localization.yaml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@example.com",
    description="Open3D based global localization for Luxi Atlas FAST-LIO maps.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grid_map_publisher = luxi_localization.grid_map_publisher:main",
            "localization_node = luxi_localization.localization_node:main",
            "set_initial_pose = luxi_localization.set_initial_pose:main",
        ],
    },
)
