from setuptools import find_packages, setup

package_name = 'oakd_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='chris@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_oakd = oakd_vision.hello_oakd:main',
            'image_viewer = oakd_vision.image_viewer:main', 
            'nn_overlay = oakd_vision.nn_overlay:main',
            'depth_point_demo = oakd_vision.depth_point_demo:main',
            'yolo11_dummyHuman = oakd_vision.yolo11_dummyHuman_node:main',  #------- new


        ],
    },
)
