from setuptools import find_packages, setup

package_name = 'fruits'

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
    maintainer='Marcel Radke',
    maintainer_email='marcel.radke@posteo.de',
    description='detects fruits',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             'webcam = fruits.webcam:main',
             'display = fruits.display:main',
             'marker = fruits.marker:main',
             'apple = fruits.apple:main'
        ],
    },
)
