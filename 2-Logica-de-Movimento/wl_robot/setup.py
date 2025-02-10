from setuptools import setup
import os
from glob import glob

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=['my_robot'],
    package_dir={'': 'src'},
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danilo',
    maintainer_email='danilo@todo.todo',
    description='Descrição do seu pacote',
    license='Licença',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_controller = my_robot.simple_controller:main',
        ],
    },
)

