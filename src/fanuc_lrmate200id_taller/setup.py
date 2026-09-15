from setuptools import find_packages, setup

package_name = 'fanuc_lrmate200id_taller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan David Botero',
    maintainer_email='juan.botero@eia.edu.co',
    description='Taller de cinematica y planeacion de movimiento - FANUC LR Mate 200iD',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros2_scene    = fanuc_lrmate200id_taller.scene:main',
            'ros2_mover    = fanuc_lrmate200id_taller.mover:main',
            'ros2_ik       = fanuc_lrmate200id_taller.ik4:main',
            'ros2_planners = fanuc_lrmate200id_taller.cmp2:main',
            'ros2_perfiles = fanuc_lrmate200id_taller.perfiles:main',
            'ros2_cart4b   = fanuc_lrmate200id_taller.cart2:main',
            'ros2_cart4d   = fanuc_lrmate200id_taller.cart4d:main',
            'ros2_jacobian = fanuc_lrmate200id_taller.jacobiano:main',
            'ros2_ciclo    = fanuc_lrmate200id_taller.ciclo:main',
        ],
    },
)
