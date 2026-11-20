from setuptools import setup

package_name = 'robot_fjw_control'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='equipo_fjw',
    maintainer_email='todo@example.com',
    description='Controlador F J W para robot Ackermann de Fusion 360',
    license='TODO',
    entry_points={
        'console_scripts': ['ejecutar_fjw = robot_fjw_control.ejecutar_fjw:main', 'probar_movimiento = robot_fjw_control.probar_movimiento:main'],
    },
)
