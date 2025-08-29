from setuptools import find_packages, setup

package_name = 'pacote8_publisher_voz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'soundfile', 'sounddevice', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu.email@example.com',
    description='Nó ROS 2 para integração com servidor VITS de síntese de fala.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_synthesis_publisher = pacote8_publisher_voz.speech_synthesis_publisher_node:main',
        ],
    },
)
