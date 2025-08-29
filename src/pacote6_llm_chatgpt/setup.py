import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pacote6_llm_chatgpt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saulo',
    maintainer_email='saulo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_server = pacote6_llm_chatgpt.llm_server_node:main',
            'assistente_chatgpt = pacote6_llm_chatgpt.assistente_node:main',
        ],
    },
)
