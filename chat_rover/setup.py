from setuptools import setup
from glob import glob

package_name = 'chat_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alaleh',
    maintainer_email='alaleh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpt1_node = chat_rover.gpt1_node:main',
            'gpt2_node = chat_rover.gpt2_node:main',
            'ar_node = chat_rover.ar_node:main',
            'vosk_node = chat_rover.vosk_node:main',
            'yolo_node = chat_rover.yolo_node:main',
        ],
    },
)
