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
            'object_recognition = chat_rover.object_recognition:main',
            'voice2text = chat_rover.voice2text:main',
            'gpt_process = chat_rover.gpt_process:main'
        ],
    },
)
