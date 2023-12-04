from setuptools import find_packages, setup

package_name = 'chat_rover_desktop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name),
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
            'llava_node = chat_rover_desktop.llava_node:main'
        ],
    },
)