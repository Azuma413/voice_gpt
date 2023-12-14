from setuptools import find_packages, setup

package_name = 'desktop_pkg'

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
    maintainer='main-pc',
    maintainer_email='hirekatsu0523@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample = desktop_pkg.sample:main',
            'llava_node = desktop_pkg.llava_node:main',
        ],
    },
)
