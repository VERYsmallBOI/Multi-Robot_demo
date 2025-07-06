from setuptools import find_packages, setup

package_name = 'zigzag_movement'

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
    maintainer='Tamil_selvan',
    maintainer_email='tamilselvanelango3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zigzag_driver=zigzag_movement.zigzag_driver:main'
        ],
    },
)
