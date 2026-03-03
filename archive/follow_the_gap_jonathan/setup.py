from setuptools import setup

package_name = 'follow_the_gap_jonathan'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johnny',
    maintainer_email='jonathan98mohr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_the_gap_jonathan = follow_the_gap_jonathan.follow_the_gap_jonathan:main',
            'follow_the_gap_jonathan_final = follow_the_gap_jonathan.follow_the_gap_jonathan_final:main',
            'follow_the_gap_jonathan2 = follow_the_gap_jonathan.follow_the_gap_jonathan2:main',
        ],
    },
)
