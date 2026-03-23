from setuptools import setup

package_name = 'makimate_dashboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_data={package_name: ['dashboard.html']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James',
    maintainer_email='you@example.com',
    description='Web dashboard for live log viewing and parameter editing.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dashboard = makimate_dashboard.dashboard_node:main',
        ],
    },
)
