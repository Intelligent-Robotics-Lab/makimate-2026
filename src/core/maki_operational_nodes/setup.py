from setuptools import setup
from glob import glob
import os

package_name = 'maki_operational_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='makimate',
    maintainer_email='you@example.com',
    description='Operational mode manager and launch orchestration for Maki.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maki_operational_modes = maki_operational_nodes.maki_operational_modes:main',
            'maki_launch_manager = maki_operational_nodes.maki_launch_manager:main',

            # NEW: expression behavior bridge for presentation mode
            'maki_awake_behavior = maki_operational_nodes.maki_awake_behavior:main',

            'voice_calibration = makimate_asr.voice_calibration_node:main',
            'speaker_recognition = makimate_asr.speaker_recognition_node:main',
            'calibration_workflow = makimate_asr.calibration_workflow_node:main',
            'realtime_display_logger = makimate_asr.realtime_display_logger:main',
        ],
    },
)
