from setuptools import find_packages, setup
import glob
import os

package_name = 'ur_script_sender_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'paths'), glob.glob('paths/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dxr',
    maintainer_email='jhjdxr@dxr-ai.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'send_script = ur_script_sender_py.send_script:main',
            'send_traj_action = ur_script_sender_py.send_traj_action:main',
        ],
    },
)
