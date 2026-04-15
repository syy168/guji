from setuptools import setup

package_name = 'woosh_nav_feedback'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/exec_task_feedback.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='feiguang',
    maintainer_email='feiguang@local',
    description='Woosh navigation feedback demo for ROS2 Foxy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exec_task_feedback_node = woosh_nav_feedback.exec_task_feedback_node:main',
        ],
    },
)
