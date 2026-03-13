from setuptools import setup

setup(
    tests_require=['pytest'], # Ensure pytest is declared so colcon does not skip the test/ suite.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drqp_brain']),
        ('share/drqp_brain', ['package.xml']),
        ('share/drqp_brain/launch', ['launch/bringup.launch.py']),
    ],
)
