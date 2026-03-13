from setuptools import setup

setup(
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drqp_gazebo']),
        ('share/drqp_gazebo', ['package.xml']),
        ('share/drqp_gazebo/launch', ['launch/sim.launch.py']),
        ('share/drqp_gazebo/config', ['config/drqp_gazebo_bridge.yml']),
    ],
)
