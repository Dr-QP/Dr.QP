from setuptools import setup

# Ensure pytest is declared so colcon does not skip the test/ suite.
setup(tests_require=["pytest"])
