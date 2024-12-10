from setuptools import find_packages
from setuptools import setup

setup(
    name='examen_scorbot_bringup',
    version='0.0.0',
    packages=find_packages(
        include=('examen_scorbot_bringup', 'examen_scorbot_bringup.*')),
)
