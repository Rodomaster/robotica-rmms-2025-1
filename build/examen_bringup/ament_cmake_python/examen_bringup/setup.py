from setuptools import find_packages
from setuptools import setup

setup(
    name='examen_bringup',
    version='0.0.0',
    packages=find_packages(
        include=('examen_bringup', 'examen_bringup.*')),
)
