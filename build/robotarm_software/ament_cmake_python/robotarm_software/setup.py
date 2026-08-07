from setuptools import find_packages
from setuptools import setup

setup(
    name='robotarm_software',
    version='0.0.0',
    packages=find_packages(
        include=('robotarm_software', 'robotarm_software.*')),
)
