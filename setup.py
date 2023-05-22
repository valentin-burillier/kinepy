from setuptools import setup

setup(
    name='kinepy',
    packages=['kinepy'],
    version='0.1.0',
    description='Python library dedicated to plane mechanism simulation',
    author='Loïc Chevalier - Valentin Burillier',
    url='https://github.com/valentin-burillier/kinepy',
    keywords=['robotics', 'simulation', 'dynamics', 'kinematics', 'statics', 'inverse-kinematics', 'mechanism'],
    requires=['numpy', 'matplotlib'],
)
