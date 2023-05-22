from setuptools import setup

setup(
    name='kinepy',
    packages=[
        'kinepy', 'kinepy.interface', 'kinepy.gui', 'kinepy.objects', 'kinepy.math', 'kinepy.compilation'
    ],
    version='0.1.1',
    description='Python library dedicated to plane mechanism simulation',
    author='Loïc Chevalier - Valentin Burillier',
    url='https://github.com/valentin-burillier/kinepy',
    requires=['numpy', 'matplotlib', 'pygame'],
    keywords=['robotics', 'simulation', 'dynamics', 'kinematics', 'statics', 'inverse-kinematics', 'mechanism'],
)
