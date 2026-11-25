from pathlib import Path
from setuptools import find_packages, setup

data = [
    ('share/ament_index/resource_index/packages', ['resource/cics_experiments']),
    ('share/cics_experiments', ['package.xml']),
]
for directory in ('runtime', 'analysis', 'assets', 'config', 'launch', 'docs'):
    for folder in sorted(p for p in Path(directory).rglob('*') if p.is_dir()):
        files = sorted(str(p) for p in folder.iterdir() if p.is_file() and
                       '__pycache__' not in p.parts and p.suffix != '.pyc')
        if files:
            data.append(('share/cics_experiments/' + str(folder), files))
    files = sorted(str(p) for p in Path(directory).glob('*') if p.is_file())
    if files:
        data.append(('share/cics_experiments/' + directory, files))

setup(
    name='cics_experiments', version='0.1.0',
    packages=find_packages(exclude=['test']), data_files=data,
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=False,
    maintainer='Junggeun Park', maintainer_email='jk@example.com',
    description='CICS final-platform experiment and analysis entry points',
    license='UNLICENSED',
    entry_points={'console_scripts': [
        'check = cics_experiments.cli:check_main',
        'reproduce_results = cics_experiments.cli:reproduce_main',
        'experiment = cics_experiments.cli:experiment_main',
        'isaac = cics_experiments.cli:isaac_main',
    ]},
)
