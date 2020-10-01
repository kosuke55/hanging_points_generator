from __future__ import print_function

import distutils.spawn
import os
import shlex
import subprocess
import sys

from setuptools import find_packages
from setuptools import setup


version = "0.0.1"


if sys.argv[-1] == "release":
    if not distutils.spawn.find_executable("twine"):
        print(
            "Please install twine:\n\n\tpip install twine\n", file=sys.stderr,
        )
        sys.exit(1)

    commands = [
        "git tag v{:s}".format(version),
        "git push origin master --tag",
        "python setup.py sdist",
        "twine upload dist/hanging_points_generator-{:s}.tar.gz".format(
            version),

    ]
    for cmd in commands:
        print("+ {}".format(cmd))
        subprocess.check_call(shlex.split(cmd))
    sys.exit(0)


def listup_package_data():
    data_files = []
    for root, _, files in os.walk('hanging_points_generator/urdf'):
        for filename in files:
            data_files.append(
                os.path.join(
                    root[len('hanging_points_generator/'):],
                    filename))
    return data_files


setup_requires = []
install_requires = [
    'cameramodels',
    'connected-components-3d',
    'extendedos',
    'gdown',
    'opencv-python; python_version >= "3.0"',
    'opencv-python==4.2.0.32; python_version < "3.0"',
    'pathlib2',
    'pybullet',
    'pymesh',
    'scikit-robot',
    'scikit-image',
    'sklearn',
    'torch',
    'trimesh>=3.8.10'
]


setup(
    name="hanging_points_generator",
    version=version,
    description="A hanging part detector",
    author="kosuke55",
    author_email="kosuke.tnp@gmail.com",
    url="https://github.com/kosuke55/hanging_points_generator",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    license="MIT",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "Natural Language :: English",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: Implementation :: CPython",
    ],
    packages=find_packages(),
    package_data={'hanging_points_generator': listup_package_data()},
    entry_points={
        'console_scripts':
        ['check-hanging-pose=hanging_points_generator.apps.check_hanging_pose:main',
         'visualize-objects=hanging_points_generator.apps.visualize_objects:main',
         'filter-points=hanging_points_generator.apps.filter_points:main']},
    zip_safe=False,
    setup_requires=setup_requires,
    install_requires=install_requires,
)
