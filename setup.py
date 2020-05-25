from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize(["geometry.pyx", "eom.pyx", "downsample.pyx", "plotting.pyx"],
                          compiler_directives={'language_level': "3"})
)
