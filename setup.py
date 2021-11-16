from setuptools import find_packages, setup

setup(
    name="pandaplotutils",
    packages=find_packages(),
    package_data={"":["__init__.py", "*.egg", "*.STL", "*.obj", "*.mtl", "*.template"]},
    version='0.1.0',
    description='panda plot utility',
    author='Me',
    lincense='MIT',
)
