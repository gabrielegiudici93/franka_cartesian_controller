# Available at setup time due to pyproject.toml
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup
import os
import sys
import subprocess


# getting the path to the current directory 
current_dir = os.path.dirname(os.path.realpath(__file__))

__version__ = "0.1"


all_library_names = []
all_library_include_dirs = []
all_library_dirs = []


    

all_library_dirs = [os.path.join(current_dir, 'third_party', 'libfranka', 'lib')]
# add pthread to the library names
# all_library_names.append("pthread")
# add unitree_sdk to the library names
all_library_names = ["franka", "pthread"]  

# add kin_dyn include to the all_library_include_dirs
all_library_include_dirs = [
    os.path.join(current_dir, 'third_party', 'libfranka', 'include'),
    '/usr/include/eigen3',
    os.path.join(current_dir, 'src'),  # Include the cartesian_franka folder
]

# The main interface is through Pybind11Extension.
# * You can add cxx_std=11/14/17, and then build_ext can be removed.
# * You can set include_pybind11=false to add the include directory yourself,
#   say from a submodule.
#
# Note:
#   Sort input source files if you glob sources to ensure bit-for-bit
#   reproducible builds (https://github.com/pybind/python_example/pull/53

sources = [
    os.path.join(current_dir, 'src', 'python.cpp'),
    # Add additional .cpp files from the cartesian_franka folder
    os.path.join(current_dir, 'src', 'cartesian_franka', 'joint_motion_generator.cpp'),
    os.path.join(current_dir, 'src', 'cartesian_franka', 'cartesian_motion_generator.cpp')
]

ext_modules = [
    Pybind11Extension("pyfranka_interface",
    	sources,
        include_dirs=all_library_include_dirs,  # path to Pinocchio and Eigen headers
        libraries=all_library_names,  #  libraries name
        library_dirs=all_library_dirs,  # path to all the libraries
        runtime_library_dirs=all_library_dirs,  # path to all the runtime libraries
        language='c++',
        extra_compile_args=['-fPIC'],
        # Example: passing in the version to the compiled code
        define_macros = [('VERSION_INFO', __version__)],
        cxx_std=14
        ),
]


setup(name="pyfranka_interface",
    version=__version__,
    author="Gabriele Giudici",
    maintainer="Gabriele Giudici",
    description="Franka Cartesian Controller — Python bindings (pybind11). "
                "Originally developed by Valerio Modugno (collaborator); "
                "maintained by Gabriele Giudici.",
    long_description="",
    ext_modules=ext_modules,
    #extras_require={"test": "pytest"},
    # Currently, build_ext only provides an optional "highest supported C++
    # level" feature, but in the future it may provide more features.
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.8"
)



