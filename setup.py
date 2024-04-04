# -*- coding: utf-8 -*-

import sys
from setuptools import setup, find_namespace_packages

# List all package dependencies installable from the PyPI here. You should at least differentiate
# between Unix and Windows systems.
# ONLY LIST DEPENDENCIES THAT ARE DIRECTLY USED BY THIS PACKAGE (no inherited dependencies from
# e.g. qudi-core)
unix_dep = [
    'qudi-core>=1.4.1',
    'entrypoints>=0.4',
    'fysom>=2.1.6',
    'lmfit>=1.0.3',
    'lxml>=4.9.1',
    'matplotlib>=3.6.0',
    'nidaqmx>=0.5.7',
    'numpy>=1.23.3',
    'pyqtgraph>=0.13.1',
    'PySide2>=5.15.2',
    'PyVisa>=1.12.0',
    'scipy>=1.9.1',
    'zaber_motion>=2.14.6'
]

windows_dep = [
    'qudi-core>=1.4.1',
    'entrypoints>=0.4',
    'fysom>=2.1.6',
    'lmfit>=1.0.3',
    'lxml>=4.9.1',
    'matplotlib>=3.6.0',
    'nidaqmx>=0.5.7',
    'numpy>=1.23.3',
    'pyqtgraph>=0.13.1',
    'PySide2>=5.15.2',
    'PyVisa>=1.12.0',
    'scipy>=1.9.1',
    'zaber_motion>=2.14.6'
]

# The version number of this package is derived from the content of the "VERSION" file located in
# the repository root. Please refer to PEP 440 (https://www.python.org/dev/peps/pep-0440/) for
# version number schemes to use.
with open('VERSION', 'r') as file:
    version = file.read().strip()

# The README.md file content is included in the package metadata as long description and will be
# automatically shown as project description on the PyPI once you release it there.
with open('README.md', 'r') as file:
    long_description = file.read()

# Please refer to https://docs.python.org/3/distutils/setupscript.html for documentation about the
# setup function.
#
# 1. Specify a package name. If you plan on releasing this on the PyPI, choose a name that is not
#    yet taken. Since it is a qudi addon package, it's a good idea to prefix it with "qudi-".
# 2. List data files to be distributed with the package. Do NOT include for example "tests" and
#    "docs" directories.
# 3. Add a short(!) description of the package
# 4. Add your projects homepage URL
# 5. Add keywords/tags for your package to be found more easily
# 6. Make sure your license tag matches the LICENSE (and maybe LICENSE.LESSER) file distributed
#    with your package (default: GNU Lesser General Public License v3)
setup(
    name='qudi-sinaps',  # Choose a custom name
    version=version,  # Automatically deduced from "VERSION" file (see above)
    packages=find_namespace_packages(where='src'),  # This should be enough for 95% of the use-cases
    package_dir={'': 'src'},  # same
    package_data={'qudi'    : ['default.cfg'],
                  'qudi.gui': ['*.ui', '*/*.ui'],
                  'qudi.hardware': ['*.h', '*/*.h'],
                  },  # include data files
    description='A template package for qudi addons.',  # Meaningful short(!) description
    long_description=long_description,  # Detailed description is taken from "README.md" file
    long_description_content_type='text/markdown',  # Content type of "README.md" file
    url='https://github.com/Ulm-IQO/qudi-addon-template',  # URL pointing to your project page
    keywords=['qudi',             # Add tags here to be easier found by searches, e.g. on PyPI
              'experiment',
              'measurement',
              'framework',
              'lab',
              'laboratory',
              'instrumentation',
              'instrument',
              'modular'
              ],
    classifiers=['Development Status :: 4 - Beta',

                 'Environment :: Win32 (MS Windows)',
                 'Environment :: X11 Applications',
                 'Environment :: MacOS X',

                 'Intended Audience :: Science/Research',
                 'Intended Audience :: End Users/Desktop',

                 'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',

                 'Natural Language :: English',

                 'Operating System :: Microsoft :: Windows :: Windows 8',
                 'Operating System :: Microsoft :: Windows :: Windows 8.1',
                 'Operating System :: Microsoft :: Windows :: Windows 10',
                 'Operating System :: MacOS :: MacOS X',
                 'Operating System :: Unix',
                 'Operating System :: POSIX :: Linux',

                 'Programming Language :: Python :: 3.8',
                 'Programming Language :: Python :: 3.9',
                 'Programming Language :: Python :: 3.10',

                 'Topic :: Scientific/Engineering',
                 ],
    license='LGPLv3',  # License tag
    install_requires=windows_dep if sys.platform == 'win32' else unix_dep,  # package dependencies
    python_requires='>=3.8, <3.11',  # Specify compatible Python versions
    zip_safe=False
)
