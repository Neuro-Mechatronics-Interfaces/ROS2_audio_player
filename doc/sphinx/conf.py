# Copyright 2022 Carnegie Mellon Neuromechatronics Lab
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.


# Package name.
# This must match the ROS / setup package name.
package_name = 'nml_task_audio'

# Author.
# ASW
author = 'Carnegie Mellon Neuromechatronics Lab'

# Master document.
# Since the MyST parser is enabled, either index.md or index.rst will work.
master_doc = 'index'

# Extract the version and release information from the ROS / setuptools package.
# https://packaging.python.org/en/latest/guides/single-sourcing-package-version/
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
from importlib.metadata import version
release = version(package_name)
version = '.'.join(release.split('.')[:2])

# Sphinx extensions.
# The MyST parser extension enables parsing of Markdown files.
extensions \
  = ['sphinx.ext.autodoc',
     'sphinx.ext.doctest',
     'sphinx.ext.todo',
     'sphinx.ext.viewcode',
     'myst_parser',
     'numpydoc'
    ]


# The default Sphinx theme is fine, but the "classic" theme provides a familiar 
# interace. See the Sphinx doumentation for builtin themes.
# https://www.sphinx-doc.org/en/master/usage/theming.html
# Some other alternatives:
#   * pydata_sphinx_theme
#   * sphinx_rtd_theme
html_theme = 'classic'

# Configure numpydoc.
# https://numpydoc.readthedocs.io/en/latest/install.html#configuration
# By default, autodoc does not show inherited class members, but numpydoc 
# changes this. For classes that inherit from ROS or some other large package, 
# this can be more confusing than useful. Disable.
numpydoc_show_inherited_class_members = False

