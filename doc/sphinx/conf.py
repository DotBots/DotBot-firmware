# Configuration file for the Sphinx documentation builder.

import glob
import os
import shutil
import subprocess
import sys


project = 'DotBot-firmware'
copyright = '2023, Inria'
author = 'Alexandre Abadie'

# -- General configuration ----------------------------------------------------
extensions = [
    'breathe',
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.githubpages",
    "sphinx.ext.graphviz",
    "sphinx.ext.inheritance_diagram",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
]

language = "en"
tls_verify = False
templates_path = ['_templates']
exclude_patterns = ["_build"]
nitpick_ignore_regex = [
    (r'c:.*', r'[u]*int\d{1,2}_t'),     # ignore int8_t, uint8_t, ...
    (r'c:.*', r'NRF_.*'),               # ignore NRF_ macros
    (r'c:.*', r'[s]*size_t'),           # ignore size_t and ssize_t 
    (r'c:.*', r'[U]*INT\d{1,2}_MAX'),   # ignore INT8_MAX, UINT8_MAX, ...
]

# -- Options for breathe ------------------------------------------------------
breathe_projects = {"DotBot-firmware": "../doxygen/xml/"}
breathe_default_project = "DotBot-firmware"
breathe_show_include = False
breathe_domain_by_extension = {
    "h" : "c",
}

myst_enable_extensions = ["html_image"]

# -- Options for HTML output --------------------------------------------------
html_theme = "pydata_sphinx_theme"
html_sourcelink_suffix = ""
html_static_path = ["_static"]

# Define the json_url for our version switcher.
json_url = "https://dotbot-firmware.readthedocs.io/en/latest/_static/switcher.json"
rtd_version = os.environ.get("READTHEDOCS_VERSION")
rtd_version_type = os.environ.get("READTHEDOCS_VERSION_TYPE")
rtd_git_identifier = os.environ.get("READTHEDOCS_GIT_IDENTIFIER")
# If READTHEDOCS_VERSION doesn't exist, we're not on RTD
# If it is an integer, we're in a PR build and the version isn't correct.
# If it's "latest" → change to "dev" (that's what we want the switcher to call it)
if not rtd_version or rtd_version.isdigit() or rtd_version == "latest":
    rtd_version = "dev"
    json_url = "_static/switcher.json"
elif rtd_version == "stable":
    rtd_version = f"{rtd_git_identifier}"
elif rtd_version_type == "tag":
    rtd_version = f"{rtd_git_identifier}"

html_theme_options = {
    "external_links": [
        {
            "url": "https://github.com/DotBots/PyDotBot",
            "name": "PyDotBot",
            "attributes": {
               "target" : "_blank",
               "rel" : "noopener me",
            },
        },
        {
            "url": "https://github.com/DotBots/DotBot-hardware",
            "name": "DotBot hardware",
            "attributes": {
               "target" : "_blank",
               "rel" : "noopener me",
            },
        },
    ],
    "icon_links": [
         {
            "name": "GitHub",
            "url": "https://github.com/DotBots/DotBot-firmware",
            "icon": "fa-brands fa-github",
        },
    ],
    "header_links_before_dropdown": 4,
    "logo": {
        "text": "DotBot firmware",
    },
    "navbar_align": "left",
    "navbar_center": ["version-switcher", "navbar-nav"],
    "switcher": {
        "json_url": json_url,
        "version_match": rtd_version,
    },
    "footer_start": ["copyright"],
    "footer_center": ["sphinx-version"],
}

# -- Options for autosummary/autodoc output -----------------------------------
autosummary_generate = True
autodoc_typehints = "description"
autodoc_member_order = "groupwise"

# Hook for building doxygen documentation -------------------------------------

def run_doxygen(app):
    """Run the doxygen make command."""
    doxygen_path = "../doxygen"
    try:
        retcode = subprocess.call(f"make -C {doxygen_path}", shell=True)
        if retcode < 0:
            sys.stderr.write(f"doxygen terminated by signal {-retcode}")
    except OSError as e:
        sys.stderr.write(f"doxygen execution failed: {e}")

# Hook for generating linked README.md files --------------------------------------------

README_INCLUDE_TEMPLATE = """```{{include}} {path_to_readme}
:relative-images:
:relative-docs: ../../
```
"""

def generate_readme(app, prefix, dest):
    projects_dir = os.path.join(app.srcdir, "../../projects/")
    projects = [os.path.basename(project) for project in glob.glob(f"{projects_dir}/{prefix}*")]
    output_dir = os.path.join(app.srcdir, dest)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    for project in projects:
        with open(os.path.join(output_dir, f"{project}.md"), "w") as f:
            f.write(README_INCLUDE_TEMPLATE.format(path_to_readme=f"../../../projects/{project}/README.md"))


def generate_projects_readme(app):
    for prefix, dest in [("01", "_examples"), ("03app", "_projects")]:
        generate_readme(app, prefix, dest)


API_INCLUDE_TEMPLATE = """{title}
=================================

.. doxygengroup:: {module}
.. doxygenfile:: {header}

"""
EXCLUDE_MODULES = [
    "board_config",
    "soft_ed25519",
    "soft_edsign",
    "soft_f25519",
    "soft_fprime",
    "soft_sha256",
    "soft_sha512",
]


def generate_api_files(app):
    output_dir = os.path.join(app.srcdir, "_api")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    for module in ["bsp", "crypto", "drv"]:
        module_dir = os.path.join(app.srcdir, f"../../{module}/")
        submodules = [os.path.basename(project).split(".")[0] for project in glob.glob(f"{module_dir}/*.h")]
        submodules = [module for module in submodules if module not in EXCLUDE_MODULES]
        for submodule in submodules:
            with open(os.path.join(output_dir, f"{module}_{submodule}.rst"), "w") as f:
                f.write(
                    API_INCLUDE_TEMPLATE.format(
                        title=f"{submodule.capitalize()}",
                        module=f"{module}_{submodule}",
                        header=f"{module}/{submodule}.h"
                    )
                )


def setup(app):
    """Add hook for building doxygen documentation."""
    app.connect("builder-inited", run_doxygen)
    app.connect("builder-inited", generate_api_files)
    app.connect("builder-inited", generate_projects_readme)
