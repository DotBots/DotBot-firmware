# Configuration file for the Sphinx documentation builder.

import os
import glob
import shutil

project = 'DotBot-firmware'
copyright = '2023, Inria'
author = 'Alexandre Abadie'

# -- General configuration ---------------------------------------------------
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
templates_path = ['_templates']
nitpick_ignore_regex = [
    (r'c:.*', r'[u]*int\d{1,2}_t'),     # ignore int8_t, uint8_t, ...
    (r'c:.*', r'NRF_.*'),               # ignore NRF_ macros
    (r'c:.*', r'[s]*size_t'),           # ignore size_t and ssize_t 
    (r'c:.*', r'[U]*INT\d{1,2}_MAX'),   # ignore INT8_MAX, UINT8_MAX, ...
]

# -- Options for breathe ----------------------------------------------------
breathe_projects = {"DotBot-firmware": "../../doxygen/xml/"}
breathe_default_project = "DotBot-firmware"
breathe_show_include = False
breathe_domain_by_extension = {
    "h" : "c",
}

# -- Options for HTML output -------------------------------------------------
html_theme = "pydata_sphinx_theme"
html_sourcelink_suffix = ""
html_static_path = ['_static']
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
}

# -- Options for autosummary/autodoc output ------------------------------------
autosummary_generate = True
autodoc_typehints = "description"
autodoc_member_order = "groupwise"
