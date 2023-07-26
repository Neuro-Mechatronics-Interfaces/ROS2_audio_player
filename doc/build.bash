#!/usr/bin/bash

# Build the PDF documentation.
pandoc -o doc/pdf/README.pdf --metadata-file README.yaml README.md

# Build the HTML documentation.
sphinx-build -b html doc/sphinx doc/html

