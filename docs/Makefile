# Makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXBUILD   = sphinx-build
SPHINXOPTS    =
BUILDDIR      = build

# Internal variables.
SOURCEDIR      = .
HTMLDIR        = $(BUILDDIR)/html
SPELLINGDIR    = $(BUILDDIR)/spelling
ALLSPHINXOPTS  = -d $(BUILDDIR)/doctrees $(SPHINXOPTS)

.PHONY: help
help:
	@echo "Please use \`make <target>' where <target> is one of"
	@echo "  help       Print this help"
	@echo "  clean      Remove $(BUILDDIR) directory"
	@echo "  html       Generate HTML documentation in $(HTMLDIR) directory"

# @echo "  test       to check the spelling, and the source code"

.PHONY: clean
clean:
	rm -rf $(BUILDDIR)/*

.PHONY: html
html:
	$(SPHINXBUILD) -b html $(ALLSPHINXOPTS) $(SOURCEDIR) $(HTMLDIR)
	@echo
	@echo "Build finished. The HTML pages are in $(BUILDDIR)/html."

.PHONY: test
test:
	rm -rf $(SPELLINGDIR)
	$(SPHINXBUILD) -Q -W --keep-going -b spelling $(ALLSPHINXOPTS) $(SOURCEDIR) $(SPELLINGDIR) || (echo "Spell checking failed:"; find $(SPELLINGDIR) -type f -exec cat {} \;; exit 1)
