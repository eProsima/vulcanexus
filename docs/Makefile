# Makefile for Vulcanexus Sphinx documentation

# You can set these variables from the command line.
SPHINXBUILD   = sphinx-build
SPHINXOPTS    =
BUILDDIR      = build

# Internal variables.
SOURCEDIR      = .
HTMLDIR        = $(BUILDDIR)/html
SPELLINGDIR    = $(BUILDDIR)/spelling
ALLSPHINXOPTS  = -d $(BUILDDIR)/doctrees $(SPHINXOPTS)

# Print Makefile help
.PHONY: help
help:
	@echo "Makefile to build and test Vulcanexus documentation"
	@echo ""
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@echo "    help       Print this help"
	@echo "    clean      Remove $(BUILDDIR) directory"
	@echo "    html       Generate HTML documentation in $(HTMLDIR) directory"
	@echo "    test       Run documentation tests"

# Remove build directory
.PHONY: clean
clean:
	rm -rf $(BUILDDIR)/*

# Generate HTML documentation in $(BUILDDIR)/html
.PHONY: html
html:
	$(SPHINXBUILD) -b html $(ALLSPHINXOPTS) $(SOURCEDIR) $(HTMLDIR)
	@echo
	@echo "Build finished. The HTML pages are in $(BUILDDIR)/html"

# Run documentation tests:
#    1. Check RST files style with doc8:
#        - invalid rst format - D000
#        - no trailing whitespace - D002
#        - no tabulation for indentation - D003
#        - no carriage returns (use unix newlines) - D004
#        - no newline at end of file - D005
#    2. Spell check
.PHONY: test
test:
	doc8 --ignore D001 .
	@echo ""
	rm -rf $(SPELLINGDIR)
	@echo ""
	$(SPHINXBUILD) -Q -W --keep-going -b spelling $(ALLSPHINXOPTS) $(SOURCEDIR) $(SPELLINGDIR) || (echo "Spell checking failed:"; find $(SPELLINGDIR) -type f -exec cat {} \;; exit 1)
