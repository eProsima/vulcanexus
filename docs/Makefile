# Makefile for Vulcanexus Sphinx documentation

# You can set these variables from the command line.
SPHINXBUILD   = sphinx-build
SPHINXOPTS    =
BUILDDIR      = build
RESOURCESDIR  = resources

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
	@echo "    help         Print this help"
	@echo "    clean        Remove $(BUILDDIR) directory"
	@echo "    html         Generate HTML documentation in $(HTMLDIR) directory"
	@echo "    test         Run all documentation tests"
	@echo "    list-tests   List documentation test"

# Remove build directory
.PHONY: clean
clean:
	rm -rf $(BUILDDIR)/*

# Render terminalizer gifs
.PHONY: terminalizer
terminalizer:
	xvfb-run -a terminalizer render $(RESOURCESDIR)/terminalizer/test.yml -o $(RESOURCESDIR)/terminalizer/test
	xvfb-run -a terminalizer render $(RESOURCESDIR)/terminalizer/core/ros2_pub_sub/pubsub_workspace_terminalizer.yml -o $(RESOURCESDIR)/terminalizer/pubsub_workspace_terminalizer
	@echo
	@echo "Terminalizer render finished. Generated gifs are in $(RESOURCESDIR)/terminalizer/"

# Generate HTML documentation in $(BUILDDIR)/html
.PHONY: html
html:
	$(SPHINXBUILD) -b html $(ALLSPHINXOPTS) $(SOURCEDIR) $(HTMLDIR)
	@echo
	@echo "Build finished. The HTML pages are in $(BUILDDIR)/html"

.PHONY: list-tests
list-tests:
	@echo "Vulcanexus documentation individual tests."
	@echo "Run an individual test with: make [test-name]"
	@echo ""
	@echo "Test names:"
	@LC_ALL=C $(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print "   "$$1}}' | sort | grep -e 'test-'

# Check RST files style with doc8:
#     - invalid rst format - D000
#     - no trailing whitespace - D002
#     - no tabulation for indentation - D003
#     - no carriage returns (use unix newlines) - D004
#     - no newline at end of file - D005
.PHONY: test-style
test-style:
	doc8 --ignore D001 .
	@echo ""

# Spell check
.PHONY: test-spelling
test-spelling:
	rm -rf $(SPELLINGDIR)
	@echo ""
	$(SPHINXBUILD) -Q -W --keep-going -b spelling $(ALLSPHINXOPTS) $(SOURCEDIR) $(SPELLINGDIR) || (echo "Spell checking failed:"; find $(SPELLINGDIR) -type f -exec cat {} \;; exit 1)

# Run documentation tests
.PHONY: test
test: test-style test-spelling
