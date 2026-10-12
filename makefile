CXX      ?= clang++
CXXFLAGS ?= -std=c++17 -O2 -Wall
CXXFLAGS += $(shell pkg-config --cflags opencv4) -Iinclude
LDLIBS   += $(shell pkg-config --libs opencv4)

SRCDIR   = src
BUILDDIR = build
BINDIR   = bin

PROGRAMS = calibration augmentedReality harris multiTarget staticAR

all: $(addprefix $(BINDIR)/,$(PROGRAMS))

$(BINDIR)/calibration: $(BUILDDIR)/calibration.o
$(BINDIR)/harris: $(BUILDDIR)/harris.o
$(BINDIR)/augmentedReality: $(BUILDDIR)/augmentedReality.o $(BUILDDIR)/virtualObject.o
$(BINDIR)/multiTarget: $(BUILDDIR)/multiTarget.o $(BUILDDIR)/virtualObject.o
$(BINDIR)/staticAR: $(BUILDDIR)/staticAR.o $(BUILDDIR)/virtualObject.o

$(BINDIR)/%: | $(BINDIR)
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp include/virtualObject.h | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILDDIR) $(BINDIR):
	mkdir -p $@

clean:
	rm -rf $(BUILDDIR) $(BINDIR)

.PHONY: all clean
