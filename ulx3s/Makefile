TOPMOD  := Murax
CHIP := 85k
PACKAGE := CABGA381
CONSTRAINTS := ulx3s_v20_constraints
VLOGFIL := $(TOPMOD).v
PROGPATH := ~/bin/ujprog/ujprog

.PHONY: all

all: $(TOPMOD).bit

.PHONY: clean
clean:
	rm -rf $(TOPMOD).json $(TOPMOD).config $(TOPMOD).bit

$(TOPMOD).bit: $(TOPMOD).config
	ecppack $(TOPMOD).config $(TOPMOD).bit

$(TOPMOD).config: $(TOPMOD).json
	nextpnr-ecp5 \
		--$(CHIP) \
		--package $(PACKAGE) \
        --json $(TOPMOD).json \
		--lpf-allow-unconstrained \
		--lpf $(CONSTRAINTS).lpf \
		--textcfg $(TOPMOD).config

$(TOPMOD).json: $(TOPMOD).v
	yosys -q -l $(TOPMOD)_yosys.log -p "synth_ecp5 -top $(TOPMOD) -json $(TOPMOD).json" $(TOPMOD).v

prog: $(TOPMOD).bit
	sudo $(PROGPATH) $(TOPMOD).bit
