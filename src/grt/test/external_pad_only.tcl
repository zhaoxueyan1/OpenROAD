# An external PAD terminal with no core-facing iterm belongs to the
# package/padframe flow and must not enter core global routing.
source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_lef external_pad_only.lef
read_def external_pad_only.def

set_routing_layers -signal metal1-metal9
global_route -verbose
