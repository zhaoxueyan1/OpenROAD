# repair_timing -max_utilization_increase resolves against entry utilization
source "helpers.tcl"
set test_name repair_max_util_increase

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def
create_clock [get_ports clk] -name core_clock -period 2

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set entry_util [rsz::utilization]

array set keys {-max_utilization_increase 5}
set relative_max_util [rsz::parse_repair_max_util keys]
set expected_max_util [expr {$entry_util + 0.05}]
if {abs($relative_max_util - $expected_max_util) > 1e-12} {
  error "relative max utilization did not use entry utilization"
}

array unset keys
array set keys {-max_utilization 70}
set absolute_max_util [rsz::parse_repair_max_util keys]
if {abs($absolute_max_util - 0.70) > 1e-12} {
  error "absolute max utilization semantics changed"
}

array unset keys
array set keys {-max_utilization 70 -max_utilization_increase 5}
if {![catch {rsz::parse_repair_max_util keys}]} {
  error "absolute and relative max utilization must be mutually exclusive"
}

puts "entry_utilization=[format %.6f [expr {$entry_util * 100.0}]]"
puts "relative_utilization=[format %.6f [expr {$relative_max_util * 100.0}]]"
puts "absolute_utilization=[format %.6f [expr {$absolute_max_util * 100.0}]]"
