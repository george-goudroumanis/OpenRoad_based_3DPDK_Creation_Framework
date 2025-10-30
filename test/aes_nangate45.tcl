# aes flow pipe cleaner
source "test/helpers.tcl"
source "test/flow_helpers.tcl"
source "test/Nangate45/Nangate45.vars"

set design "aes"
set top_module "aes_cipher_top"
set synth_verilog "test/aes_nangate45.v"
set sdc_file "test/aes_nangate45.sdc"
set die_area {0 0 1020 920.8}
set core_area {10 12 1010 911.2}

set cap_margin 20

include -echo "flow.tcl"
