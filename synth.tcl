create_project -in_memory -part xc7a100tcsg324-1

read_verilog -sv raster_top.sv rasterizer.sv dpram.sv
synth_design -top raster_top -part xc7a100tcsg324-1

report_utilization -file utilization.txt

puts "\n=== Resource Utilization ==="
report_utilization

exit
