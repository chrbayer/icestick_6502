// FPGA based SOC
.file [name="%O", type="bin", segments="Program"]
.segmentdef Data    [ start=$0000, min=$0000, max=$0fff ]
.segmentdef Code    [ start=$f000, min=$f000, max=$fff9 ]
.segmentdef Vectors [ start=$fffa, min=$fffa, max=$ffff ]
.segmentdef Program [ segments="Code, Vectors" ]
