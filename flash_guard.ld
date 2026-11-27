/* Added to env:course_nav's link (platformio.ini). The run log owns flash from 0x0800D000 up (SLOT_ADDR in
   src/run_record.cpp): the program image, including the initial values of .data, must end below it. */
ASSERT(_sidata + (_edata - _sdata) <= 0x0800D000, "program overlaps the run-log flash slots (src/run_record.cpp)")
