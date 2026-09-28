
CXX = g++
CXXFLAGS = -std=c++14 -O2

.PHONY : all

all : serial_io_ctrl
	@true

serial_io_ctrl: serial_io_ctrl.cpp
	$(CXX) $(CXXFLAGS) serial_io_ctrl.cpp -o $@

clean:
	-rm serial_io_ctrl
