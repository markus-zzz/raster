raster: raster.cpp
	g++ -O2 -o $@ $<

clean:
	rm -f raster out.ppm
