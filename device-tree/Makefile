overlay: encoder.dtbo

encoder.dtbo: encoder.dts
	dtc -@ -I dts -O dtb -o $@ $<

clean:
	rm encoder.dtbo
