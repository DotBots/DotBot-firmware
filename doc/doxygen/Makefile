.PHONY: all clean

all:
	@mkdir -p html/_static
	@cp -R ../../_static/* html/_static/.
	@doxygen

clean:
	@rm -rf html xml
