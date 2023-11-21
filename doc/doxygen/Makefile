.PHONY: all clean

all:
	@mkdir -p html/static
	@cp -R ../../static/* html/static/.
	@doxygen

clean:
	@rm -rf html xml
