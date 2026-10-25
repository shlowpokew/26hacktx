test tests:
	python3 -m unittest discover -s tests
	cd arduino_uno_rs/brain && cargo test

.PHONY: test tests
