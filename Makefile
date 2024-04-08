
clean:
	rm -rf _build
	rm -rf docs
	jupyter-book clean .

build:
	jupyter-book build --all .;
	cp -r _build/html docs
	touch docs/.nojekyll
	# make include

rebuild:
	make clean
	make build

url:
	python3 jupyterhub.py

serve:
	python3 api/main.py

pid:
	sudo lsof -i :8000

include: _build/html/intro.html chat/chat_button.html
	python3 chat/insert-html.py
