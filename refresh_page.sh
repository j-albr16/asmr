. venv/bin/activate
git checkout main
git pull
git checkout gh-pages
git merge main
make rebuild
git add .
git commit -m "refresh page"
git push
git checkout main

