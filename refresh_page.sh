. venv/bin/activate
git pull origin/main
git merge main
make rebuild
git add .
git commit -m "refresh page"
git push
