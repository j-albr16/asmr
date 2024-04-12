. venv/bin/activate
<<<<<<< HEAD
git pull origin/main
=======
git checkout main
git pull
git checkout gh-pages
>>>>>>> main
git merge main
make rebuild
git add .
git commit -m "refresh page"
git push
<<<<<<< HEAD
=======
git checkout main

>>>>>>> main
