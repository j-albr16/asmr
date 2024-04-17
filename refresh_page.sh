#!/bin/bash

# change to the root directory of the repository
git fetch --all
git checkout main

# check if the virtual environment exists
if [ ! -d "venv" ]; then
    echo "Virtual environment not found. Setting up .."
    python3 -m venv venv
    echo "Virtual environment set up. Activating .."
    source venv/bin/activate
    echo "Installing requirements .."
    python3 -m pip install -r requirements.txt
fi

# activate the virtual environment
source venv/bin/activate

# merge the main branch into the gh-pages branch
git checkout gh-pages
git merge main --no-edit

# rebuild the page
make rebuild

# pushcommit the changes
git add .
git commit -m "refresh page"
git push

# switch back to the main branch
git checkout main
